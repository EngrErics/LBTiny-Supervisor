/*
 * main.c - LBTiny Supervisor, Step 3
 * ==================================
 *
 * Goal: Receive a binary payload from the host IDE, compute its CRC using
 * the verified peripheral path, and send back a response containing both
 * the host-claimed length and the computed CRC. This is the protocol that
 * the IDE's transfer dialog (Step 4) will speak.
 *
 * Protocol:
 *   IDE -> Nucleo:  0xA5  [len_le_4B]  [payload...]
 *   Nucleo -> IDE:  0x5A  [recv_len_le_4B]  [crc32_le_4B]
 *
 * Where:
 *   - len_le_4B    = host's declared payload size, little-endian uint32
 *   - recv_len_le_4B = echoes the host's declared length verbatim (NOT the
 *                    number of bytes we stored). This lets the host detect
 *                    overflow as "lengths match but CRC mismatches" vs
 *                    truncation as "lengths disagree."
 *   - crc32_le_4B  = STM32F4 CRC over the bytes we stored, max 4096
 *
 * State machine:
 *   WAIT_SYNC     -> got 0xA5?         -> READ_LEN
 *   READ_LEN      -> got 4 bytes?      -> if len > 0: READ_PAYLOAD
 *                                         else:       READY
 *   READ_PAYLOAD  -> got len bytes?    -> READY
 *                 -> hit MAX limit?    -> DRAIN_OVERFLOW
 *                    (or stale > 1.5s) -> TIMEOUT -> WAIT_SYNC
 *   DRAIN_OVERFLOW-> declared bytes done? -> READY
 *   READY         -> main loop: compute CRC, send response, -> WAIT_SYNC
 *
 * Overflow handling: if host declares len > 4096, we still drain all
 * declared bytes off the wire (only storing the first 4096) so the next
 * transfer's framing isn't lost. CRC is over the stored portion only.
 *
 * Inter-byte timeout: 1500 ms. If we're mid-transfer and no bytes arrive
 * for that long, we abort to WAIT_SYNC. The host can retry.
 *
 * Byte-to-word packing for CRC: little-endian (matches Step 1/2 convention).
 */

#include "stm32f4xx_hal.h"
#include <string.h>
#include <stdio.h>
#include <stdarg.h>
#include <stdbool.h>

/* ------------------------------------------------------------------------ */
/* Protocol constants                                                       */
/* ------------------------------------------------------------------------ */
#define SYNC_HOST_TO_NUCLEO   0xA5u
#define SYNC_NUCLEO_TO_HOST   0x5Au
#define MAX_PAYLOAD_BYTES     4096u
#define TIMEOUT_MS            1500u

/* ------------------------------------------------------------------------ */
/* Peripheral handles                                                       */
/* ------------------------------------------------------------------------ */
static UART_HandleTypeDef huart2;
static CRC_HandleTypeDef  hcrc;

/* ------------------------------------------------------------------------ */
/* Receive state machine                                                    */
/* ------------------------------------------------------------------------ */
typedef enum {
    RX_STATE_WAIT_SYNC = 0,
    RX_STATE_READ_LEN,
    RX_STATE_READ_PAYLOAD,
    RX_STATE_DRAIN_OVERFLOW,
    RX_STATE_READY,        /* full transfer received, main loop will process */
} rx_state_t;

static volatile rx_state_t rx_state = RX_STATE_WAIT_SYNC;
static volatile uint32_t   last_rx_tick = 0;   /* HAL_GetTick() of last byte */

/* Staging buffers used by the RX ISR */
static uint8_t  sync_byte_buf;
static uint8_t  len_bytes_buf[4];
static uint8_t  payload_byte_buf;          /* single-byte arming for payload */
static uint8_t  drain_byte_buf;            /* single-byte arming for overflow drain */

/* Transfer payload buffer + bookkeeping */
static uint8_t  rx_payload[MAX_PAYLOAD_BYTES];
static volatile uint32_t declared_len = 0;     /* what host said */
static volatile uint32_t stored_count = 0;     /* what we actually kept */
static volatile uint32_t drained_count = 0;    /* bytes drained beyond MAX */

/* ------------------------------------------------------------------------ */
/* Forward declarations                                                     */
/* ------------------------------------------------------------------------ */
static void SystemClock_Config(void);
static void GPIO_Init(void);
static void USART2_Init(void);
static void CRC_Init(void);

static uint32_t compute_crc(const uint8_t *data, uint32_t len);
static void uart_print(const char *s);
static void uart_printf(const char *fmt, ...);
static void uart_send_raw(const uint8_t *data, uint16_t len);

static void rx_reset_to_wait_sync(void);
static void rx_process_transfer(void);

/* ------------------------------------------------------------------------ */
/* main                                                                     */
/* ------------------------------------------------------------------------ */
int main(void)
{
    HAL_Init();
    SystemClock_Config();
    GPIO_Init();
    USART2_Init();
    CRC_Init();

    HAL_Delay(500);
    uart_print("\r\n");
    uart_print("============================================================\r\n");
    uart_print("LBTiny Supervisor - Step 3: receive protocol [v2]\r\n");
    uart_print("Protocol: 0xA5 [len_le_4B] [payload]  ->  0x5A [len_le_4B] [crc_le_4B]\r\n");
    uart_print("Max payload: 4096 bytes.  Inter-byte timeout: 1500 ms.\r\n");
    uart_print("============================================================\r\n");
    uart_print("\r\n");

    /* Prime the receive: wait for sync byte */
    rx_reset_to_wait_sync();

    uint32_t blink_tick = HAL_GetTick();

    while (1) {
        /* Heartbeat LED at 1 Hz so we know the chip is alive */
        if ((HAL_GetTick() - blink_tick) >= 500u) {
            HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
            blink_tick = HAL_GetTick();
        }

        /* Inter-byte timeout: if we're mid-transfer and stale, abort */
        if (rx_state == RX_STATE_READ_LEN ||
            rx_state == RX_STATE_READ_PAYLOAD ||
            rx_state == RX_STATE_DRAIN_OVERFLOW) {
            if ((HAL_GetTick() - last_rx_tick) > TIMEOUT_MS) {
                uart_print("[timeout] transfer aborted, returning to WAIT_SYNC\r\n");
                /* Cancel the pending receive request before re-arming. */
                HAL_UART_AbortReceive_IT(&huart2);
                rx_reset_to_wait_sync();
            }
        }

        /* Process a fully-received transfer */
        if (rx_state == RX_STATE_READY) {
            rx_process_transfer();
            rx_reset_to_wait_sync();
        }
    }
}

/* ------------------------------------------------------------------------ */
/* Receive state helpers                                                    */
/* ------------------------------------------------------------------------ */

/*
 * rx_reset_to_wait_sync - return the receiver to the idle state and arm
 * a 1-byte receive waiting for the sync byte.
 */
static void rx_reset_to_wait_sync(void)
{
    rx_state      = RX_STATE_WAIT_SYNC;
    declared_len  = 0;
    stored_count  = 0;
    drained_count = 0;
    last_rx_tick  = HAL_GetTick();
    HAL_UART_Receive_IT(&huart2, &sync_byte_buf, 1);
}

/*
 * rx_process_transfer - compute CRC over received payload, send response.
 * Called from the main loop when rx_state == RX_STATE_READY.
 *
 * Response format: 0x5A [declared_len_le_4B] [crc_le_4B] = 9 bytes total.
 * The declared length is echoed verbatim (NOT the stored count), so the
 * host can distinguish overflow (lengths match, CRC mismatches) from
 * truncation/drop (lengths disagree).
 */
static void rx_process_transfer(void)
{
    uint32_t crc = compute_crc(rx_payload, stored_count);

    uint8_t response[9];
    response[0] = SYNC_NUCLEO_TO_HOST;
    response[1] = (uint8_t)(declared_len      & 0xFFu);
    response[2] = (uint8_t)((declared_len >> 8)  & 0xFFu);
    response[3] = (uint8_t)((declared_len >> 16) & 0xFFu);
    response[4] = (uint8_t)((declared_len >> 24) & 0xFFu);
    response[5] = (uint8_t)(crc           & 0xFFu);
    response[6] = (uint8_t)((crc >> 8)    & 0xFFu);
    response[7] = (uint8_t)((crc >> 16)   & 0xFFu);
    response[8] = (uint8_t)((crc >> 24)   & 0xFFu);
    uart_send_raw(response, sizeof(response));

    /*
     * NOTE: No human-readable diagnostic line here. Sending text after the
     * binary response races with the next transfer's incoming bytes and can
     * trigger a UART overrun that fires HAL_UART_ErrorCallback and resets
     * the state machine mid-transfer. The binary response is sufficient;
     * the host script prints all relevant values on its side.
     *
     * If you need the diagnostic line during dev, gate it behind a flag:
     *   #ifdef LBTINY_VERBOSE_XFER
     *   uart_printf("[xfer] declared=%lu ...\r\n", ...);
     *   #endif
     * and only enable it when running the monitor manually (not the test script).
     */
}

/* ------------------------------------------------------------------------ */
/* HAL receive callback - drives the state machine                          */
/* ------------------------------------------------------------------------ */
/*
 * Keep ISR work short: update state, re-arm next receive. No printf, no
 * CRC, no blocking calls in here. Heavy work is deferred to the main loop
 * (which polls for RX_STATE_READY).
 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart != &huart2) {
        return;
    }
    last_rx_tick = HAL_GetTick();

    switch (rx_state) {

    case RX_STATE_WAIT_SYNC:
        if (sync_byte_buf == SYNC_HOST_TO_NUCLEO) {
            rx_state = RX_STATE_READ_LEN;
            HAL_UART_Receive_IT(&huart2, len_bytes_buf, 4);
        } else {
            /* Wrong byte: re-arm and wait. Stray bytes (terminal noise,
             * resync after a timeout, etc.) just get dropped silently. */
            HAL_UART_Receive_IT(&huart2, &sync_byte_buf, 1);
        }
        break;

    case RX_STATE_READ_LEN:
        declared_len = ((uint32_t)len_bytes_buf[0])
                     | (((uint32_t)len_bytes_buf[1]) << 8)
                     | (((uint32_t)len_bytes_buf[2]) << 16)
                     | (((uint32_t)len_bytes_buf[3]) << 24);
        stored_count  = 0;
        drained_count = 0;
        if (declared_len == 0) {
            /* Empty payload - skip straight to processing */
            rx_state = RX_STATE_READY;
        } else {
            rx_state = RX_STATE_READ_PAYLOAD;
            HAL_UART_Receive_IT(&huart2, &payload_byte_buf, 1);
        }
        break;

    case RX_STATE_READ_PAYLOAD:
        if (stored_count < MAX_PAYLOAD_BYTES) {
            rx_payload[stored_count] = payload_byte_buf;
        }
        stored_count++;

        if (stored_count >= declared_len) {
            /* Got everything declared */
            rx_state = RX_STATE_READY;
        } else if (stored_count >= MAX_PAYLOAD_BYTES) {
            /* Buffer full but more bytes still coming - drain them so we
             * stay aligned on the next transfer's framing. */
            rx_state = RX_STATE_DRAIN_OVERFLOW;
            HAL_UART_Receive_IT(&huart2, &drain_byte_buf, 1);
        } else {
            /* More to come, stay in this state */
            HAL_UART_Receive_IT(&huart2, &payload_byte_buf, 1);
        }
        break;

    case RX_STATE_DRAIN_OVERFLOW:
        drained_count++;
        if ((stored_count + drained_count) >= declared_len) {
            rx_state = RX_STATE_READY;
        } else {
            HAL_UART_Receive_IT(&huart2, &drain_byte_buf, 1);
        }
        break;

    case RX_STATE_READY:
        /* Shouldn't happen - main loop should have consumed READY before
         * we arm another receive. Defensive: ignore. */
        break;
    }
}

/*
 * HAL_UART_ErrorCallback - HAL drops us here on framing/overrun/etc.
 * Most likely during dev: PC closing the COM port mid-transfer.
 * Reset cleanly and wait for the next sync.
 */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart == &huart2) {
        rx_reset_to_wait_sync();
    }
}

/* ------------------------------------------------------------------------ */
/* CRC computation - unchanged from Step 2 (verified path)                  */
/* ------------------------------------------------------------------------ */
static uint32_t compute_crc(const uint8_t *data, uint32_t len)
{
    /* Word buffer sized for MAX_PAYLOAD_BYTES + padding. ~4 KB - fine
     * on the F446's 128 KB SRAM. Marked static so it doesn't sit on the
     * (small) main-thread stack. */
    static uint32_t words[(MAX_PAYLOAD_BYTES + 3) / 4 + 1];

    uint32_t padded_len = (len + 3u) & ~3u;
    uint32_t num_words = padded_len / 4u;

    /* Pack bytes into 32-bit little-endian words, padding with 0xFF beyond
     * the actual data length (matches Python's stm32_crc.py convention). */
    for (uint32_t w = 0; w < num_words; w++) {
        uint32_t base = w * 4u;
        uint8_t b0 = (base + 0 < len) ? data[base + 0] : 0xFFu;
        uint8_t b1 = (base + 1 < len) ? data[base + 1] : 0xFFu;
        uint8_t b2 = (base + 2 < len) ? data[base + 2] : 0xFFu;
        uint8_t b3 = (base + 3 < len) ? data[base + 3] : 0xFFu;
        words[w] = ((uint32_t)b0)
                 | (((uint32_t)b1) << 8)
                 | (((uint32_t)b2) << 16)
                 | (((uint32_t)b3) << 24);
    }

    /*
     * Reset CRC peripheral to initial value 0xFFFFFFFF.
     * __DSB() ensures the reset propagates before we read/use DR.
     * Without it, at low optimization a read races the peripheral and
     * returns stale data (Step 2 diagnostic confirmed this).
     */
    __HAL_CRC_DR_RESET(&hcrc);
    __DSB();

    if (num_words == 0) {
        return hcrc.Instance->DR;
    }
    return HAL_CRC_Accumulate(&hcrc, words, num_words);
}

/* ------------------------------------------------------------------------ */
/* UART output helpers                                                      */
/* ------------------------------------------------------------------------ */
static void uart_send_raw(const uint8_t *data, uint16_t len)
{
    HAL_UART_Transmit(&huart2, (uint8_t *)data, len, HAL_MAX_DELAY);
}

static void uart_print(const char *s)
{
    HAL_UART_Transmit(&huart2, (uint8_t *)s, (uint16_t)strlen(s), HAL_MAX_DELAY);
}

static void uart_printf(const char *fmt, ...)
{
    char buf[160];
    va_list args;
    va_start(args, fmt);
    int n = vsnprintf(buf, sizeof(buf), fmt, args);
    va_end(args);
    if (n > 0) {
        HAL_UART_Transmit(&huart2, (uint8_t *)buf,
                          (uint16_t)((n < (int)sizeof(buf)) ? n : (int)sizeof(buf) - 1),
                          HAL_MAX_DELAY);
    }
}

/* ------------------------------------------------------------------------ */
/* Peripheral initialization (unchanged from Step 2, except USART2 IRQ)     */
/* ------------------------------------------------------------------------ */
static void SystemClock_Config(void)
{
    RCC_OscInitTypeDef osc = {0};
    RCC_ClkInitTypeDef clk = {0};

    __HAL_RCC_PWR_CLK_ENABLE();
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

    osc.OscillatorType = RCC_OSCILLATORTYPE_HSI;
    osc.HSIState       = RCC_HSI_ON;
    osc.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    osc.PLL.PLLState   = RCC_PLL_OFF;
    if (HAL_RCC_OscConfig(&osc) != HAL_OK) { while (1) {} }

    clk.ClockType = RCC_CLOCKTYPE_HCLK   | RCC_CLOCKTYPE_SYSCLK
                  | RCC_CLOCKTYPE_PCLK1  | RCC_CLOCKTYPE_PCLK2;
    clk.SYSCLKSource   = RCC_SYSCLKSOURCE_HSI;
    clk.AHBCLKDivider  = RCC_SYSCLK_DIV1;
    clk.APB1CLKDivider = RCC_HCLK_DIV1;
    clk.APB2CLKDivider = RCC_HCLK_DIV1;
    if (HAL_RCC_ClockConfig(&clk, FLASH_LATENCY_0) != HAL_OK) { while (1) {} }
}

static void GPIO_Init(void)
{
    __HAL_RCC_GPIOA_CLK_ENABLE();
    GPIO_InitTypeDef gpio = {0};
    gpio.Pin   = GPIO_PIN_5;
    gpio.Mode  = GPIO_MODE_OUTPUT_PP;
    gpio.Pull  = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOA, &gpio);
}

static void USART2_Init(void)
{
    __HAL_RCC_USART2_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();

    GPIO_InitTypeDef gpio = {0};
    gpio.Pin       = GPIO_PIN_2 | GPIO_PIN_3;
    gpio.Mode      = GPIO_MODE_AF_PP;
    gpio.Pull      = GPIO_PULLUP;
    gpio.Speed     = GPIO_SPEED_FREQ_HIGH;
    gpio.Alternate = GPIO_AF7_USART2;
    HAL_GPIO_Init(GPIOA, &gpio);

    huart2.Instance          = USART2;
    huart2.Init.BaudRate     = 115200;
    huart2.Init.WordLength   = UART_WORDLENGTH_8B;
    huart2.Init.StopBits     = UART_STOPBITS_1;
    huart2.Init.Parity       = UART_PARITY_NONE;
    huart2.Init.Mode         = UART_MODE_TX_RX;
    huart2.Init.HwFlowCtl    = UART_HWCONTROL_NONE;
    huart2.Init.OverSampling = UART_OVERSAMPLING_16;
    if (HAL_UART_Init(&huart2) != HAL_OK) { while (1) {} }

    /* Enable USART2 interrupt - required for HAL_UART_Receive_IT to work */
    HAL_NVIC_SetPriority(USART2_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(USART2_IRQn);
}

static void CRC_Init(void)
{
    __HAL_RCC_CRC_CLK_ENABLE();
    hcrc.Instance = CRC;
    if (HAL_CRC_Init(&hcrc) != HAL_OK) { while (1) {} }
}

/* ------------------------------------------------------------------------ */
/* IRQ handlers                                                             */
/* ------------------------------------------------------------------------ */
void SysTick_Handler(void)
{
    HAL_IncTick();
}

void USART2_IRQHandler(void)
{
    HAL_UART_IRQHandler(&huart2);
}