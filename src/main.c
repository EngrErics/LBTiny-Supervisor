/*
 * main.c - LBTiny Supervisor, Step 3 / v3 protocol
 * ================================================
 *
 * Receive binary payloads, compute CRCs, send back framed responses.
 * Protocol now uses a command code byte so we can add more commands
 * (flash read, sector erase, status query, etc.) without breaking framing.
 *
 * Protocol:
 *   PC -> Nucleo:  0xA5  [cmd_1B]  [len_le_4B]  [payload...]
 *   Nucleo -> PC:  0x5A  [cmd_1B]  [status_1B]  [data_len_le_4B]  [data...]
 *
 * Commands implemented:
 *   CMD_TRANSFER_CRC (0x01):
 *      payload = bytes to be CRC'd (declared by len_le_4B)
 *      response data = [declared_len_le_4B] [crc_le_4B]  (8 bytes)
 *      status: 0x00 = OK
 *              0x01 = overflow (declared_len > MAX, CRC over stored portion)
 *
 * Future commands (not yet implemented):
 *   CMD_PING (0x02), CMD_FLASH_READ (0x10), CMD_FLASH_ERASE (0x11), etc.
 *   Unknown commands return status=0xFF with data_len=0.
 *
 * State machine:
 *   WAIT_FRAME    -> got 0xA5?         -> READ_CMD
 *   READ_CMD      -> got 1 byte?       -> READ_LEN
 *   READ_LEN      -> got 4 bytes?      -> if len > 0: READ_PAYLOAD
 *                                         else:       READY
 *   READ_PAYLOAD  -> got len bytes?    -> READY
 *                 -> hit MAX limit?    -> DRAIN_OVERFLOW
 *                    (or stale > 1.5s) -> TIMEOUT -> WAIT_FRAME
 *   DRAIN_OVERFLOW-> declared bytes done? -> READY
 *   READY         -> main loop: dispatch command, send response, -> WAIT_FRAME
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

#define CMD_TRANSFER_CRC      0x01u    /* echo + CRC the payload */
/* Future commands reserve their own codes here */

#define STATUS_OK             0x00u
#define STATUS_OVERFLOW       0x01u
#define STATUS_UNKNOWN_CMD    0xFFu

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
    RX_STATE_WAIT_FRAME = 0,
    RX_STATE_READ_CMD,
    RX_STATE_READ_LEN,
    RX_STATE_READ_PAYLOAD,
    RX_STATE_DRAIN_OVERFLOW,
    RX_STATE_READY,
} rx_state_t;

static volatile rx_state_t rx_state = RX_STATE_WAIT_FRAME;
static volatile uint32_t   last_rx_tick = 0;

/* Staging buffers for the RX ISR */
static uint8_t  frame_byte_buf;
static uint8_t  cmd_byte_buf;
static uint8_t  len_bytes_buf[4];
static uint8_t  payload_byte_buf;
static uint8_t  drain_byte_buf;

/* Transfer payload buffer + bookkeeping */
static uint8_t  rx_payload[MAX_PAYLOAD_BYTES];
static volatile uint8_t  current_cmd = 0;
static volatile uint32_t declared_len = 0;
static volatile uint32_t stored_count = 0;
static volatile uint32_t drained_count = 0;

/* ------------------------------------------------------------------------ */
/* Forward declarations                                                     */
/* ------------------------------------------------------------------------ */
static void SystemClock_Config(void);
static void GPIO_Init(void);
static void USART2_Init(void);
static void CRC_Init(void);

static uint32_t compute_crc(const uint8_t *data, uint32_t len);
static void uart_print(const char *s);
static void uart_send_raw(const uint8_t *data, uint16_t len);

static void rx_reset_to_wait_frame(void);
static void dispatch_command(void);
static void handle_cmd_transfer_crc(void);
static void send_response(uint8_t cmd, uint8_t status,
                          const uint8_t *data, uint32_t data_len);

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
    uart_print("LBTiny Supervisor - protocol v3 (command-framed)\r\n");
    uart_print("Frame: 0xA5 [cmd] [len_le_4B] [payload]\r\n");
    uart_print("       -> 0x5A [cmd] [status] [data_len_le_4B] [data]\r\n");
    uart_print("Max payload: 4096 bytes.  Inter-byte timeout: 1500 ms.\r\n");
    uart_print("============================================================\r\n");
    uart_print("\r\n");

    rx_reset_to_wait_frame();

    uint32_t blink_tick = HAL_GetTick();

    while (1) {
        if ((HAL_GetTick() - blink_tick) >= 500u) {
            HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
            blink_tick = HAL_GetTick();
        }

        /* Inter-byte timeout */
        if (rx_state != RX_STATE_WAIT_FRAME && rx_state != RX_STATE_READY) {
            if ((HAL_GetTick() - last_rx_tick) > TIMEOUT_MS) {
                HAL_UART_AbortReceive_IT(&huart2);
                rx_reset_to_wait_frame();
            }
        }

        if (rx_state == RX_STATE_READY) {
            dispatch_command();
            rx_reset_to_wait_frame();
        }
    }
}

/* ------------------------------------------------------------------------ */
/* Receive helpers                                                          */
/* ------------------------------------------------------------------------ */
static void rx_reset_to_wait_frame(void)
{
    rx_state      = RX_STATE_WAIT_FRAME;
    current_cmd   = 0;
    declared_len  = 0;
    stored_count  = 0;
    drained_count = 0;
    last_rx_tick  = HAL_GetTick();
    HAL_UART_Receive_IT(&huart2, &frame_byte_buf, 1);
}

/* ------------------------------------------------------------------------ */
/* Command dispatch                                                         */
/* ------------------------------------------------------------------------ */
static void dispatch_command(void)
{
    switch (current_cmd) {
    case CMD_TRANSFER_CRC:
        handle_cmd_transfer_crc();
        break;
    default:
        /* Unknown command - report and move on */
        send_response(current_cmd, STATUS_UNKNOWN_CMD, NULL, 0);
        break;
    }
}

static void handle_cmd_transfer_crc(void)
{
    uint32_t crc = compute_crc(rx_payload, stored_count);
    uint8_t  status = (declared_len > MAX_PAYLOAD_BYTES)
                    ? STATUS_OVERFLOW
                    : STATUS_OK;

    /* Response data: [declared_len_le_4B] [crc_le_4B] = 8 bytes */
    uint8_t data[8];
    data[0] = (uint8_t)(declared_len      & 0xFFu);
    data[1] = (uint8_t)((declared_len >> 8)  & 0xFFu);
    data[2] = (uint8_t)((declared_len >> 16) & 0xFFu);
    data[3] = (uint8_t)((declared_len >> 24) & 0xFFu);
    data[4] = (uint8_t)(crc          & 0xFFu);
    data[5] = (uint8_t)((crc >> 8)   & 0xFFu);
    data[6] = (uint8_t)((crc >> 16)  & 0xFFu);
    data[7] = (uint8_t)((crc >> 24)  & 0xFFu);

    send_response(CMD_TRANSFER_CRC, status, data, sizeof(data));
}

/*
 * send_response - assemble and transmit a framed response in a single
 * UART transmit call so it doesn't race with incoming bytes for the
 * next command (the issue we hit in v1 with separate uart_printf).
 */
static void send_response(uint8_t cmd, uint8_t status,
                          const uint8_t *data, uint32_t data_len)
{
    /* Max response: 1 sync + 1 cmd + 1 status + 4 data_len + data_len bytes.
     * For CMD_TRANSFER_CRC: 7 + 8 = 15 bytes. Sized for headroom. */
    uint8_t buf[64];
    if (data_len > sizeof(buf) - 7u) {
        /* Defensive - shouldn't happen with current commands */
        data_len = sizeof(buf) - 7u;
    }

    buf[0] = SYNC_NUCLEO_TO_HOST;
    buf[1] = cmd;
    buf[2] = status;
    buf[3] = (uint8_t)(data_len       & 0xFFu);
    buf[4] = (uint8_t)((data_len >> 8)  & 0xFFu);
    buf[5] = (uint8_t)((data_len >> 16) & 0xFFu);
    buf[6] = (uint8_t)((data_len >> 24) & 0xFFu);
    if (data_len > 0 && data != NULL) {
        memcpy(&buf[7], data, data_len);
    }
    uart_send_raw(buf, (uint16_t)(7u + data_len));
}

/* ------------------------------------------------------------------------ */
/* HAL receive callback - drives the state machine                          */
/* ------------------------------------------------------------------------ */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart != &huart2) {
        return;
    }
    last_rx_tick = HAL_GetTick();

    switch (rx_state) {

    case RX_STATE_WAIT_FRAME:
        if (frame_byte_buf == SYNC_HOST_TO_NUCLEO) {
            rx_state = RX_STATE_READ_CMD;
            HAL_UART_Receive_IT(&huart2, &cmd_byte_buf, 1);
        } else {
            HAL_UART_Receive_IT(&huart2, &frame_byte_buf, 1);
        }
        break;

    case RX_STATE_READ_CMD:
        current_cmd = cmd_byte_buf;
        rx_state = RX_STATE_READ_LEN;
        HAL_UART_Receive_IT(&huart2, len_bytes_buf, 4);
        break;

    case RX_STATE_READ_LEN:
        declared_len = ((uint32_t)len_bytes_buf[0])
                     | (((uint32_t)len_bytes_buf[1]) << 8)
                     | (((uint32_t)len_bytes_buf[2]) << 16)
                     | (((uint32_t)len_bytes_buf[3]) << 24);
        stored_count  = 0;
        drained_count = 0;
        if (declared_len == 0) {
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
            rx_state = RX_STATE_READY;
        } else if (stored_count >= MAX_PAYLOAD_BYTES) {
            rx_state = RX_STATE_DRAIN_OVERFLOW;
            HAL_UART_Receive_IT(&huart2, &drain_byte_buf, 1);
        } else {
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
        /* Main loop should consume READY before we re-arm. Defensive. */
        break;
    }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart == &huart2) {
        rx_reset_to_wait_frame();
    }
}

/* ------------------------------------------------------------------------ */
/* CRC computation (verified path from Step 2)                              */
/* ------------------------------------------------------------------------ */
static uint32_t compute_crc(const uint8_t *data, uint32_t len)
{
    static uint32_t words[(MAX_PAYLOAD_BYTES + 3) / 4 + 1];

    uint32_t padded_len = (len + 3u) & ~3u;
    uint32_t num_words = padded_len / 4u;

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

    __HAL_CRC_DR_RESET(&hcrc);
    __DSB();

    if (num_words == 0) {
        return hcrc.Instance->DR;
    }
    return HAL_CRC_Accumulate(&hcrc, words, num_words);
}

/* ------------------------------------------------------------------------ */
/* UART helpers                                                             */
/* ------------------------------------------------------------------------ */
static void uart_send_raw(const uint8_t *data, uint16_t len)
{
    HAL_UART_Transmit(&huart2, (uint8_t *)data, len, HAL_MAX_DELAY);
}

static void uart_print(const char *s)
{
    HAL_UART_Transmit(&huart2, (uint8_t *)s, (uint16_t)strlen(s), HAL_MAX_DELAY);
}

/* ------------------------------------------------------------------------ */
/* Peripheral init                                                          */
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
void SysTick_Handler(void) { HAL_IncTick(); }
void USART2_IRQHandler(void) { HAL_UART_IRQHandler(&huart2); }