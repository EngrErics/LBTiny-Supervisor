/*
 * main.c - LBTiny Supervisor, Step 2
 * =====================================
 *
 * Goal: Prove the STM32F4 hardware CRC peripheral agrees with the Python
 * implementation in stm32_crc.py (Step 1) on a known set of test vectors.
 *
 * What this firmware does, in order:
 *   1. Initialize the system clock (HSI -> 16 MHz, simplest possible)
 *   2. Initialize USART2 at 115200 baud (wired to ST-Link VCP)
 *   3. Initialize the CRC peripheral (fixed config on F4, nothing to set)
 *   4. For each hard-coded test vector:
 *        - pad to 4-byte boundary with 0xFF
 *        - feed into the CRC peripheral as 32-bit little-endian words
 *        - print the result over UART
 *   5. Loop forever (so you can re-read output by reopening the monitor)
 *
 * Expected workflow:
 *   - Flash this firmware
 *   - Open serial monitor at 115200 baud
 *   - Compare each printed CRC to the corresponding value from Step 1
 *   - If all match: algorithm agreement confirmed. Promote Step 1's
 *     cross-check vectors to assertions and proceed to Step 3.
 *
 * Byte-to-word packing: little-endian (b0 | b1<<8 | b2<<16 | b3<<24).
 * This MUST match Python's struct.unpack('<I', ...) convention.
 */

#include "stm32f4xx_hal.h"
#include <string.h>
#include <stdio.h>

/* ------------------------------------------------------------------------ */
/* Peripheral handles                                                       */
/* ------------------------------------------------------------------------ */
static UART_HandleTypeDef huart2;
static CRC_HandleTypeDef  hcrc;

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

/* ------------------------------------------------------------------------ */
/* Test vectors - MUST mirror the cross-check vectors in stm32_crc.py       */
/* ------------------------------------------------------------------------ */
typedef struct {
    const char    *label;
    const uint8_t *data;
    uint32_t       len;
} test_vector_t;

/* The trusted vector from Step 1 - we expect 0xC704DD7B */
static const uint8_t tv_trusted[]   = { 0x00, 0x00, 0x00, 0x00 };

/* Cross-check vectors */
static const uint8_t tv_all_ones[]  = { 0xFF, 0xFF, 0xFF, 0xFF };
static const uint8_t tv_deadbeef[]  = { 0xEF, 0xBE, 0xAD, 0xDE };
static const uint8_t tv_abcd[]      = { 'A', 'B', 'C', 'D' };
static const uint8_t tv_12345678[]  = { '1', '2', '3', '4', '5', '6', '7', '8' };
static const uint8_t tv_one_byte[]  = { 0x00 };
static const uint8_t tv_abc[]       = { 'A', 'B', 'C' };

static const test_vector_t vectors[] = {
    { "TRUSTED 0x00000000        ", tv_trusted,   sizeof(tv_trusted)   },
    { "all-ones word             ", tv_all_ones,  sizeof(tv_all_ones)  },
    { "0xDEADBEEF (LE)           ", tv_deadbeef,  sizeof(tv_deadbeef)  },
    { "'ABCD'                    ", tv_abcd,      sizeof(tv_abcd)      },
    { "'12345678'                ", tv_12345678,  sizeof(tv_12345678)  },
    { "empty (-> 0xFF x4 pad)    ", NULL,         0                    },
    { "single byte 0x00          ", tv_one_byte,  sizeof(tv_one_byte)  },
    { "3 bytes 'ABC'             ", tv_abc,       sizeof(tv_abc)       },
};

#define NUM_VECTORS (sizeof(vectors) / sizeof(vectors[0]))

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

    /* Give the serial monitor time to attach after flashing */
    HAL_Delay(500);

    uart_print("\r\n");
    uart_print("============================================================\r\n");
    uart_print("LBTiny Supervisor - Step 2: CRC peripheral verification [v3]\r\n");
    uart_print("============================================================\r\n");
    uart_print("\r\n");

    for (uint32_t i = 0; i < NUM_VECTORS; i++) {
        uint32_t crc = compute_crc(vectors[i].data, vectors[i].len);
        uart_printf("  %s  len=%3lu  CRC=0x%08lX\r\n",
                    vectors[i].label,
                    (unsigned long)vectors[i].len,
                    (unsigned long)crc);
    }

    uart_print("\r\n");
    uart_print("Compare each CRC against the corresponding value from\r\n");
    uart_print("stm32_crc.py self-test output. They must match exactly.\r\n");
    uart_print("\r\n");

    /* Idle loop with heartbeat blink so we know the chip is alive */
    while (1) {
        HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);  /* LD2 on the Nucleo */
        HAL_Delay(500);
    }
}

/* ------------------------------------------------------------------------ */
/* CRC computation                                                          */
/* ------------------------------------------------------------------------ */
/*
 * compute_crc - Compute STM32F4 hardware CRC over a byte buffer.
 *
 * Pads with 0xFF to a 4-byte boundary, packs bytes into 32-bit words using
 * EXPLICIT little-endian shifts (no pointer casts - portable and unambiguous),
 * resets the CRC peripheral, then accumulates.
 *
 * The F4 CRC peripheral has fixed config:
 *   poly = 0x04C11DB7, init = 0xFFFFFFFF, no reflection, no final XOR.
 * There is nothing to configure - HAL_CRC_Init() just enables the clock.
 */
static uint32_t compute_crc(const uint8_t *data, uint32_t len)
{
    /* Local padded buffer. Max test vector here is 8 bytes; we size for
     * comfort. In Step 3 (the real protocol) we'll use a 4 KB buffer. */
    uint8_t  padded[16];
    uint32_t words[4];

    /* Copy input */
    if (len > sizeof(padded)) {
        /* Defensive guard - should never happen with these test vectors */
        return 0xBADBAD00;
    }
    if (len > 0 && data != NULL) {
        memcpy(padded, data, len);
    }

    /* Pad with 0xFF to next 4-byte boundary */
    uint32_t padded_len = (len + 3u) & ~3u;
    for (uint32_t i = len; i < padded_len; i++) {
        padded[i] = 0xFF;
    }

    /* Pack into little-endian 32-bit words */
    uint32_t num_words = padded_len / 4u;
    for (uint32_t i = 0; i < num_words; i++) {
        words[i] =  ((uint32_t)padded[i * 4 + 0])
                 | (((uint32_t)padded[i * 4 + 1]) << 8)
                 | (((uint32_t)padded[i * 4 + 2]) << 16)
                 | (((uint32_t)padded[i * 4 + 3]) << 24);
    }

    /*
     * Reset CRC peripheral to initial value (0xFFFFFFFF).
     *
     * On F4 the reset bit in CR is self-clearing but has latency before
     * DR becomes valid. With -O0, a direct register read immediately
     * after the CR write races the peripheral and returns stale data.
     *
     * Solution: write CR_RESET, then call __HAL_CRC_DR_RESET which the
     * HAL implements as a write to CR with the appropriate barriers, and
     * insert a small read-after-write barrier so the peripheral catches up
     * before we read DR.
     */
    __HAL_CRC_DR_RESET(&hcrc);
    __DSB();   /* data synchronisation barrier - ensures the CR write
                * has completed in the peripheral before we read DR */

    /* DIAGNOSTIC: read DR immediately after reset. Should always be 0xFFFFFFFF.
     * If this prints something else, the reset isn't working as expected. */
    uint32_t dr_after_reset = hcrc.Instance->DR;
    uart_printf("    [diag] DR after reset = 0x%08lX\r\n",
                (unsigned long)dr_after_reset);

    if (num_words == 0) {
        /*
         * Empty input: CRC is just the initial value 0xFFFFFFFF.
         * HAL_CRC_Calculate with len=0 is undefined behaviour in HAL,
         * so we read DR directly after the reset above.
         */
        return dr_after_reset;
    }

    /*
     * HAL_CRC_Calculate resets before accumulating, which would double-reset
     * here. Use HAL_CRC_Accumulate instead so we control the reset ourselves
     * (already done above) and just feed the words in.
     */
    return HAL_CRC_Accumulate(&hcrc, words, num_words);
}

/* ------------------------------------------------------------------------ */
/* UART helpers                                                             */
/* ------------------------------------------------------------------------ */
static void uart_print(const char *s)
{
    HAL_UART_Transmit(&huart2, (uint8_t *)s, (uint16_t)strlen(s), HAL_MAX_DELAY);
}

#include <stdarg.h>
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
/* Peripheral initialization                                                */
/* ------------------------------------------------------------------------ */

/*
 * Simplest possible clock config: use the internal 16 MHz HSI directly.
 * No PLL, no HSE. UART baud divisors at 16 MHz hit 115200 with very low
 * error. This is fine for bring-up; we'll revisit for higher throughput
 * later if needed.
 */
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
    if (HAL_RCC_OscConfig(&osc) != HAL_OK) {
        while (1) { /* trap */ }
    }

    clk.ClockType = RCC_CLOCKTYPE_HCLK   | RCC_CLOCKTYPE_SYSCLK
                  | RCC_CLOCKTYPE_PCLK1  | RCC_CLOCKTYPE_PCLK2;
    clk.SYSCLKSource   = RCC_SYSCLKSOURCE_HSI;
    clk.AHBCLKDivider  = RCC_SYSCLK_DIV1;
    clk.APB1CLKDivider = RCC_HCLK_DIV1;
    clk.APB2CLKDivider = RCC_HCLK_DIV1;
    if (HAL_RCC_ClockConfig(&clk, FLASH_LATENCY_0) != HAL_OK) {
        while (1) { /* trap */ }
    }
}

static void GPIO_Init(void)
{
    __HAL_RCC_GPIOA_CLK_ENABLE();

    /* PA5 = LD2 user LED (heartbeat) */
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

    /* PA2 = USART2_TX, PA3 = USART2_RX, AF7 */
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
    if (HAL_UART_Init(&huart2) != HAL_OK) {
        while (1) { /* trap */ }
    }
}

static void CRC_Init(void)
{
    __HAL_RCC_CRC_CLK_ENABLE();
    hcrc.Instance = CRC;
    if (HAL_CRC_Init(&hcrc) != HAL_OK) {
        while (1) { /* trap */ }
    }
    /* On F4 there is nothing else to configure - poly, init, reflection
     * are all fixed in silicon. HAL_CRC_Init just enables the clock and
     * resets the data register. */
}

/* ------------------------------------------------------------------------ */
/* HAL tick - HAL_Init() configures SysTick at 1 ms by default              */
/* ------------------------------------------------------------------------ */
void SysTick_Handler(void)
{
    HAL_IncTick();
}