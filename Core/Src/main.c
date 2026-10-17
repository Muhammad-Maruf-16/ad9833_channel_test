/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : AD9833 Per-Channel Tester (STM32F103 + 4x AD9833)
  *                   Dikontrol secara real-time melalui debugger (Live Expressions)
  *
  * CARA PENGGUNAAN LEWAT DEBUGGER (STM32CubeIDE):
  * 1. Hubungkan ST-Link dan masuk ke mode Debug (ikon kumbang/bug).
  * 2. Buka tab "Live Expressions" (Window -> Show View -> Live Expressions).
  * 3. Tambahkan variabel berikut ke Live Expressions:
  *
  *    --- STATUS ON/OFF PER CHANNEL (1 = ON, 0 = OFF) ---
  *    ch1_on
  *    ch2_on
  *    ch3_on
  *    ch4_on
  *
  *    --- FREKUENSI PER CHANNEL (Nilai dalam Hz, misal 25000) ---
  *    ch1_freq
  *    ch2_freq
  *    ch3_freq
  *    ch4_freq
  *
  *    --- BENTUK GELOMBANG (0: Sinus, 1: Segitiga, 2: Kotak/Square) ---
  *    ch1_wave  (Default: 2 - Square DIV2 seperti di audio_jamm)
  *    ch2_wave
  *    ch3_wave
  *    ch4_wave
  *
  *    --- MONITOR DETAK/HEARTBEAT ---
  *    loop_counter (Nilai bertambah terus membuktikan MCU berjalan aktif)
  *
  * 4. Tekan "Resume" (F8) agar program berjalan.
  * 5. Ubah nilai variabel langsung di Live Expressions untuk menyalakan/mematikan
  *    atau mengganti frekuensi tiap channel secara independen.
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdint.h>
#include <stdbool.h>
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi1;

/* USER CODE BEGIN PV */
#define NCH 4

/* =========================================================================
 * VARIABEL KONTROL DEBUG (Dapat diubah real-time di Live Expressions)
 * ========================================================================= */

/* Status On/Off masing-masing channel: 1 = ON, 0 = OFF */
volatile uint8_t  ch1_on   = 0;
volatile uint8_t  ch2_on   = 0;
volatile uint8_t  ch3_on   = 0;
volatile uint8_t  ch4_on   = 0;

/* Frekuensi masing-masing channel dalam Hertz (Hz) */
volatile uint32_t ch1_freq = 25000;
volatile uint32_t ch2_freq = 25000;
volatile uint32_t ch3_freq = 25000;
volatile uint32_t ch4_freq = 25000;

/* Bentuk gelombang (Waveform mode):
 * 0 = SINE     (Sinus)
 * 1 = TRIANGLE (Segitiga)
 * 2 = SQUARE   (Kotak / Square DIV2 50% duty, default audio_jamm)
 * 3 = SQUARE_D1(Kotak / Square direct)
 */
volatile uint8_t  ch1_wave = 2;
volatile uint8_t  ch2_wave = 2;
volatile uint8_t  ch3_wave = 2;
volatile uint8_t  ch4_wave = 2;

/* Counter detak pengujian (heartbeat) */
volatile uint32_t loop_counter = 0;

/* Mapping Pin FSYNC */
static GPIO_TypeDef* fnc_port[NCH];
static uint16_t      fnc_pin[NCH];

/* State pelacak perubahan nilai */
static uint8_t  last_on[NCH]   = { 0, 0, 0, 0 };
static uint32_t last_freq[NCH] = { 0, 0, 0, 0 };
static uint8_t  last_wave[NCH] = { 0, 0, 0, 0 };
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* ---------- Timing (DWT cycle counter) ---------- */
static void dwt_init(void) {
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0;
    DWT->CTRL  |= DWT_CTRL_CYCCNTENA_Msk;
}

static inline void delay_us(uint32_t us) {
    uint32_t start  = DWT->CYCCNT;
    uint32_t cycles = us * (SystemCoreClock / 1000000UL);
    while ((DWT->CYCCNT - start) < cycles) { }
}

/* ---------- AD9833 Driver ---------- */

/* Kirim 16-bit word ke AD9833 melalui SPI1 */
static inline void ad_write(uint8_t ch, uint16_t w) {
    fnc_port[ch]->BRR = fnc_pin[ch];             /* FSYNC low  */
    while (!(SPI1->SR & SPI_SR_TXE)) { }
    SPI1->DR = (uint8_t)(w >> 8);
    while (!(SPI1->SR & SPI_SR_TXE)) { }
    SPI1->DR = (uint8_t)(w & 0xFF);
    while (!(SPI1->SR & SPI_SR_TXE)) { }
    while (SPI1->SR & SPI_SR_BSY) { }
    fnc_port[ch]->BSRR = fnc_pin[ch];            /* FSYNC high */
}

/* Dapatkan register kontrol AD9833 berdasarkan tipe gelombang */
static inline uint16_t get_ctrl_word(uint8_t wave) {
    switch (wave) {
        case 1:  return 0x2002; /* Triangle wave (OPBITEN=0, MODE=1) */
        case 2:  return 0x2028; /* Square wave DIV2 (50% duty, default audio_jamm) */
        case 3:  return 0x2020; /* Square wave direct MSB */
        case 0:
        default: return 0x2000; /* Sine wave (OPBITEN=0, MODE=0) */
    }
}

/* Set frekuensi ke register FREQ0 AD9833 (MCLK = 25 MHz) */
static inline void ad_set_freq(uint8_t ch, uint32_t f_hz) {
    /* freq_word = f * 2^28 / 25 MHz */
    uint32_t w = (uint32_t)(((uint64_t)f_hz * 268435456ULL + 12500000ULL) / 25000000ULL);
    ad_write(ch, (uint16_t)(0x4000 | (w & 0x3FFF)));
    ad_write(ch, (uint16_t)(0x4000 | ((w >> 14) & 0x3FFF)));
}

/* Inisialisasi pin dan AD9833 ke kondisi awal (semua OFF) */
static void ad_init_all(void) {
    fnc_port[0] = FNC1_GPIO_Port; fnc_pin[0] = FNC1_Pin;
    fnc_port[1] = FNC2_GPIO_Port; fnc_pin[1] = FNC2_Pin;
    fnc_port[2] = FNC3_GPIO_Port; fnc_pin[2] = FNC3_Pin;
    fnc_port[3] = FNC4_GPIO_Port; fnc_pin[3] = FNC4_Pin;

    for (uint8_t ch = 0; ch < NCH; ch++) {
        fnc_port[ch]->BSRR = fnc_pin[ch];        /* FSYNC idle HIGH */
    }
    HAL_Delay(50);

    /* Reset dan matikan semua modul AD9833 saat awal */
    for (uint8_t ch = 0; ch < NCH; ch++) {
        ad_write(ch, 0x2140);                    /* RESET=1, SLEEP12=1 (DAC power down) */
        ad_set_freq(ch, 25000);
        ad_write(ch, 0xC000);                    /* PHASE0 = 0 */
    }
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  */
int main(void)
{
  HAL_Init();
  SystemClock_Config();
  MX_GPIO_Init();
  MX_SPI1_Init();

  /* USER CODE BEGIN 2 */
  dwt_init();
  __HAL_SPI_ENABLE(&hspi1);
  ad_init_all();
  /* USER CODE END 2 */

  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    /* Array pointer ke variabel debug per channel */
    volatile uint8_t*  p_on[NCH]   = { &ch1_on,   &ch2_on,   &ch3_on,   &ch4_on };
    volatile uint32_t* p_freq[NCH] = { &ch1_freq, &ch2_freq, &ch3_freq, &ch4_freq };
    volatile uint8_t*  p_wave[NCH] = { &ch1_wave, &ch2_wave, &ch3_wave, &ch4_wave };

    for (uint8_t i = 0; i < NCH; i++)
    {
      uint8_t  cur_on   = (*p_on[i] != 0) ? 1 : 0;
      uint32_t cur_freq = *p_freq[i];
      uint8_t  cur_wave = *p_wave[i];

      /* Proteksi batas frekuensi (1 Hz s/d 12.5 MHz) */
      if (cur_freq == 0) cur_freq = 1;
      if (cur_freq > 12500000UL) cur_freq = 12500000UL;

      /* Cek jika status ON / OFF berubah */
      if (cur_on != last_on[i])
      {
        last_on[i]   = cur_on;
        last_freq[i] = cur_freq;
        last_wave[i] = cur_wave;

        if (cur_on)
        {
          /* Nyalakan channel: set frekuensi, fase, lalu aktifkan gelombang */
          ad_write(i, 0x2100);                    /* Reset aktif saat setting register */
          ad_set_freq(i, cur_freq);
          ad_write(i, 0xC000);                    /* Phase = 0 */
          ad_write(i, get_ctrl_word(cur_wave));   /* Keluar reset dengan waveform terpilih */
        }
        else
        {
          /* Matikan channel: masukkan ke RESET dan SLEEP DAC */
          ad_write(i, 0x2140);
        }
      }
      /* Jika channel sedang ON, cek perubahan frekuensi atau waveform secara on-the-fly */
      else if (cur_on)
      {
        if (cur_freq != last_freq[i])
        {
          last_freq[i] = cur_freq;
          ad_set_freq(i, cur_freq);               /* Update frekuensi secara langsung tanpa glitch */
        }

        if (cur_wave != last_wave[i])
        {
          last_wave[i] = cur_wave;
          ad_write(i, get_ctrl_word(cur_wave));   /* Update waveform */
        }
      }
    }

    loop_counter++;
    HAL_Delay(10); /* Delay 10 ms memberikan ruang bandwidth SWD debug yang stabil dan responsif */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration (HSE 8 MHz x9 = 72 MHz)
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief SPI1 Initialization Function
  * Prescaler 4 -> 18 MHz (AD9833 maks 40 MHz).
  */
static void MX_SPI1_Init(void)
{
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_HIGH;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief GPIO Initialization Function
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  HAL_GPIO_WritePin(GPIOA, FNC1_Pin|FNC2_Pin|FNC3_Pin|FNC4_Pin, GPIO_PIN_SET);

  GPIO_InitStruct.Pin = FNC1_Pin|FNC2_Pin|FNC3_Pin|FNC4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

void Error_Handler(void)
{
  __disable_irq();
  while (1)
  {
  }
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
}
#endif /* USE_FULL_ASSERT */
