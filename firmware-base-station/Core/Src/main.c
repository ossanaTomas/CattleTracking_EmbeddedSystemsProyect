	/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "spi.h"
#include "usb_device.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "usbd_cdc_if.h"
#include <string.h>
#include <stdio.h>
#include "LoRa.h"
#include "base_simple.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
extern volatile uint8_t  g_usb_rx_ready;
extern volatile uint32_t g_usb_rx_len;
volatile uint8_t g_rx_len = 0;
extern uint8_t g_usb_rx_buf[64];

static uint8_t CDC_Transmit_Blocking(uint8_t *buf, uint16_t len, uint32_t timeout_ms)
{
    uint32_t t0 = HAL_GetTick();
    while (CDC_Transmit_FS(buf, len) == USBD_BUSY) {
        if ((HAL_GetTick() - t0) >= timeout_ms) return USBD_BUSY;
        HAL_Delay(1);
    }
    return USBD_OK;
}


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
#include <inttypes.h>

static inline uint16_t get_u16_le(const uint8_t *p) {
    return (uint16_t)p[0] | ((uint16_t)p[1] << 8);
}
static inline uint32_t get_u32_le(const uint8_t *p) {
    return (uint32_t)p[0] |
           ((uint32_t)p[1] << 8) |
           ((uint32_t)p[2] << 16) |
           ((uint32_t)p[3] << 24);
}
static inline int32_t get_i32_le(const uint8_t *p) {
    return (int32_t)get_u32_le(p);
}

// imprime v como entero con "frac_digits" decimales, sin float.
static void fmt_i32_scaled(char *out, size_t out_sz, int32_t v, uint32_t frac_digits)
{
    uint32_t scale = 1;
    for (uint32_t i = 0; i < frac_digits; i++) scale *= 10;

    int32_t ip = v / (int32_t)scale;
    int32_t rem = v % (int32_t)scale;
    uint32_t fp = (rem < 0) ? (uint32_t)(-rem) : (uint32_t)rem;

    // %0*u imprime con ceros a la izquierda (ancho = frac_digits)
    snprintf(out, out_sz, "%" PRId32 ".%0*" PRIu32, ip, (int)frac_digits, fp);
}

static void usb_print_frame_decoded(const uint8_t *fr, uint8_t len)
{
    // esperamos: hdr(8) + plen(23) + crc(2) = 33 para DATA
    if (len < 8) {
        const char *m = "Frame corto (<8)\r\n";
        CDC_Transmit_Blocking((uint8_t*)m, (uint16_t)strlen(m), 50);
        return;
    }

    uint8_t ver   = fr[0];
    uint8_t net   = fr[1];
    uint8_t type  = fr[2];
    uint8_t src   = fr[3];
    uint8_t dst   = fr[4];
    uint8_t seq   = fr[5];
    uint8_t flags = fr[6];
    uint8_t plen  = fr[7];

    uint16_t expected = (uint16_t)(8 + plen + 2);
    if (len != expected) {
        char msg[96];
        int n = snprintf(msg, sizeof(msg),
            "Len mismatch: got=%u expected=%u (plen=%u)\r\n", len, expected, plen);
        CDC_Transmit_Blocking((uint8_t*)msg, (uint16_t)n, 50);
        return;
    }

    const uint8_t *pl = &fr[8];

    // Por ahora decodifico solo TYPE=0x10 (DATA) y PLEN=23
    if (type == 0x10 && plen == 23) {
        uint32_t t_ms    = get_u32_le(pl + 0);
        int32_t  lat_e7  = get_i32_le(pl + 4);
        int32_t  lon_e7  = get_i32_le(pl + 8);
        uint8_t  sats    = pl[12];
        uint16_t course  = get_u16_le(pl + 13);   // centi-deg (deg*100)
        int32_t  temp_mC = get_i32_le(pl + 15);   // m°C
        uint16_t batt_mV = get_u16_le(pl + 19);   // mV
        uint16_t err     = get_u16_le(pl + 21);   // mask

        char lat_s[24], lon_s[24], temp_s[24], batt_s[24], course_s[24];
        fmt_i32_scaled(lat_s,  sizeof(lat_s),  lat_e7, 7);      // e7 -> 7 decimales
        fmt_i32_scaled(lon_s,  sizeof(lon_s),  lon_e7, 7);
        fmt_i32_scaled(temp_s, sizeof(temp_s), temp_mC, 3);     // mC -> 3 decimales
        // batt mV y course cdeg: también sin float
        snprintf(batt_s, sizeof(batt_s), "%" PRIu16 ".%03" PRIu16, (uint16_t)(batt_mV/1000), (uint16_t)(batt_mV%1000));
        snprintf(course_s, sizeof(course_s), "%" PRIu16 ".%02" PRIu16, (uint16_t)(course/100), (uint16_t)(course%100));

        char msg[220];
        int n = snprintf(msg, sizeof(msg),
            "VER=%u NET=0x%02X TYPE=0x%02X SRC=%u DST=%u SEQ=%u FLAGS=0x%02X\r\n"
            "t_ms=%" PRIu32 " lat=%s lon=%s sats=%u course=%s deg temp=%s C batt=%s V err=0x%04X\r\n\r\n",
            ver, net, type, src, dst, seq, flags,
            t_ms, lat_s, lon_s, sats, course_s, temp_s, batt_s, err);

        CDC_Transmit_Blocking((uint8_t*)msg, (uint16_t)n, 50);
        return;
    }

    // fallback: si no es DATA conocido, imprimí header al menos
    {
        char msg[120];
        int n = snprintf(msg, sizeof(msg),
            "Frame: VER=%u NET=0x%02X TYPE=0x%02X SRC=%u DST=%u SEQ=%u FLAGS=0x%02X PLEN=%u\r\n",
            ver, net, type, src, dst, seq, flags, plen);
        CDC_Transmit_Blocking((uint8_t*)msg, (uint16_t)n, 50);
    }
}



LoRa myLoRa;
uint8_t FlagRecibir=0;
uint8_t RxBuffer1[120]={0};




/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_SPI1_Init();
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN 2 */
	myLoRa = newLoRa();

	myLoRa.CS_port = NSS_GPIO_Port;
	myLoRa.CS_pin = NSS_Pin;
	myLoRa.reset_port = RST_GPIO_Port;
	myLoRa.reset_pin = RST_Pin;
	myLoRa.DIO0_port = DIO0_GPIO_Port;
	myLoRa.DIO0_pin = DIO0_Pin;
	myLoRa.hSPIx = &hspi1;



	 uint32_t t0 = HAL_GetTick();
	 const char hello[] = "BluePill CDC OK\r\n";


	HAL_Delay(50);

	uint8_t LoRa_stat = 0;
	for (int i = 0; i < 1000; i++) {
		if (LoRa_init(&myLoRa) == LORA_OK) {
			LoRa_stat = 1;
			break;
		}
		HAL_Delay(50);
	}

	if (LoRa_stat != 1) {
		while (1) {

		}
	}


  BaseSimple_Init(&myLoRa);

	HAL_Delay(100);









  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    BaseSimple_Task();
  }
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLL;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if (GPIO_Pin == DIO0_Pin) {
    	//HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
        BaseSimple_OnDIO0IRQ();

    }
}






/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
