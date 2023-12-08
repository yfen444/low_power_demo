/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#include "icache.h"
#include "lpdma.h"
#include "memorymap.h"
#include "rtc.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include "bme68x.h"
#include "lpbam_master.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef enum
{
  RESET_CAUSE_UNKNOWN = 0,
  RESET_CAUSE_LOW_POWER_RESET,
  RESET_CAUSE_WINDOW_WATCHDOG_RESET,
  RESET_CAUSE_INDEPENDENT_WATCHDOG_RESET,
  RESET_CAUSE_SOFTWARE_RESET,
  RESET_CAUSE_POWER_ON_POWER_DOWN_RESET,
  RESET_CAUSE_EXTERNAL_RESET_PIN_RESET,
  RESET_CAUSE_OPTIONAL_BYTE_LOADING_RESET,
  RESET_CAUSE_BROWNOUT_RESET,
} reset_cause_t;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define LOW_POWER_MODE_STOP2

#define PRINT_BUS                huart1
#define DBG_IF_MAX_BUFFER_SIZE  (uint16_t)(256)

#define RTC_TIMER                hrtc

/* Uncomment the next line to enable debug */
//#define EnableDBG

/* Uncomment the next line to enable debug after waking up from Standby mode */
//#define EnableDBGPOSTLPMode

/* Uncomment the next line to use unconditional PRINT */
//#define LOG_HIDE
#ifndef LOG_HIDE
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define PRINT(format, args ...)\
  TRACE_PRINT(format "\n\r",  \
                    ## args)
#else
#define PRINT(...) __NOP();
#endif

#define PRINT_FORCE(format, args ...)\
  TRACE_PRINT(format "\n\r",  \
                    ## args)

#define PRINT_PLAIN(format, args ...)\
  TRACE_PRINT(format ,  \
                    ## args)

#define TRACE_PRINT(format, args...) do {\
  if (snprintf((char *)print_buf, DBG_IF_MAX_BUFFER_SIZE, format "", ## args) > 0)\
  {\
  traceIF_uartPrintForce((uint8_t *)print_buf,\
                         (uint16_t)strlen((const char *) print_buf));\
  }\
} while (0);

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

I2C_HandleTypeDef hi2c3;
//RTC_HandleTypeDef hrtc;
/* USER CODE BEGIN PV */
static uint8_t print_buf[DBG_IF_MAX_BUFFER_SIZE] = {0};

static reset_cause_t reset_cause = RESET_CAUSE_UNKNOWN;

static uint32_t backup_sram_value_previous = 0;

static bool wakeup_from_standby = false;
static bool wakeup_from_stop = false;

static bool wakeup_from_RTC = false;
static bool wakeup_from_user_botton = false;

/* Transmission Buffers */
uint8_t aTxBuffer1[TX_BUFFER1_SIZE] = {BME68X_REG_CHIP_ID};

/* Reception Buffers */
uint8_t aRxBuffer1[RX_BUFFER1_SIZE] = {0};

uint32_t lpbam_idx = 0;
bool i2c3_er_irq = false;
bool i2c3_ev_irq = false;
bool lpdma_ch0_irq = false;
bool rtc_irq = false;

bool DMA_TC_flag = false;
bool DMA_error_flag = false;
bool i2c3_er_flag = false;
bool i2c3_rx_complete_flag = false;
bool i2c3_tx_complete_flag = false;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void SystemPower_Config(void);
/* USER CODE BEGIN PFP */
static void traceIF_uartTransmit(uint8_t *ptr, uint16_t len);
static void traceIF_uartPrintForce(uint8_t *pptr, uint16_t len);
static void determine_reset_cause(void);
const char * reset_cause_get_name(reset_cause_t reset_cause);
static void configure_optional_byte_sram2_retention(bool mode);
static void configure_debuggability(void);
static void determine_wake_up_mode(void);
static void determine_wake_up_source_from_standby(void);
static void configure_sram2_retention(void);
static void configure_gpio_retention(void);
static void print_reset_info(void);
static void print_wakeup_info(void);
static void print_unhandled_interrupt(void);
static void update_sram_content(void);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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
  backup_sram_value_previous = *((uint32_t*) (BKPSRAM_BASE));

  configure_optional_byte_sram2_retention(false);

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* Configure the System Power */
  SystemPower_Config();

  /* USER CODE BEGIN SysInit */
  configure_debuggability();

  /* Determine previous MCU mode and reset cause */
  determine_wake_up_mode();
  determine_wake_up_source_from_standby();
  determine_reset_cause();
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_MEMORYMAP_Init();
  MX_LPDMA1_Init();
  MX_RTC_Init();
  MX_ICACHE_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
#ifndef EnableDBG
  turn_off_dgb_pins();
#endif

  HAL_RTC_DeInit(&RTC_TIMER);

  print_reset_info();
  print_wakeup_info();

  /* The following command enables backup SRAM retention but seems unnecessary from test result*/
//  HAL_PWREx_EnableBkupRAMRetention();

  HAL_PWREx_EnableUltraLowPowerMode();

  /* Enable Power Control clock */
  __HAL_RCC_PWR_CLK_ENABLE();

  /* SRAMs configuration */
  __HAL_RCC_BKPSRAM_CLK_ENABLE();
  configure_sram2_retention();

#ifdef LOW_POWER_MODE_STOP2
#endif

  configure_gpio_retention();

  PRINT("Previous Backup SRAM content: %lu", backup_sram_value_previous);

  update_sram_content();

  bool reinit_LPBAM = false;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* Insert 5 seconds delay */
    HAL_Delay(3000);

    /* MX_Master_Init() has to be placed after HAL_Delay since MX_Master_Init() changes clock configuration */
    if (lpbam_idx == 0)
    {
      PRINT("Entering LPBAM.");
      /* LPBAM Master application init */
      MX_Master_Init();

      /* LPBAM Master application ReloadSeq scenario init */
      MX_Master_ReloadSeq_Init();

      /* LPBAM Master application ReloadSeq scenario build */
      MX_Master_ReloadSeq_Build();

      /* LPBAM Master application ReloadSeq scenario link */
      MX_Master_ReloadSeq_Link(&handle_LPDMA1_Channel0);

      /* LPBAM Master application ReloadSeq scenario start */
      MX_Master_ReloadSeq_Start(&handle_LPDMA1_Channel0);
    }

    if (reinit_LPBAM)
    {
      PRINT("Entering LPBAM.");
      /* LPBAM Master application init */
      MX_Master_Init();

      /* LPBAM Master application ReloadSeq scenario init */
      MX_Master_ReloadSeq_Init();

      /* LPBAM Master application ReloadSeq scenario build */
      /* For some reason the Queue can't be built twice */
//      MX_Master_ReloadSeq_Build();

      /* LPBAM Master application ReloadSeq scenario link */
      MX_Master_ReloadSeq_Link(&handle_LPDMA1_Channel0);

      /* LPBAM Master application ReloadSeq scenario start */
      MX_Master_ReloadSeq_Start(&handle_LPDMA1_Channel0);

      reinit_LPBAM = false;
    }

    /* Enter the target low-power mode */
    HAL_SuspendTick();
    HAL_PWREx_EnterSTOP2Mode(PWR_STOPENTRY_WFI);
    HAL_ResumeTick();

    /* LPBAM Master application ReloadSeq scenario stop */
    MX_Master_ReloadSeq_Stop(&handle_LPDMA1_Channel0);

    /* LPBAM Master application ReloadSeq scenario unlink */
    MX_Master_ReloadSeq_UnLink(&handle_LPDMA1_Channel0);

    /* LPBAM Master application ReloadSeq scenario de-init */
    MX_Master_ReloadSeq_DeInit();

    reinit_LPBAM = true;

    /* Configure the system clock */
    SystemClock_Config();

    /* Configure the System Power */
    SystemPower_Config();

    print_wakeup_info();
    print_unhandled_interrupt();

    update_sram_content();

    PRINT("BME680 sensor ID read: %d", aRxBuffer1[0]);
    aRxBuffer1[0] = 0;
    lpbam_idx++;
    if (i2c3_er_irq)
    {
      PRINT("i2c3_er_irq is on");
    }
    if (i2c3_ev_irq)
    {
      PRINT("i2c3_ev_irq is on");
    }
    if (lpdma_ch0_irq)
    {
      PRINT("lpdma_ch0_irq is on");
    }
    if (rtc_irq)
    {
      PRINT("rtc_irq is on");
    }
    if (DMA_TC_flag)
    {
      PRINT("DMA_TC_flag is on");
    }
    if (DMA_error_flag)
    {
      PRINT("DMA_error_flag is on");
    }
    if (i2c3_er_flag)
    {
      PRINT("i2c3_er_flag is on");
    }
    if (i2c3_rx_complete_flag)
    {
      PRINT("i2c3_rx_complete_flag is on");
    }
    if (i2c3_tx_complete_flag)
    {
      PRINT("i2c3_tx_complete_flag is on");
    }

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    wakeup_from_user_botton = false;
    wakeup_from_RTC = false;
    DMA_TC_flag = false;


    i2c3_er_irq = false;
    i2c3_ev_irq = false;
    lpdma_ch0_irq = false;
    rtc_irq = false;
    DMA_TC_flag = false;
    DMA_error_flag = false;
    i2c3_er_flag = false;
    i2c3_rx_complete_flag = false;
    i2c3_tx_complete_flag = false;
  }
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

  /** Configure the main internal regulator output voltage
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure LSE Drive Capability
  */
  HAL_PWR_EnableBkUpAccess();
  __HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_LOW);

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSE|RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = RCC_MSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_4;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLMBOOST = RCC_PLLMBOOST_DIV1;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 80;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLLVCIRANGE_0;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_PCLK3;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief Power Configuration
  * @retval None
  */
static void SystemPower_Config(void)
{

  /*
   * Disable the internal Pull-Up in Dead Battery pins of UCPD peripheral
   */
  HAL_PWREx_DisableUCPDDeadBattery();

  /*
   * Switch to SMPS regulator instead of LDO
   */
  if (HAL_PWREx_ConfigSupply(PWR_SMPS_SUPPLY) != HAL_OK)
  {
    Error_Handler();
  }
/* USER CODE BEGIN PWR */
/* USER CODE END PWR */
}

/* USER CODE BEGIN 4 */
static void traceIF_uartTransmit(uint8_t *ptr, uint16_t len)
{
  /* Send the trace */
  (void)HAL_UART_Transmit(&huart1, (uint8_t *)ptr, len, HAL_MAX_DELAY);

}

static void traceIF_uartPrintForce(uint8_t *pptr, uint16_t len)
{
  uint8_t *ptr;
  ptr = pptr;
  traceIF_uartTransmit(ptr, len);
}

/// @brief      Obtain the STM32 system reset cause
/// @param      None
/// @return     The system reset cause
static void determine_reset_cause(void)
{
  if (__HAL_RCC_GET_FLAG(RCC_FLAG_LPWRRST))
  {
    reset_cause = RESET_CAUSE_LOW_POWER_RESET;
  }
  else if (__HAL_RCC_GET_FLAG(RCC_FLAG_WWDGRST))
  {
    reset_cause = RESET_CAUSE_WINDOW_WATCHDOG_RESET;
  }
  else if (__HAL_RCC_GET_FLAG(RCC_FLAG_IWDGRST))
  {
    reset_cause = RESET_CAUSE_INDEPENDENT_WATCHDOG_RESET;
  }
  else if (__HAL_RCC_GET_FLAG(RCC_FLAG_SFTRST))
  {
    // This reset is induced by calling the ARM CMSIS
    // `NVIC_SystemReset()` function!
    reset_cause = RESET_CAUSE_SOFTWARE_RESET;
  }
  else if (__HAL_RCC_GET_FLAG(RCC_FLAG_PINRST))
  {
    reset_cause = RESET_CAUSE_EXTERNAL_RESET_PIN_RESET;
  }
  else if (__HAL_RCC_GET_FLAG(RCC_FLAG_OBLRST))
  {
    reset_cause = RESET_CAUSE_OPTIONAL_BYTE_LOADING_RESET;
  }
  // Needs to come *after* checking the `RCC_FLAG_PORRST` flag in order to
  // ensure first that the reset cause is NOT a POR/PDR reset. See note
  // below.
  else if (__HAL_RCC_GET_FLAG(RCC_FLAG_BORRST))
  {
    reset_cause = RESET_CAUSE_BROWNOUT_RESET;
  }
  else
  {
    reset_cause = RESET_CAUSE_UNKNOWN;
  }

  // Clear all the reset flags or else they will remain set during future
  // resets until system power is fully removed.
  __HAL_RCC_CLEAR_RESET_FLAGS();
}

// Note: any of the STM32 Hardware Abstraction Layer (HAL) Reset and Clock
// Controller (RCC) header files, such as
// "STM32Cube_FW_F7_V1.12.0/Drivers/STM32F7xx_HAL_Driver/Inc/stm32f7xx_hal_rcc.h",
// "STM32Cube_FW_F2_V1.7.0/Drivers/STM32F2xx_HAL_Driver/Inc/stm32f2xx_hal_rcc.h",
// etc., indicate that the brownout flag, `RCC_FLAG_BORRST`, will be set in
// the event of a "POR/PDR or BOR reset". This means that a Power-On Reset
// (POR), Power-Down Reset (PDR), OR Brownout Reset (BOR) will trip this flag.
// See the doxygen just above their definition for the
// `__HAL_RCC_GET_FLAG()` macro to see this:
//      "@arg RCC_FLAG_BORRST: POR/PDR or BOR reset." <== indicates the Brownout
//      Reset flag will *also* be set in the event of a POR/PDR.
// Therefore, you must check the Brownout Reset flag, `RCC_FLAG_BORRST`, *after*
// first checking the `RCC_FLAG_PORRST` flag in order to ensure first that the
// reset cause is NOT a POR/PDR reset.


/// @brief      Obtain the system reset cause as an ASCII-printable name string
///             from a reset cause type
/// @param[in]  reset_cause     The previously-obtained system reset cause
/// @return     A null-terminated ASCII name string describing the system
///             reset cause
const char * reset_cause_get_name(reset_cause_t reset_cause)
{
  const char * reset_cause_name = "TBD";

  switch (reset_cause)
  {
    case RESET_CAUSE_UNKNOWN:
      reset_cause_name = "UNKNOWN";
      break;
    case RESET_CAUSE_LOW_POWER_RESET:
      reset_cause_name = "LOW_POWER_RESET";
      break;
    case RESET_CAUSE_WINDOW_WATCHDOG_RESET:
      reset_cause_name = "WINDOW_WATCHDOG_RESET";
      break;
    case RESET_CAUSE_INDEPENDENT_WATCHDOG_RESET:
      reset_cause_name = "INDEPENDENT_WATCHDOG_RESET";
      break;
    case RESET_CAUSE_SOFTWARE_RESET:
      reset_cause_name = "SOFTWARE_RESET";
      break;
    case RESET_CAUSE_POWER_ON_POWER_DOWN_RESET:
      reset_cause_name = "POWER-ON_RESET (POR) / POWER-DOWN_RESET (PDR)";
      break;
    case RESET_CAUSE_EXTERNAL_RESET_PIN_RESET:
      reset_cause_name = "EXTERNAL_RESET_PIN_RESET";
      break;
    case RESET_CAUSE_OPTIONAL_BYTE_LOADING_RESET:
      reset_cause_name = "Option_Byte_Loading_RESET";
      break;
    case RESET_CAUSE_BROWNOUT_RESET:
      reset_cause_name = "BROWNOUT_RESET (BOR)";
      break;
  }

  return reset_cause_name;
}

static void configure_optional_byte_sram2_retention(bool mode)
{
  // Configure SRAM2 retention when system reset
  FLASH_OBProgramInitTypeDef ob_conf;

  HAL_FLASHEx_OBGetConfig(&ob_conf); // get current config
  if (!(ob_conf.USERConfig & OB_SRAM2_RST_NOT_ERASE) == mode)
  {
    FLASH_OBProgramInitTypeDef pOBInit;
    (void)memset((void *)&pOBInit, 0x00, sizeof(pOBInit));
    HAL_FLASH_Unlock();
    __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_ALL_ERRORS); // Clear the FLASH's pending flags.
    HAL_FLASH_OB_Unlock();
    HAL_FLASHEx_OBGetConfig(&pOBInit); // Get the Option bytes configuration.
    pOBInit.OptionType = OPTIONBYTE_USER;
    pOBInit.USERType = OB_USER_SRAM2_RST;
    pOBInit.USERConfig = (mode == true) ? OB_SRAM2_RST_NOT_ERASE : OB_SRAM2_RST_ERASE;
    HAL_FLASHEx_OBProgram(&pOBInit);
    // device will reboot at this stage
    HAL_FLASH_OB_Launch();
    HAL_FLASH_OB_Lock();
    HAL_FLASH_Lock();
  }
}

static void configure_debuggability(void)
{
#ifdef EnableDBGPOSTLPMode
  /* Be able to debug after wake-up from Standby. Consumption will be increased */
  HAL_DBGMCU_EnableDBGStopMode();
#else
  HAL_DBGMCU_DisableDBGStandbyMode();
  HAL_DBGMCU_DisableDBGStopMode();
#endif
}

static void determine_wake_up_mode(void)
{
  wakeup_from_standby = __HAL_PWR_GET_FLAG(PWR_FLAG_SBF);
  if (wakeup_from_standby != RESET)
  {
    /* Clear Standby flag */
    __HAL_PWR_CLEAR_FLAG(PWR_FLAG_SBF);
  }
  wakeup_from_stop = __HAL_PWR_GET_FLAG(PWR_FLAG_STOPF);
  if (wakeup_from_stop != RESET)
  {
    /* Clear STOP flag */
    __HAL_PWR_CLEAR_FLAG(PWR_FLAG_STOPF);
  }
}

static void determine_wake_up_source_from_standby(void)
{
  /* Check and Clear the Wakeup flag */
  /* This is flag clearance compulsory for the wakeup source to work for the next time */
  wakeup_from_RTC = __HAL_PWR_GET_FLAG(PWR_WAKEUP_FLAG7) && __NVIC_GetEnableIRQ(RTC_IRQn);
  if (wakeup_from_RTC != RESET)
  {
    __HAL_PWR_CLEAR_FLAG(PWR_WAKEUP_FLAG7);
  }
  wakeup_from_user_botton = __HAL_PWR_GET_FLAG(PWR_WAKEUP_FLAG2) && __NVIC_GetEnableIRQ(EXTI13_IRQn);
  if (wakeup_from_user_botton != RESET)
  {
    __HAL_PWR_CLEAR_FLAG(PWR_WAKEUP_FLAG2);
  }
}

static void configure_sram2_retention(void)
{
  __HAL_RCC_SRAM2_CLK_ENABLE();
  HAL_PWREx_DisableSRAM2ContentStandbyRetention(PWR_SRAM2_FULL_STANDBY_RETENTION);
  HAL_PWREx_DisableRAMsContentStopRetention(PWR_SRAM2_FULL_STOP);
  __HAL_RCC_SRAM2_CLK_DISABLE();
}

static void configure_gpio_retention(void)
{
  /* To demonstrate that we can maintain Output pin level in low power mode */

  /* Keep certain GPIO status under Standby */
  HAL_PWREx_EnableGPIOPullUp(PWR_GPIO_F, PWR_GPIO_BIT_13);
  HAL_PWREx_EnablePullUpPullDownConfig();
}

static void print_reset_info(void)
{
  if (reset_cause != RESET_CAUSE_UNKNOWN)
  {
    PRINT("System reset cause: %s.", reset_cause_get_name(reset_cause));
  }
}

static void print_wakeup_info(void)
{
  /* Check if the system was resumed from Standby mode */
  if (wakeup_from_standby != RESET)
  {
    PRINT_PLAIN("System wakes up from standby mode");
    /* Check and Clear the Wakeup flag */
    /* This is compulsory for the wakeup source to work for the next time */
    if (wakeup_from_user_botton != RESET)
    {
      PRINT_PLAIN(", wake up source: user button");
    }
    if (wakeup_from_RTC != RESET)
    {
      PRINT_PLAIN(", wake up source: RTC");
    }
    PRINT(".")
  }
  /* Check if the system was resumed from STOP2 mode */
  if (wakeup_from_stop != RESET)
  {
    PRINT_PLAIN("System wakes up from stop mode");
    if (wakeup_from_user_botton != RESET)
    {
      PRINT_PLAIN(", wake up source: user button");
    }
    if (wakeup_from_RTC != RESET)
    {
      PRINT_PLAIN(", wake up source: RTC");
    }
    PRINT(".")
  }
}

static void print_unhandled_interrupt(void)
{
  for (int i = 0; i <= LSECSSD_IRQn; i++)
  {
    if (__NVIC_GetPendingIRQ(i))
    {
      PRINT("%d interrupt is enabled and pending.", i);
    }
  }
  IRQn_Type IRQ[11] = {
      Reset_IRQn, NonMaskableInt_IRQn, HardFault_IRQn, MemoryManagement_IRQn,
      BusFault_IRQn, UsageFault_IRQn, SecureFault_IRQn, SVCall_IRQn,
      DebugMonitor_IRQn, PendSV_IRQn, SysTick_IRQn
  };
  for (int i = 0; i < 11; i++)
  {
    if (__NVIC_GetPendingIRQ(IRQ[i]))
    {
      PRINT("IRQ[%d] interrupt is enabled and pending.", i);
    }
  }

  if (DMA_TC_flag)
  {
    PRINT("Wake up due to DMA Transfer Complete");
  }
}

static void update_sram_content(void)
{
  /* Write something to backup SRAM and SRAM2 */
  HAL_PWR_EnableBkUpAccess();

  if (wakeup_from_user_botton || DMA_TC_flag)
  {
    *(uint32_t *) (BKPSRAM_BASE) += 1;
  }
  else
  {
    *(uint32_t *) (BKPSRAM_BASE) = 0;
  }

  PRINT("Current Backup SRAM content: %lu", *((uint32_t*) (BKPSRAM_BASE)));
}

void HAL_RTCEx_WakeUpTimerEventCallback(RTC_HandleTypeDef *hrtc)
{
  UNUSED(hrtc);
#ifdef LOW_POWER_MODE_STOP2
  wakeup_from_RTC = true;
#endif
}

void HAL_GPIO_EXTI_Rising_Callback(uint16_t GPIO_Pin)
{
  UNUSED(GPIO_Pin);
#ifdef LOW_POWER_MODE_STOP2
  wakeup_from_user_botton = true;
#endif
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
