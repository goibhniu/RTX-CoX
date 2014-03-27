/**
  ******************************************************************************
  * @file    IO_Toggle/main.c 
  * @author  MCD Application Team
  * @version V1.0.0
  * @date    19-September-2011
  * @brief   Main program body
  ******************************************************************************
  * @attention
  *
  * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
  * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
  * TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
  * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
  * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
  * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
  *
  * <h2><center>&copy; COPYRIGHT 2011 STMicroelectronics</center></h2>
  ******************************************************************************  
  */ 

/* Includes ------------------------------------------------------------------*/



#include "cmsis_os.h"
#include "xhw_types.h"
#include "xhw_memmap.h"
#include "xgpio.h"
#include "xuart.h"
#include "xsysctl.h"



#define __FI        1                   /* Font index 16x24                  */

osThreadId tid_phaseA;                  /* Thread id of thread: phase_a      */
osThreadId tid_phaseB;                  /* Thread id of thread: phase_b      */
osThreadId tid_phaseC;                  /* Thread id of thread: phase_c      */
osThreadId tid_phaseD;                  /* Thread id of thread: phase_d      */
osThreadId tid_clock;                   /* Thread id of thread: clock        */
osThreadId tid_lcd;                     /* Thread id of thread: lcd          */

//osMutexId mut_GLCD;                     /* Mutex to controll GLCD access     */

#define LED_A      0
#define LED_B      1
#define LED_C      2
#define LED_D      3
#define LED_CLK    7


/*----------------------------------------------------------------------------
  switch LED on
 *---------------------------------------------------------------------------*/
void LED_on  (unsigned char led) {
//	 STM_EVAL_LEDOn (led);


}

/*----------------------------------------------------------------------------
  switch LED off
 *---------------------------------------------------------------------------*/
void LED_off (unsigned char led) {
//  STM_EVAL_LEDOff(led);

}


/*----------------------------------------------------------------------------
  Function 'signal_func' called from multiple tasks
 *---------------------------------------------------------------------------*/
void signal_func (osThreadId tid) {
  osSignalSet(tid_clock, 0x0100);           /* set signal to clock thread    */
  osDelay(500);                             /* delay 500ms                   */
  osSignalSet(tid_clock, 0x0100);           /* set signal to clock thread    */
  osDelay(500);                             /* delay 500ms                   */
  osSignalSet(tid, 0x0001);                 /* set signal to thread 'thread' */
  osDelay(500);                             /* delay 500ms                   */
}

/*----------------------------------------------------------------------------
  Thread 1 'phaseA': Phase A output
 *---------------------------------------------------------------------------*/
void phaseA (void const *argument) {
  for (;;) {
    osSignalWait(0x0001, osWaitForever);    /* wait for an event flag 0x0001 */
    LED_on (LED_A);
    signal_func (tid_phaseB);               /* call common signal function   */
    LED_off(LED_A);
  }
}

/*----------------------------------------------------------------------------
  Thread 2 'phaseB': Phase B output
 *---------------------------------------------------------------------------*/
void phaseB (void const *argument) {
  for (;;) {
    osSignalWait(0x0001, osWaitForever);    /* wait for an event flag 0x0001 */
    LED_on (LED_B);
    signal_func (tid_phaseC);               /* call common signal function   */
    LED_off(LED_B);
  }
}

/*----------------------------------------------------------------------------
  Thread 3 'phaseC': Phase C output
 *---------------------------------------------------------------------------*/
void phaseC (void const *argument) {
  for (;;) {
    osSignalWait(0x0001, osWaitForever);    /* wait for an event flag 0x0001 */
    LED_on (LED_C);
    signal_func (tid_phaseD);               /* call common signal function   */
    LED_off(LED_C);
  }
}

/*----------------------------------------------------------------------------
  Thread 4 'phaseD': Phase D output
 *---------------------------------------------------------------------------*/
void phaseD (void const *argument) {
  for (;;) {
    osSignalWait(0x0001, osWaitForever);    /* wait for an event flag 0x0001 */
    LED_on (LED_D);
    signal_func (tid_phaseA);               /* call common signal function   */
    LED_off(LED_D);
  }
}

/*----------------------------------------------------------------------------
  Thread 5 'clock': Signal Clock
 *---------------------------------------------------------------------------*/
void clock (void const *argument) {
  for (;;) {
    osSignalWait(0x0100, osWaitForever);    /* wait for an event flag 0x0100 */
   // LED_on (LED_CLK);
    osDelay(80);                            /* delay 80ms                    */
  //  LED_off(LED_CLK);
  }
}

/*----------------------------------------------------------------------------
  Thread 6 'lcd': LCD Control task
 *---------------------------------------------------------------------------*/
void lcd (void const *argument) {

  for (;;) {

    osDelay(4000);                          /* delay 4s                      */


    osDelay(4000);                          /* delay 4s                      */
  }
}




osThreadDef(phaseA, osPriorityNormal, 1, 0);
osThreadDef(phaseB, osPriorityNormal, 1, 0);
osThreadDef(phaseC, osPriorityNormal, 1, 0);
osThreadDef(phaseD, osPriorityNormal, 1, 0);
osThreadDef(clock,  osPriorityNormal, 1, 0);
osThreadDef(lcd,    osPriorityNormal, 1, 0);
/* Private typedef -----------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
void Delay( uint32_t nCount);
/* Private functions ---------------------------------------------------------*/
void LED_Init(void) {
	  /* GPIOD Periph clock enable */
	  //RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
	  xSysCtlPeripheralReset(xSYSCTL_PERIPH_GPIOD);

	  xGPIOSPinTypeGPIOOutput(PD12);
	  xGPIOSPinTypeGPIOOutput(PD13);
	  xGPIOSPinTypeGPIOOutput(PD14);
	  xGPIOSPinTypeGPIOOutput(PD15);
	  /* Configure PD12, PD13, PD14 and PD15 in output pushpull mode */
	//  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12 | GPIO_Pin_13| GPIO_Pin_14| GPIO_Pin_15;
	//  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	//  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	//  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	//  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	//  GPIO_Init(GPIOD, &GPIO_InitStructure);
}
void uart_test()
 {
     unsigned char ch[] = "Hello, UART!";

     //SysCtlKeyAddrUnlock();
     //xHWREG(SYSCLK_PWRCON) |= SYSCLK_PWRCON_XTL12M_EN;

     //SysCtlDelay(10000);

     xSPinTypeUART(UART1RX,PA10);
     xSPinTypeUART(UART1TX,PA9);

     xSysCtlPeripheralReset(xSYSCTL_PERIPH_UART1);
     xSysCtlPeripheralEnable(xSYSCTL_PERIPH_UART1);
     //SysCtlPeripheralClockSourceSet(SYSCTL_PERIPH_UART_S_EXT12M);

     UARTConfigSet(USART1_BASE, 115200, (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE | UART_CONFIG_PAR_NONE));

     UARTEnable(USART1_BASE, (UART_BLOCK_UART | UART_BLOCK_TX | UART_BLOCK_RX));

     UARTBufferWrite(USART1_BASE, ch, sizeof(ch)/sizeof(*ch));

     while(1);
 }

/*----------------------------------------------------------------------------
  Main Thread 'main'
 *---------------------------------------------------------------------------*/
int main (void) {


  LED_Init ();                              /* Initialize the LEDs           */


  /* PD12 to be toggled */
  GPIOPinSet(GPIOD_BASE, xGPIO_PIN_12);

  /* Insert delay */
  Delay(0x3FFFFF);

  /* PD13 to be toggled */
  GPIOPinSet(GPIOD_BASE, xGPIO_PIN_13);

  /* Insert delay */
  Delay(0x3FFFFF);

  /* PD14 to be toggled */
  GPIOPinSet(GPIOD_BASE, xGPIO_PIN_14);

  /* Insert delay */
  Delay(0x3FFFFF);

  /* PD15 to be toggled */
  GPIOPinSet(GPIOD_BASE, xGPIO_PIN_15);

  /* Insert delay */
  Delay(0x7FFFFF);

 // GPIO_ResetBits(GPIOD, GPIO_Pin_12|GPIO_Pin_13|GPIO_Pin_14|GPIO_Pin_15);

  /* Insert delay */
  Delay(0xFFFFFF);


  tid_phaseA = osThreadCreate(osThread(phaseA), NULL);
  LED_on(0);
  tid_phaseB = osThreadCreate(osThread(phaseB), NULL);
  LED_on(1);
  tid_phaseC = osThreadCreate(osThread(phaseC), NULL);
  LED_on(2);
  tid_phaseD = osThreadCreate(osThread(phaseD), NULL);
  LED_on(3);
  tid_clock  = osThreadCreate(osThread(clock),  NULL);
  LED_off(0);

  osSignalSet(tid_phaseA, 0x0001);          /* set signal to phaseA thread   */
  LED_off(1);
  osDelay(osWaitForever);
  LED_off(2);
  while(1);
}
/** @addtogroup STM32F4_Discovery_Peripheral_Examples
  * @{
  */

/** @addtogroup IO_Toggle
  * @{
  */ 



/**
  * @brief  Main program
  * @param  None
  * @retval None
  */


/**
  * @brief  Delay Function.
  * @param  nCount:specifies the Delay time length.
  * @retval None
  */
void Delay(uint32_t nCount)
{
  while(nCount--)
  {
  }
}

#ifdef  USE_FULL_ASSERT

/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1)
  {
  }
}
#endif

/**
  * @}
  */ 

/**
  * @}
  */ 

/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/
