/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "assignment.h"

void SystemClock_Config(void);
uint8_t check_button_state(GPIO_TypeDef* PORT, uint8_t PIN);

uint8_t switch_state = 0;
uint8_t help = 0;
uint8_t pin_state = 0x00;
uint8_t old_pin_state = 0x10;

int main(void)
{
  /*Default system setup*/
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_SYSCFG);
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_PWR);

  NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_4);

  SystemClock_Config();


  /*
   * TASK - configure MCU peripherals so that button triggers external interrupt - EXTI.
   * Button must be connected to the GPIO port B, pin 4.
   * LED must be connected to the GPIO port A and its pin 4.
   *
   * Adjust values of macros defined in "assignment.h".
   * Implement function "checkButtonState" declared in "assignment.h".
   */


  /* Configure external interrupt - EXTI*/

  	  //type your code for EXTI configuration (priority, enable EXTI, setup EXTI for input pin, trigger edge) here:
   NVIC_SetPriority(EXTI4_IRQn, 2);
   NVIC_EnableIRQ(EXTI4_IRQn);

   /*set EXTI source PB4*/
   SYSCFG->EXTICR[1] |= (1 << 0U);
   //Enable interrupt from EXTI line 4
   EXTI->IMR |= EXTI_IMR_MR4;
   //Set EXTI trigger to falling edge
   EXTI->RTSR &= ~(EXTI_IMR_MR4);
   EXTI->FTSR |= EXTI_IMR_MR4;

  /* Configure GPIOB-4 pin as an input pin - button */

   RCC->AHBENR |= RCC_AHBENR_GPIOBEN;
   GPIOB->MODER &= ~(GPIO_MODER_MODER4);
   GPIOB->PUPDR &= ~(GPIO_PUPDR_PUPDR4);
   GPIOB->PUPDR |= GPIO_PUPDR_PUPDR4_0;

  /* Configure GPIOA-4 pin as an output pin - LED */

   RCC->AHBENR |= RCC_AHBENR_GPIOAEN;
   GPIOA->MODER &= ~(GPIO_MODER_MODER4);
   GPIOA->MODER |= GPIO_MODER_MODER4_0;
   GPIOA->OTYPER &= ~(GPIO_OTYPER_OT_4);
   GPIOA->OSPEEDR &= ~(GPIO_OSPEEDER_OSPEEDR4);
   GPIOA->PUPDR &= ~(GPIO_PUPDR_PUPDR4);


  while (1)
  {
	  pin_state = GPIOB->IDR & GPIO_IDR_4;
	  if(switch_state)
	  {

		  GPIOA->BSRR |= GPIO_BSRR_BS_4;
		  for(uint16_t i=0; i<0xFF00; i++){}
		  GPIOA->BRR |= GPIO_BRR_BR_4;
		  for(uint16_t i=0; i<0xFF00; i++){}
	  }
	  else
	  {
		  GPIOA->BRR |= GPIO_BRR_BR_4;
	  }
  }

}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  LL_FLASH_SetLatency(LL_FLASH_LATENCY_0);

  if(LL_FLASH_GetLatency() != LL_FLASH_LATENCY_0)
  {
  Error_Handler();  
  }
  LL_RCC_HSI_Enable();

   /* Wait till HSI is ready */
  while(LL_RCC_HSI_IsReady() != 1)
  {
    
  }
  LL_RCC_HSI_SetCalibTrimming(16);
  LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);
  LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_1);
  LL_RCC_SetAPB2Prescaler(LL_RCC_APB1_DIV_1);
  LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_HSI);

   /* Wait till System clock is ready */
  while(LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_HSI)
  {
  
  }
  LL_Init1msTick(8000000);
  LL_SYSTICK_SetClkSource(LL_SYSTICK_CLKSOURCE_HCLK);
  LL_SetSystemCoreClock(8000000);
}

#define PRESSED 0x00
#define RELEASED 0x10
uint8_t helpsamples = 0;

uint8_t checkButtonState(GPIO_TypeDef* PORT, uint8_t PIN, uint8_t edge, uint8_t samples_window, uint8_t samples_required)
{
	pin_state = GPIOB->IDR & GPIO_IDR_4;
	if (old_pin_state == RELEASED && pin_state == PRESSED && edge == TRIGGER_RISE){
		// rising
		old_pin_state = pin_state;
		while(1){
			if((GPIOB->IDR & GPIO_IDR_4) == RELEASED){
				helpsamples++;
				for(uint16_t i=0; i<0xFF00; i++){}
				if(helpsamples == samples_window){helpsamples=0;return 1;}
			}
			else{helpsamples=0;break;}
		}

		return 0;
	}
	if (old_pin_state == PRESSED && pin_state == RELEASED && edge == TRIGGER_FALL){
		// falling
		old_pin_state = pin_state;
		while(1){
			if((GPIOB->IDR & GPIO_IDR_4) == RELEASED){
				helpsamples++;
				for(uint16_t i=0; i<0xFF00; i++){}
				if(helpsamples == samples_window){helpsamples=0;return 1;}
			}
			else{helpsamples=0;break;}
		}
		return 0;
	}
	/*if ((old_pin_state == RELEASED && pin_state == RELEASED) || (old_pin_state == PRESSED && pin_state == PRESSED)){
		// 0 or 1
		pin_state = GPIOB->IDR & GPIO_IDR_4;
		old_pin_state = pin_state;
		return 0;
	}*/
	old_pin_state = pin_state;
	helpsamples=0;
}


void EXTI4_IRQHandler(void)
{
	if(checkButtonState(GPIO_PORT_BUTTON,
						GPIO_PIN_BUTTON,
						1,
						BUTTON_EXTI_SAMPLES_WINDOW,
						BUTTON_EXTI_SAMPLES_REQUIRED))
	{
		switch_state ^= 1;
	}

	/* Clear EXTI4 pending register flag */

	EXTI->PR |= (EXTI_PR_PIF4);
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

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
void assert_failed(char *file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
