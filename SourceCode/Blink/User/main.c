/**
  ******************************************************************************
  * @file    Project/main.c 
  * @author  MCD Application Team
  * @version V2.3.0
  * @date    16-June-2017
  * @brief   Main program body
   ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2014 STMicroelectronics</center></h2>
  *
  * Licensed under MCD-ST Liberty SW License Agreement V2, (the "License");
  * You may not use this file except in compliance with the License.
  * You may obtain a copy of the License at:
  *
  *        http://www.st.com/software_license_agreement_liberty_v2
  *
  * Unless required by applicable law or agreed to in writing, software 
  * distributed under the License is distributed on an "AS IS" BASIS, 
  * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  * See the License for the specific language governing permissions and
  * limitations under the License.
  *
  ******************************************************************************
  */ 


/* Includes ------------------------------------------------------------------*/
#include "stm8s.h"

/* Private defines -----------------------------------------------------------*/
#define LED_PORT (GPIOF)
#define LED_PIN  (GPIO_PIN_4)

#define LED_PORT1 (GPIOA)
#define LED_PIN1  (GPIO_PIN_3)

/* Private function prototypes -----------------------------------------------*/
void selfDelay(void);

/* Private functions ---------------------------------------------------------*/

void main(void)
{
  GPIO_Init(LED_PORT, (GPIO_Pin_TypeDef)LED_PIN, GPIO_MODE_OUT_PP_HIGH_FAST);
  
  GPIO_Init(LED_PORT1, (GPIO_Pin_TypeDef)LED_PIN1, GPIO_MODE_OUT_PP_HIGH_FAST);
  GPIO_WriteReverse(LED_PORT1, LED_PIN1);
  
  /* Infinite loop */
  while (1)
  {
    GPIO_WriteReverse(LED_PORT, LED_PIN);
    GPIO_WriteReverse(LED_PORT1, LED_PIN1);
    selfDelay();
  }
}

void selfDelay(void)
{
  int i = 0;
  for (; i <= 30000; i++)
  {
  }
}

#ifdef USE_FULL_ASSERT

/**
  * @brief  Reports the name of the source file and the source line number
  *   where the assert_param error has occurred.
  * @param file: pointer to the source file name
  * @param line: assert_param error line source number
  * @retval : None
  */
void assert_failed(u8* file, u32 line)
{ 
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1)
  {
  }
}
#endif


/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
