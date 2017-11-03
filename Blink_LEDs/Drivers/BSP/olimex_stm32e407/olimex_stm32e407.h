/**
  ******************************************************************************
  * @file    olimex_stm32e407.h
  * @author  SAI CHARAN BANDI
  * @version V1.0
  * @date    25-October-2017
  * @brief   This file contains definitions for STM32E407's User- LED, 
  *          and push-button hardware resources.
  ******************************************************************************
  * MORE DESCRIPTION OF FILE HERE
  *
  * I find it would be more convenient to define the LED and PUSH
  * BUTTON details directly on the main file
  * 
  * Achtung: olimex_stm32e407.c is yet to be programmed for the
  * respective changes !!!!
  ******************************************************************************
  */ 
  
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __STM32E407_H
#define __STM32E407_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"
   
/** @addtogroup BSP
  * @{
  */ 

/** @addtogroup STM32E407
  * @{
  */ 
      
/** @addtogroup STM32E407_LOW_LEVEL
  * @{
  */ 

/** @defgroup STM32E407_LOW_LEVEL_Exported_Types
  * @{
  */
typedef enum 
{
  LED1 = 0,
  LED2 = 1,
  LED3 = 2,
  LED4 = 3
}Led_TypeDef;

typedef enum 
{  
  BUTTON_USER_1 = 0,
  BUTTON_USER_2 = 1
}Button_TypeDef;

typedef enum 
{  
  BUTTON_MODE_GPIO = 0,
  BUTTON_MODE_EXTI = 1
}ButtonMode_TypeDef;
  

/**
  * @}
  */ 

/** @defgroup STM32E407_LOW_LEVEL_Exported_Constants
  * @{
  */ 

/** 
  * @brief  Define for STM32E407 board  
  */ 
#if !defined (USE_STM32E407)
 #define USE_STM32E407
#endif

/** @addtogroup STM32E407_LOW_LEVEL_LED
  * @{
  */
#define LEDn                                  4

#define LED_GREEN_PIN                         GPIO_PIN_3
#define LED_GREEN_GPIO_PORT                   GPIOD
#define LED_GREEN_GPIO_CLK_ENABLE()           __GPIOD_CLK_ENABLE()
#define LED_GREEN_GPIO_CLK_DISABLE()          __GPIOD_CLK_DISABLE()
  
#define LED_BLUE_PI N                         GPIO_PIN_4
#define LED_BLUE_GPIO_PORT                    GPIOD
#define LED_BLUE_GPIO_CLK_ENABLE()            __GPIOD_CLK_ENABLE()
#define LED_BLUE_GPIO_CLK_DISABLE()           __GPIOD_CLK_DISABLE()
  
#define LED_YELLOW_PIN                        GPIO_PIN_5
#define LED_YELLOW_GPIO_PORT                  GPIOD
#define LED_YELLOW_GPIO_CLK_ENABLE()          __GPIOD_CLK_ENABLE()
#define LED_YELLOW_GPIO_CLK_DISABLE()         __GPIOD_CLK_DISABLE()
  
#define LED_RED_PIN                           GPIO_PIN_6
#define LED_RED_GPIO_PORT                     GPIOD
#define LED_RED_GPIO_CLK                      RCC_AHB1Periph_GPIOC
#define LED_RED_GPIO_CLK_ENABLE()             __GPIOC_CLK_ENABLE()
#define LED_RED_GPIO_CLK_DISABLE()            __GPIOC_CLK_DISABLE()

#define LEDx_GPIO_CLK_ENABLE(__INDEX__)       (((__INDEX__) == 0) ? LED_GREEN_GPIO_CLK_ENABLE() :\
                                              ((__INDEX__) == 1) ? LED_BLUE_GPIO_CLK_ENABLE() :\
                                              ((__INDEX__) == 2) ? LED_YELLOW_GPIO_CLK_ENABLE() : LED_RED_GPIO_CLK_ENABLE())

#define LEDx_GPIO_CLK_DISABLE(__INDEX__)      (((__INDEX__) == 0) ? LED_GREEN_GPIO_CLK_DISABLE() :\
                                              ((__INDEX__) == 1) ? LED_BLUE_GPIO_CLK_DISABLE() :\
                                              ((__INDEX__) == 2) ? LED_YELLOW_GPIO_CLK_DISABLE() : LED_RED_GPIO_CLK_DISABLE())

/**
  * @}
  */ 
  
/** @addtogroup STM32E407_LOW_LEVEL_BUTTON
  * @{
  */  
/* Joystick pins are connected to IO Expander (accessible through I2C1 interface) */
#define BUTTONn                              2

/**
  * @brief Wakeup push-button
  */
#define BUTTON_USER_1_PIN                    GPIO_PIN_0
#define BUTTON_USER_1_GPIO_PORT              GPIOD
#define BUTTON_USER_1_GPIO_CLK_ENABLE()      __GPIOA_CLK_ENABLE()
#define BUTTON_USER_1_GPIO_CLK_DISABLE()     __GPIOA_CLK_DISABLE()
#define BUTTON_USER_1_EXTI_IRQn              EXTI0_IRQn

/**
  * @brief Tamper push-button
  */
#define BUTTON_USER_2_PIN                    GPIO_PIN_1
#define BUTTON_USER_2_GPIO_PORT              GPIOD
#define BUTTON_USER_2_GPIO_CLK_ENABLE()      __GPIOC_CLK_ENABLE()
#define BUTTON_USER_2_GPIO_CLK_DISABLE()     __GPIOC_CLK_DISABLE()
#define BUTTON_USER_2_EXTI_IRQn              EXTI15_10_IRQn


#define BUTTONx_GPIO_CLK_ENABLE(__INDEX__)    (((__INDEX__) == 0) ? BUTTON_USER_1_GPIO_CLK_ENABLE() :\
                                               ((__INDEX__) == 1) ? BUTTON_USER_2_GPIO_CLK_ENABLE() : KEY_BUTTON_GPIO_CLK_ENABLE())

#define BUTTONx_GPIO_CLK_DISABLE(__INDEX__)    (((__INDEX__) == 0) ? BUTTON_USER_1_GPIO_CLK_DISABLE() :\
                                                ((__INDEX__) == 1) ? BUTTON_USER_2_GPIO_CLK_DISABLE() : KEY_BUTTON_GPIO_CLK_DISABLE())

/**
  * @}
  */ 

/**
  * @}
  */ 
  
/** @defgroup STM32E407_LOW_LEVEL_Exported_Macros
  * @{
  */  
/**
  * @}
  */ 


/** @defgroup STM32E407_LOW_LEVEL_Exported_Functions
  * @{
  */
uint32_t         BSP_GetVersion(void);  
void             BSP_LED_Init(Led_TypeDef Led);
void             BSP_LED_On(Led_TypeDef Led);
void             BSP_LED_Off(Led_TypeDef Led);
void             BSP_LED_Toggle(Led_TypeDef Led);
void             BSP_PB_Init(Button_TypeDef Button, ButtonMode_TypeDef Button_Mode);
uint32_t         BSP_PB_GetState(Button_TypeDef Button);

/**
  * @}
  */

/**
  * @}
  */ 

/**
  * @}
  */ 

/**
  * @}
  */

#ifdef __cplusplus
}
#endif

#endif /* __STM32E407_H */
 
