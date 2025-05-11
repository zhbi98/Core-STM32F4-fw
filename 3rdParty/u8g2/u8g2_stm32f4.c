/**
 * @file u8g2.c
 *
 */

/*********************
 *      INCLUDES
 *********************/

#include "stm32f4xx_hal.h"
#include "gpio.h"
#include "spi.h"
#include "u8g2.h"
#include "time.h"

/*********************
 *      DEFINES
 *********************/

#define DEV_ADDR   0x3C
#define TX_TIMEOUT 100U
#define _USE_SPI   1U

/**********************
 *      TYPEDEFS
 **********************/

SPI_HandleTypeDef * _SPI_DEV = NULL; /**SPI Interface*/
I2C_HandleTypeDef * _I2C_DEV = NULL; /**I2C Interface*/

/**********************
 *   GLOBAL FUNCTIONS
 **********************/

/**
 * Creates a blocking millisecond-level delay using busy loops.
 * @param ms Number of millisecond to delay 
 * (valid range depends on CPU frequency).
 */
static void sleep_ms(uint32_t ms)
{
    volatile uint32_t i;

    for (; ms > 0; ms--)
        for (i = 12000; 
            i > 0; i--);
}

/**
  * @brief  Initialize the SPI according to the specified parameters
  *         in the SPI_InitTypeDef and initialize the associated handle.
  * @param  hspi pointer to a SPI_HandleTypeDef structure that contains
  *               the configuration information for SPI module.
  * @retval HAL status
  */
uint8_t u8x8_stm32_gpio_and_delay(u8x8_t *u8x8, 
    uint8_t msg, uint8_t arg_int, void *arg_ptr)
{
    /**STM32 supports HW SPI, 
    Remove unused cases like U8X8_MSG_DELAY_XXX & 
    U8X8_MSG_GPIO_XXX */
    switch (msg) {
    case U8X8_MSG_GPIO_AND_DELAY_INIT:
        /*Insert codes for initialization*/
        break;

    case U8X8_MSG_DELAY_MILLI:
        /* ms Delay */
        sleep_ms(arg_int);
        break;

#ifdef _USE_SPI
        /*SPI Interface*/
    case U8X8_MSG_GPIO_CS:
        /*Insert codes for SS pin control */
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, arg_int);
        break;

    case U8X8_MSG_GPIO_DC:
        /*Insert codes for DC pin control */
        HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, arg_int);
        break;

    case U8X8_MSG_GPIO_RESET:
        /*Insert codes for RST pin control*/
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, arg_int);
        break;

    case U8X8_MSG_GPIO_SPI_CLOCK:
        /*Insert codes for CLOCK pin control */
        break;

    case U8X8_MSG_GPIO_SPI_DATA:
        /*Insert codes for DATA pin control */
        break;

#endif /*_USE_SPI*/

#ifdef _USE_I2C
    case U8X8_MSG_GPIO_I2C_CLOCK:
        /*Insert codes for CLOCK pin control */
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, arg_int);
        break;
    case U8X8_MSG_GPIO_I2C_DATA:
        /*Insert codes for CLOCK pin control */
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, arg_int);
        break;
#endif /*_USE_I2C*/
    }
    return 1;
}

#ifdef _USE_SPI

/**
  * @brief  Initialize the SPI according to the specified parameters
  *         in the SPI_InitTypeDef and initialize the associated handle.
  * @param  hspi pointer to a SPI_HandleTypeDef structure that contains
  *               the configuration information for SPI module.
  * @retval HAL status
  */
uint8_t u8x8_byte_stm32_hw_spi(u8x8_t *u8x8, 
    uint8_t msg, uint8_t arg_int, void *arg_ptr)
{
    HAL_StatusTypeDef _res = HAL_ERROR;

    switch (msg) {
    case U8X8_MSG_BYTE_SEND:
        /*Insert codes to transmit data*/
        _res = HAL_SPI_Transmit(&hspi3, arg_ptr, 
            arg_int, TX_TIMEOUT);
        if (_res != HAL_OK) return 0;
        break;

    case U8X8_MSG_BYTE_INIT:
        /*Insert codes to begin 
        SPI transmission*/
        break;

    case U8X8_MSG_BYTE_SET_DC:
        /*Control DC pin, U8X8_MSG_GPIO_DC 
        will be called*/
        u8x8_gpio_SetDC(u8x8, arg_int);
        break;

    case U8X8_MSG_BYTE_START_TRANSFER:
        /* Select slave, U8X8_MSG_GPIO_CS will be called */
        u8x8_gpio_SetCS(u8x8, 
            u8x8->display_info->chip_enable_level);
        sleep_ms(2);
        break;

    case U8X8_MSG_BYTE_END_TRANSFER:
        sleep_ms(2);
        /* Insert codes to end SPI transmission */
        u8x8_gpio_SetCS(u8x8, 
            u8x8->display_info->chip_disable_level);
        break;

    default: return 0;
        break;
    }
    return 1;
}

#endif /*_USE_SPI*/

#ifdef _USE_I2C

/**
  * @brief  Initialize the SPI according to the specified parameters
  *         in the SPI_InitTypeDef and initialize the associated handle.
  * @param  hspi pointer to a SPI_HandleTypeDef structure that contains
  *               the configuration information for SPI module.
  * @retval HAL status
  */
uint8_t u8x8_byte_stm32_hw_i2c(u8x8_t *u8x8, 
    uint8_t msg, uint8_t arg_int, void *arg_ptr)
{
    /**u8g2/u8x8 will never send more than 
    32 bytes between START_TRANSFER and END_TRANSFER */
    static uint8_t buffer[32] = {0};
    static uint8_t buf_idx = 0;
    uint8_t * data = NULL;

    switch (msg) {
    case U8X8_MSG_BYTE_SEND:
        data = (uint8_t *) arg_ptr;
        while (arg_int > 0) {
            buffer[buf_idx++] = *data;
            data++;
            arg_int--;
        }
        break;

    case U8X8_MSG_BYTE_INIT:
        /*add your custom code to init i2c subsystem*/
        break;

    case U8X8_MSG_BYTE_SET_DC:
        break;

    case U8X8_MSG_BYTE_START_TRANSFER:
        buf_idx = 0;
        break;

    case U8X8_MSG_BYTE_END_TRANSFER:
        HAL_I2C_Master_Transmit(_I2C_DEV, (DEV_ADDR << 1), 
            buffer, buf_idx, TX_TIMEOUT);
        break;

    default: return 0;
        break;
    }
    return 1;
}

#endif /*_USE_I2C*/

/**
  * @brief  Initialize the SPI according to the specified parameters
  *         in the SPI_InitTypeDef and initialize the associated handle.
  * @param  hspi pointer to a SPI_HandleTypeDef structure that contains
  *               the configuration information for SPI module.
  * @retval HAL status
  */
void u8g2_init(u8g2_t *u8g2)
{
    u8g2_Setup_ssd1312_128x64_noname_f(u8g2, U8G2_R2, 
        u8x8_byte_stm32_hw_spi, u8x8_stm32_gpio_and_delay);

    u8g2_SetPowerSave(u8g2, 1);
    u8g2_InitDisplay(u8g2);
    u8g2_ClearDisplay(u8g2);
    u8g2_ClearBuffer(u8g2);
    u8g2_SetPowerSave(u8g2, 0);
}
