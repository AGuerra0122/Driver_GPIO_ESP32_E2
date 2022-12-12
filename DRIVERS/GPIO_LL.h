 // FileName:        GPIO_LL.h
 // Dependencies:    None
 // Processor:       ESP32
 // Board:           ESP32
 // Program version: Espressif 2.7.0
 // Company:         Espressif
 // Description:
 // Authors:         Jesús Adrián Guerra Delgado
 //                  Andrew Joshua Barrientos Lonzo
 //                  Jaime Alejandro Herrera Armendariz
 // Updated:         12/2022
/*
 * SPDX-FileCopyrightText: 2015-2021 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

// The LL layer for ESP32 GPIO register operations

#pragma once

#include <stdbool.h>
#include "soc/soc.h"
#include "soc/gpio_periph.h"
#include "soc/gpio_struct.h"
#include "soc/rtc_cntl_reg.h"
#include "soc/rtc_io_reg.h"
#include "hal/gpio_types.h"
#include "hal/misc.h"

#ifdef __cplusplus
extern "C" {
#endif
/**************************************************************************
* Function: GPIO_LL_GET_HW
* Preconditions:
* Overview: Esta es una macro funcion para entregar la direccion de un puerto
* Input: El numero del puerto del que se quiere la direccion
* Output: La direccion del puerto deceado
*
*****************************************************************************/
// Se toma la direccion del GPIO en hardware al entregar un numero de gpio
#define GPIO_LL_GET_HW(num) (((num) == 0) ? (&GPIO) : NULL)

#define GPIO_LL_APP_CPU_INTR_ENA      (BIT(0))
#define GPIO_LL_APP_CPU_NMI_INTR_ENA  (BIT(1))
#define GPIO_LL_PRO_CPU_INTR_ENA      (BIT(2))
#define GPIO_LL_PRO_CPU_NMI_INTR_ENA  (BIT(3))
#define GPIO_LL_SDIO_EXT_INTR_ENA     (BIT(4))

/**************************************************************************
* Function: gpio_ll_act_left_op
* Preconditions:
* Overview: En esta funcion se establecen como salidas todos los pines GPIO del lado izquierdo en la tarjeta ESP32.
* Input:
* Output:
*
*****************************************************************************/
static inline void gpio_ll_act_left_op(gpio_dev_t *hw){
	hw->enable_w1ts = (0x1 << 12);
	hw->enable_w1ts = (0x1 << 13);
	hw->enable_w1ts = (0x1 << 14);
	hw->enable_w1ts = (0x1 << 25);
	hw->enable_w1ts = (0x1 << 26);
	hw->enable_w1ts = (0x1 << 27);
	HAL_FORCE_MODIFY_U32_REG_FIELD(hw->enable1_w1ts, data, (0x1 << (32 - 32)));
	HAL_FORCE_MODIFY_U32_REG_FIELD(hw->enable1_w1ts, data, (0x1 << (33 - 32)));
	HAL_FORCE_MODIFY_U32_REG_FIELD(hw->enable1_w1ts, data, (0x1 << (34 - 32)));
	HAL_FORCE_MODIFY_U32_REG_FIELD(hw->enable1_w1ts, data, (0x1 << (35 - 32)));
	HAL_FORCE_MODIFY_U32_REG_FIELD(hw->enable1_w1ts, data, (0x1 << (36 - 32)));
	HAL_FORCE_MODIFY_U32_REG_FIELD(hw->enable1_w1ts, data, (0x1 << (39 - 32)));
}
/**************************************************************************
* Function: gpio_ll_deact_left_op
* Preconditions:
* Overview: En esta funcion se destactivan todos los pines GPIO del lado izquierdo de la tarjeta ESP32 como salidas
* Input:
* Output:
*
*****************************************************************************/
static inline void gpio_ll_deact_left_op(gpio_dev_t *hw){
	hw->enable_w1tc = (0x1 << 12);
	REG_WRITE(GPIO_FUNC0_OUT_SEL_CFG_REG + (12 * 4), SIG_GPIO_OUT_IDX);
	hw->enable_w1tc = (0x1 << 13);
	REG_WRITE(GPIO_FUNC0_OUT_SEL_CFG_REG + (13 * 4), SIG_GPIO_OUT_IDX);
	hw->enable_w1tc = (0x1 << 14);
	REG_WRITE(GPIO_FUNC0_OUT_SEL_CFG_REG + (14 * 4), SIG_GPIO_OUT_IDX);
	hw->enable_w1tc = (0x1 << 25);
	REG_WRITE(GPIO_FUNC0_OUT_SEL_CFG_REG + (25 * 4), SIG_GPIO_OUT_IDX);
	hw->enable_w1tc = (0x1 << 26);
	REG_WRITE(GPIO_FUNC0_OUT_SEL_CFG_REG + (26 * 4), SIG_GPIO_OUT_IDX);
	hw->enable_w1tc = (0x1 << 27);
	REG_WRITE(GPIO_FUNC0_OUT_SEL_CFG_REG + (27 * 4), SIG_GPIO_OUT_IDX);
	HAL_FORCE_MODIFY_U32_REG_FIELD(hw->enable1_w1tc, data, (0x1 << (32 - 32)));
	REG_WRITE(GPIO_FUNC0_OUT_SEL_CFG_REG + (32 * 4), SIG_GPIO_OUT_IDX);
	HAL_FORCE_MODIFY_U32_REG_FIELD(hw->enable1_w1tc, data, (0x1 << (33 - 32)));
	REG_WRITE(GPIO_FUNC0_OUT_SEL_CFG_REG + (33 * 4), SIG_GPIO_OUT_IDX);
	HAL_FORCE_MODIFY_U32_REG_FIELD(hw->enable1_w1tc, data, (0x1 << (34 - 32)));
	REG_WRITE(GPIO_FUNC0_OUT_SEL_CFG_REG + (34 * 4), SIG_GPIO_OUT_IDX);
	HAL_FORCE_MODIFY_U32_REG_FIELD(hw->enable1_w1tc, data, (0x1 << (35 - 32)));
	REG_WRITE(GPIO_FUNC0_OUT_SEL_CFG_REG + (35 * 4), SIG_GPIO_OUT_IDX);
	HAL_FORCE_MODIFY_U32_REG_FIELD(hw->enable1_w1tc, data, (0x1 << (36 - 32)));
	REG_WRITE(GPIO_FUNC0_OUT_SEL_CFG_REG + (36 * 4), SIG_GPIO_OUT_IDX);
	HAL_FORCE_MODIFY_U32_REG_FIELD(hw->enable1_w1tc, data, (0x1 << (39 - 32)));
	REG_WRITE(GPIO_FUNC0_OUT_SEL_CFG_REG + (39 * 4), SIG_GPIO_OUT_IDX);
}
/**************************************************************************
* Function: gpio_ll_act_right_op
* Preconditions:
* Overview: En esta funcion se establecen como salidas todos los pines GPIO del lado derecho de la tarjeta ESP32
* Input:
* Output:
*
*****************************************************************************/
static inline void gpio_ll_act_right_op(gpio_dev_t *hw){
	hw->enable_w1ts = (0x1 << 1);
	hw->enable_w1ts = (0x1 << 2);
	hw->enable_w1ts = (0x1 << 3);
	hw->enable_w1ts = (0x1 << 4);
	hw->enable_w1ts = (0x1 << 5);
	hw->enable_w1ts = (0x1 << 15);
	hw->enable_w1ts = (0x1 << 16);
	hw->enable_w1ts = (0x1 << 17);
	hw->enable_w1ts = (0x1 << 18);
	hw->enable_w1ts = (0x1 << 19);
	hw->enable_w1ts = (0x1 << 21);
	hw->enable_w1ts = (0x1 << 22);
	hw->enable_w1ts = (0x1 << 23);
}
/**************************************************************************
* Function: gpio_ll_deact_right_op
* Preconditions:
* Overview: Esta funcion se desactivan todos los pines GPIO del lado derecho de la tarjeta ESP32 como salidas
* Input:
* Output:
*
*****************************************************************************/
static inline void gpio_ll_deact_right_op(gpio_dev_t *hw){
	hw->enable_w1tc = (0x1 << 1);
	REG_WRITE(GPIO_FUNC0_OUT_SEL_CFG_REG + (1 * 4), SIG_GPIO_OUT_IDX);
	hw->enable_w1tc = (0x1 << 2);
	REG_WRITE(GPIO_FUNC0_OUT_SEL_CFG_REG + (2 * 4), SIG_GPIO_OUT_IDX);
	hw->enable_w1tc = (0x1 << 3);
	REG_WRITE(GPIO_FUNC0_OUT_SEL_CFG_REG + (3 * 4), SIG_GPIO_OUT_IDX);
	hw->enable_w1tc = (0x1 << 4);
	REG_WRITE(GPIO_FUNC0_OUT_SEL_CFG_REG + (4 * 4), SIG_GPIO_OUT_IDX);
	hw->enable_w1tc = (0x1 << 5);
	REG_WRITE(GPIO_FUNC0_OUT_SEL_CFG_REG + (5 * 4), SIG_GPIO_OUT_IDX);
	hw->enable_w1tc = (0x1 << 15);
	REG_WRITE(GPIO_FUNC0_OUT_SEL_CFG_REG + (15 * 4), SIG_GPIO_OUT_IDX);
	hw->enable_w1tc = (0x1 << 16);
	REG_WRITE(GPIO_FUNC0_OUT_SEL_CFG_REG + (16 * 4), SIG_GPIO_OUT_IDX);
	hw->enable_w1tc = (0x1 << 17);
	REG_WRITE(GPIO_FUNC0_OUT_SEL_CFG_REG + (17 * 4), SIG_GPIO_OUT_IDX);
	hw->enable_w1tc = (0x1 << 18);
	REG_WRITE(GPIO_FUNC0_OUT_SEL_CFG_REG + (18 * 4), SIG_GPIO_OUT_IDX);
	hw->enable_w1tc = (0x1 << 19);
	REG_WRITE(GPIO_FUNC0_OUT_SEL_CFG_REG + (19 * 4), SIG_GPIO_OUT_IDX);
	hw->enable_w1tc = (0x1 << 21);
	REG_WRITE(GPIO_FUNC0_OUT_SEL_CFG_REG + (21 * 4), SIG_GPIO_OUT_IDX);
	hw->enable_w1tc = (0x1 << 22);
	REG_WRITE(GPIO_FUNC0_OUT_SEL_CFG_REG + (22 * 4), SIG_GPIO_OUT_IDX);
	hw->enable_w1tc = (0x1 << 23);
	REG_WRITE(GPIO_FUNC0_OUT_SEL_CFG_REG + (23 * 4), SIG_GPIO_OUT_IDX);
}
/**************************************************************************
* Function: gpio_ll_act_left_ip
* Preconditions:
* Overview: En esta funcion se establecen todos los pines GPIO del lado izquierdo de la tarjeta ESP32 como entradas
* Input:
* Output:
*
*****************************************************************************/
static inline void gpio_ll_act_left_ip(gpio_dev_t *hw){
	PIN_INPUT_ENABLE(GPIO_PIN_MUX_REG[12]);
	PIN_INPUT_ENABLE(GPIO_PIN_MUX_REG[13]);
	PIN_INPUT_ENABLE(GPIO_PIN_MUX_REG[14]);
	PIN_INPUT_ENABLE(GPIO_PIN_MUX_REG[25]);
	PIN_INPUT_ENABLE(GPIO_PIN_MUX_REG[26]);
	PIN_INPUT_ENABLE(GPIO_PIN_MUX_REG[27]);
	PIN_INPUT_ENABLE(GPIO_PIN_MUX_REG[32]);
	PIN_INPUT_ENABLE(GPIO_PIN_MUX_REG[33]);
	PIN_INPUT_ENABLE(GPIO_PIN_MUX_REG[34]);
	PIN_INPUT_ENABLE(GPIO_PIN_MUX_REG[35]);
	PIN_INPUT_ENABLE(GPIO_PIN_MUX_REG[36]);
	PIN_INPUT_ENABLE(GPIO_PIN_MUX_REG[39]);
}
/**************************************************************************
* Function: gpio_ll_deact_left_ip
* Preconditions:
* Overview: En esta funcion se desactivan todos los pines GPIO del lado izquierdo de la tarjeta ESP32 como entradas
* Input:
* Output:
*
*****************************************************************************/
static inline void gpio_ll_deact_left_ip(gpio_dev_t *hw){
	PIN_INPUT_DISABLE(GPIO_PIN_MUX_REG[12]);
	PIN_INPUT_DISABLE(GPIO_PIN_MUX_REG[13]);
	PIN_INPUT_DISABLE(GPIO_PIN_MUX_REG[14]);
	PIN_INPUT_DISABLE(GPIO_PIN_MUX_REG[25]);
	PIN_INPUT_DISABLE(GPIO_PIN_MUX_REG[26]);
	PIN_INPUT_DISABLE(GPIO_PIN_MUX_REG[27]);
	PIN_INPUT_DISABLE(GPIO_PIN_MUX_REG[32]);
	PIN_INPUT_DISABLE(GPIO_PIN_MUX_REG[33]);
	PIN_INPUT_DISABLE(GPIO_PIN_MUX_REG[34]);
	PIN_INPUT_DISABLE(GPIO_PIN_MUX_REG[35]);
	PIN_INPUT_DISABLE(GPIO_PIN_MUX_REG[36]);
	PIN_INPUT_DISABLE(GPIO_PIN_MUX_REG[39]);
}
/**************************************************************************
* Function: gpio_ll_act_left_pulldown
* Preconditions:
* Overview: En esta funcion se activa el pulldown para todos los pines GPIO del lado izquierdo de la tarjeta ESP32
* Input:
* Output:
*
*****************************************************************************/
static inline void gpio_ll_act_left_pulldown(gpio_dev_t *hw){
	REG_SET_BIT(GPIO_PIN_MUX_REG[12], FUN_PD);
	REG_SET_BIT(GPIO_PIN_MUX_REG[13], FUN_PD);
	REG_SET_BIT(GPIO_PIN_MUX_REG[14], FUN_PD);
	REG_SET_BIT(GPIO_PIN_MUX_REG[25], FUN_PD);
	REG_SET_BIT(GPIO_PIN_MUX_REG[26], FUN_PD);
	REG_SET_BIT(GPIO_PIN_MUX_REG[27], FUN_PD);
	REG_SET_BIT(GPIO_PIN_MUX_REG[32], FUN_PD);
	REG_SET_BIT(GPIO_PIN_MUX_REG[33], FUN_PD);
	REG_SET_BIT(GPIO_PIN_MUX_REG[34], FUN_PD);
	REG_SET_BIT(GPIO_PIN_MUX_REG[35], FUN_PD);
	REG_SET_BIT(GPIO_PIN_MUX_REG[36], FUN_PD);
	REG_SET_BIT(GPIO_PIN_MUX_REG[39], FUN_PD);
}
/**************************************************************************
* Function: gpio_ll_deact_left_pulldown
* Preconditions:
* Overview: En esta funcion se desctiva el pulldown de todos los pines GPIO del lado izquierdo de la tarjeta ESP32
* Input:
* Output:
*
*****************************************************************************/
static inline void gpio_ll_deact_left_pulldown(gpio_dev_t *hw){
	REG_CLR_BIT(GPIO_PIN_MUX_REG[12], FUN_PD);
	REG_CLR_BIT(GPIO_PIN_MUX_REG[13], FUN_PD);
	REG_CLR_BIT(GPIO_PIN_MUX_REG[14], FUN_PD);
	REG_CLR_BIT(GPIO_PIN_MUX_REG[25], FUN_PD);
	REG_CLR_BIT(GPIO_PIN_MUX_REG[26], FUN_PD);
	REG_CLR_BIT(GPIO_PIN_MUX_REG[27], FUN_PD);
	REG_CLR_BIT(GPIO_PIN_MUX_REG[32], FUN_PD);
	REG_CLR_BIT(GPIO_PIN_MUX_REG[33], FUN_PD);
	REG_CLR_BIT(GPIO_PIN_MUX_REG[34], FUN_PD);
	REG_CLR_BIT(GPIO_PIN_MUX_REG[35], FUN_PD);
	REG_CLR_BIT(GPIO_PIN_MUX_REG[36], FUN_PD);
	REG_CLR_BIT(GPIO_PIN_MUX_REG[39], FUN_PD);
}
/**************************************************************************
* Function: gpio_ll_act_left_pullup
* Preconditions:
* Overview: En esta funcion se activa el pullup para todos los pines GPIO del lado izquierdo de la tarjeta ESP32
* Input:
* Output:
*
*****************************************************************************/
static inline void gpio_ll_act_left_pullup(gpio_dev_t *hw){
	REG_SET_BIT(GPIO_PIN_MUX_REG[12], FUN_PU);
	REG_SET_BIT(GPIO_PIN_MUX_REG[13], FUN_PU);
	REG_SET_BIT(GPIO_PIN_MUX_REG[14], FUN_PU);
	REG_SET_BIT(GPIO_PIN_MUX_REG[25], FUN_PU);
	REG_SET_BIT(GPIO_PIN_MUX_REG[26], FUN_PU);
	REG_SET_BIT(GPIO_PIN_MUX_REG[27], FUN_PU);
	REG_SET_BIT(GPIO_PIN_MUX_REG[32], FUN_PU);
	REG_SET_BIT(GPIO_PIN_MUX_REG[33], FUN_PU);
	REG_SET_BIT(GPIO_PIN_MUX_REG[34], FUN_PU);
	REG_SET_BIT(GPIO_PIN_MUX_REG[35], FUN_PU);
	REG_SET_BIT(GPIO_PIN_MUX_REG[36], FUN_PU);
	REG_SET_BIT(GPIO_PIN_MUX_REG[39], FUN_PU);
}
/**************************************************************************
* Function: gpio_ll_deact_left_pullup
* Preconditions:
* Overview: En esta funcion se desactiva el pullup en todos los pines GPIO del lado izquierdo de la tarjeta ESP32
* Input:
* Output:
*
*****************************************************************************/
static inline void gpio_ll_deact_left_pullup(gpio_dev_t *hw){
	REG_CLR_BIT(GPIO_PIN_MUX_REG[12], FUN_PU);
	REG_CLR_BIT(GPIO_PIN_MUX_REG[13], FUN_PU);
	REG_CLR_BIT(GPIO_PIN_MUX_REG[14], FUN_PU);
	REG_CLR_BIT(GPIO_PIN_MUX_REG[25], FUN_PU);
	REG_CLR_BIT(GPIO_PIN_MUX_REG[26], FUN_PU);
	REG_CLR_BIT(GPIO_PIN_MUX_REG[27], FUN_PU);
	REG_CLR_BIT(GPIO_PIN_MUX_REG[32], FUN_PU);
	REG_CLR_BIT(GPIO_PIN_MUX_REG[33], FUN_PU);
	REG_CLR_BIT(GPIO_PIN_MUX_REG[34], FUN_PU);
	REG_CLR_BIT(GPIO_PIN_MUX_REG[35], FUN_PU);
	REG_CLR_BIT(GPIO_PIN_MUX_REG[36], FUN_PU);
	REG_CLR_BIT(GPIO_PIN_MUX_REG[39], FUN_PU);
}
/**************************************************************************
* Function: gpio_ll_act_right_ip
* Preconditions:
* Overview: En esta funcion se activan todos los pines GPIO del lado derecho de la tarjeta ESP32 como entradas
* Input:
* Output:
*
*****************************************************************************/
static inline void gpio_ll_act_right_ip(gpio_dev_t *hw){
	PIN_INPUT_ENABLE(GPIO_PIN_MUX_REG[1]);
	PIN_INPUT_ENABLE(GPIO_PIN_MUX_REG[2]);
	PIN_INPUT_ENABLE(GPIO_PIN_MUX_REG[3]);
	PIN_INPUT_ENABLE(GPIO_PIN_MUX_REG[4]);
	PIN_INPUT_ENABLE(GPIO_PIN_MUX_REG[5]);
	PIN_INPUT_ENABLE(GPIO_PIN_MUX_REG[15]);
	PIN_INPUT_ENABLE(GPIO_PIN_MUX_REG[16]);
	PIN_INPUT_ENABLE(GPIO_PIN_MUX_REG[17]);
	PIN_INPUT_ENABLE(GPIO_PIN_MUX_REG[18]);
	PIN_INPUT_ENABLE(GPIO_PIN_MUX_REG[19]);
	PIN_INPUT_ENABLE(GPIO_PIN_MUX_REG[21]);
	PIN_INPUT_ENABLE(GPIO_PIN_MUX_REG[22]);
	PIN_INPUT_ENABLE(GPIO_PIN_MUX_REG[23]);
}
/**************************************************************************
* Function: gpio_ll_deact_right_ip
* Preconditions:
* Overview: En esta funcion se desactivan todos los pines GPIO del lado derecho de la tarjeta ESP32 como entradas
* Input:
* Output:
*
*****************************************************************************/
static inline void gpio_ll_deact_right_ip(gpio_dev_t *hw){
	PIN_INPUT_DISABLE(GPIO_PIN_MUX_REG[1]);
	PIN_INPUT_DISABLE(GPIO_PIN_MUX_REG[2]);
	PIN_INPUT_DISABLE(GPIO_PIN_MUX_REG[3]);
	PIN_INPUT_DISABLE(GPIO_PIN_MUX_REG[4]);
	PIN_INPUT_DISABLE(GPIO_PIN_MUX_REG[5]);
	PIN_INPUT_DISABLE(GPIO_PIN_MUX_REG[15]);
	PIN_INPUT_DISABLE(GPIO_PIN_MUX_REG[16]);
	PIN_INPUT_DISABLE(GPIO_PIN_MUX_REG[17]);
	PIN_INPUT_DISABLE(GPIO_PIN_MUX_REG[18]);
	PIN_INPUT_DISABLE(GPIO_PIN_MUX_REG[19]);
	PIN_INPUT_DISABLE(GPIO_PIN_MUX_REG[21]);
	PIN_INPUT_DISABLE(GPIO_PIN_MUX_REG[22]);
	PIN_INPUT_DISABLE(GPIO_PIN_MUX_REG[23]);
}
/**************************************************************************
* Function: gpio_ll_act_right_pullup
* Preconditions:
* Overview: En esta funcion se activa el pullup para todos los pines GPIO del lado derecho de la tarjeta ESP32
* Input:
* Output:
*
*****************************************************************************/
static inline void gpio_ll_act_right_pullup(gpio_dev_t *hw){
	REG_SET_BIT(GPIO_PIN_MUX_REG[1], FUN_PU);
	REG_SET_BIT(GPIO_PIN_MUX_REG[2], FUN_PU);
	REG_SET_BIT(GPIO_PIN_MUX_REG[3], FUN_PU);
	REG_SET_BIT(GPIO_PIN_MUX_REG[4], FUN_PU);
	REG_SET_BIT(GPIO_PIN_MUX_REG[5], FUN_PU);
	REG_SET_BIT(GPIO_PIN_MUX_REG[15], FUN_PU);
	REG_SET_BIT(GPIO_PIN_MUX_REG[16], FUN_PU);
	REG_SET_BIT(GPIO_PIN_MUX_REG[17], FUN_PU);
	REG_SET_BIT(GPIO_PIN_MUX_REG[18], FUN_PU);
	REG_SET_BIT(GPIO_PIN_MUX_REG[19], FUN_PU);
	REG_SET_BIT(GPIO_PIN_MUX_REG[21], FUN_PU);
	REG_SET_BIT(GPIO_PIN_MUX_REG[22], FUN_PU);
	REG_SET_BIT(GPIO_PIN_MUX_REG[23], FUN_PU);
}
/**************************************************************************
* Function: gpio_ll_deact_right_pullup
* Preconditions:
* Overview: En esta funcion se desactiva el pullup para todos los pines GPIO del lado derecho de la tarjeta ESP32
* Input:
* Output:
*
*****************************************************************************/
static inline void gpio_ll_deact_right_pullup(gpio_dev_t *hw){
	REG_CLR_BIT(GPIO_PIN_MUX_REG[1], FUN_PU);
	REG_CLR_BIT(GPIO_PIN_MUX_REG[2], FUN_PU);
	REG_CLR_BIT(GPIO_PIN_MUX_REG[3], FUN_PU);
	REG_CLR_BIT(GPIO_PIN_MUX_REG[4], FUN_PU);
	REG_CLR_BIT(GPIO_PIN_MUX_REG[5], FUN_PU);
	REG_CLR_BIT(GPIO_PIN_MUX_REG[15], FUN_PU);
	REG_CLR_BIT(GPIO_PIN_MUX_REG[16], FUN_PU);
	REG_CLR_BIT(GPIO_PIN_MUX_REG[17], FUN_PU);
	REG_CLR_BIT(GPIO_PIN_MUX_REG[18], FUN_PU);
	REG_CLR_BIT(GPIO_PIN_MUX_REG[19], FUN_PU);
	REG_CLR_BIT(GPIO_PIN_MUX_REG[21], FUN_PU);
	REG_CLR_BIT(GPIO_PIN_MUX_REG[22], FUN_PU);
	REG_CLR_BIT(GPIO_PIN_MUX_REG[23], FUN_PU);
}
/**************************************************************************
* Function: gpio_ll_act_right_pulldown
* Preconditions:
* Overview: En esta funcion se activa el pulldown para todos los pines GPIO del lado derecho de la tarjeta ESP32
* Input:
* Output:
*
*****************************************************************************/
static inline void gpio_ll_act_right_pulldown(gpio_dev_t *hw){
	REG_SET_BIT(GPIO_PIN_MUX_REG[1], FUN_PD);
	REG_SET_BIT(GPIO_PIN_MUX_REG[2], FUN_PD);
	REG_SET_BIT(GPIO_PIN_MUX_REG[3], FUN_PD);
	REG_SET_BIT(GPIO_PIN_MUX_REG[4], FUN_PD);
	REG_SET_BIT(GPIO_PIN_MUX_REG[5], FUN_PD);
	REG_SET_BIT(GPIO_PIN_MUX_REG[15], FUN_PD);
	REG_SET_BIT(GPIO_PIN_MUX_REG[16], FUN_PD);
	REG_SET_BIT(GPIO_PIN_MUX_REG[17], FUN_PD);
	REG_SET_BIT(GPIO_PIN_MUX_REG[18], FUN_PD);
	REG_SET_BIT(GPIO_PIN_MUX_REG[19], FUN_PD);
	REG_SET_BIT(GPIO_PIN_MUX_REG[21], FUN_PD);
	REG_SET_BIT(GPIO_PIN_MUX_REG[22], FUN_PD);
	REG_SET_BIT(GPIO_PIN_MUX_REG[23], FUN_PD);
}
/**************************************************************************
* Function: gpio_ll_deact_right_pulldown
* Preconditions: ¿
* Overview: En esta funcion se desctiva el pulldown para todos los pines GPIO del lado derecho de la tarjeta ESP32
* Input:
* Output:
*
*****************************************************************************/
static inline void gpio_ll_deact_right_pulldown(gpio_dev_t *hw){
	REG_CLR_BIT(GPIO_PIN_MUX_REG[1], FUN_PD);
	REG_CLR_BIT(GPIO_PIN_MUX_REG[2], FUN_PD);
	REG_CLR_BIT(GPIO_PIN_MUX_REG[3], FUN_PD);
	REG_CLR_BIT(GPIO_PIN_MUX_REG[4], FUN_PD);
	REG_CLR_BIT(GPIO_PIN_MUX_REG[5], FUN_PD);
	REG_CLR_BIT(GPIO_PIN_MUX_REG[15], FUN_PD);
	REG_CLR_BIT(GPIO_PIN_MUX_REG[16], FUN_PD);
	REG_CLR_BIT(GPIO_PIN_MUX_REG[17], FUN_PD);
	REG_CLR_BIT(GPIO_PIN_MUX_REG[18], FUN_PD);
	REG_CLR_BIT(GPIO_PIN_MUX_REG[19], FUN_PD);
	REG_CLR_BIT(GPIO_PIN_MUX_REG[21], FUN_PD);
	REG_CLR_BIT(GPIO_PIN_MUX_REG[22], FUN_PD);
	REG_CLR_BIT(GPIO_PIN_MUX_REG[23], FUN_PD);
}
/**************************************************************************
* Function: gpio_ll_pullup_act
* Preconditions:
* Overview: En esta funcion se activa el pullup para un pin GPIO especifico de la tarjeta
* Input: Recibe el numero del pin al que se le quiere activar el pullup
* Output:
*
*****************************************************************************/
static inline void gpio_ll_pullup_act(gpio_dev_t *hw, gpio_num_t gpio_num)
{
    REG_SET_BIT(GPIO_PIN_MUX_REG[gpio_num], FUN_PU);
}
/**************************************************************************
* Function: gpio_ll_pullup_deact
* Preconditions:
* Overview: En esta funcion se desactiva el pullup para un pin GPIO especifico de la tarjeta
* Input: Recibe el numero del pin al que se le quiere desactivar el pullup
* Output:
*
*****************************************************************************/
static inline void gpio_ll_pullup_deact(gpio_dev_t *hw, gpio_num_t gpio_num)
{
    REG_CLR_BIT(GPIO_PIN_MUX_REG[gpio_num], FUN_PU);
}
/**************************************************************************
* Function: gpio_ll_pullup_is_enabled
* Preconditions:
* Overview: Esta funcion retorna si es que el pullup esta activo en uno de los pines GPIO
* Input: Recibe el numero de pin del que se decea saber si tiene el pullup activo
* Output: Entrega un true o false segun sea el caso
*
*****************************************************************************/
static inline bool gpio_ll_pullup_is_enabled(gpio_dev_t *hw, gpio_num_t gpio_num)
{
    return REG_GET_BIT(GPIO_PIN_MUX_REG[gpio_num], FUN_PU) ? true : false;
}
/**************************************************************************
* Function: gpio_ll_pulldown_act
* Preconditions:
* Overview: En esta funcion se activa el pulldown para un pin GPIO especifico de la tarjeta
* Input: Recibe el numero del pin al que se desea activar el pulldown
* Output:
*
*****************************************************************************/
static inline void gpio_ll_pulldown_act(gpio_dev_t *hw, gpio_num_t gpio_num)
{
    REG_SET_BIT(GPIO_PIN_MUX_REG[gpio_num], FUN_PD);
}
/**************************************************************************
* Function: gpio_ll_pulldown_deact
* Preconditions:
* Overview: En esta funcion se desactiva el pulldown para un pin GPIO especifico de la tarjeta
* Input: Recibe el numero de pin al que se quiere desactivar el pulldown
* Output:
*
*****************************************************************************/
static inline void gpio_ll_pulldown_deact(gpio_dev_t *hw, gpio_num_t gpio_num)
{
    REG_CLR_BIT(GPIO_PIN_MUX_REG[gpio_num], FUN_PD);
}
/**************************************************************************
* Function: gpio_ll_pulldown_is_enabled
* Preconditions:
* Overview: En esta funcion se retorna si es que el pulldown esta activo en el pin deceado
* Input: Recibe el numero de pin del cual se decea saber si tiene activo el pulldown
* Output: Retorna true o false segun sea el caso
*
*****************************************************************************/
static inline bool gpio_ll_pulldown_is_enabled(gpio_dev_t *hw, gpio_num_t gpio_num)
{
    return REG_GET_BIT(GPIO_PIN_MUX_REG[gpio_num], FUN_PD) ? true : false;
}
/**************************************************************************
* Function: gpio_ll_sleep_sel_en
* Preconditions:
* Overview: En esta funcion se establece un pin GPIO para usarse como wakeup para la tarjeta
* Input: Recibe el numero de pin GPIO el cual se decea usar para el wakeup de la tarjeta
* Output:
*
*****************************************************************************/
static inline void gpio_ll_sleep_sel_en(gpio_dev_t *hw, gpio_num_t gpio_num)
{
    PIN_SLP_SEL_ENABLE(GPIO_PIN_MUX_REG[gpio_num]);
}
/**************************************************************************
* Function: gpio_ll_sleep_sel_dis
* Preconditions:
* Overview: En esta funcion se desactiva el que un pin GPIO sea usado para el wakeup de la tarjeta
* Input: Recibe el numero de pin al cual se decea desactivar el wakeup para la terjeta
* Output:
*
*****************************************************************************/
static inline void gpio_ll_sleep_sel_dis(gpio_dev_t *hw, gpio_num_t gpio_num)
{
    PIN_SLP_SEL_DISABLE(GPIO_PIN_MUX_REG[gpio_num]);
}
/**************************************************************************
* Function: gpio_ll_sleep_sel_is_enabled
* Preconditions:
* Overview: En esta funcion se restorna si es que un pin esta siendo usado como wakeup
* Input: Recibe el numero de pin del cual se decea verificar si esta siendo usado para el wakeup
* Output: Entrega true o false segun sea el caso
*
*****************************************************************************/
static inline bool gpio_ll_sleep_sel_is_enabled(gpio_dev_t *hw, gpio_num_t gpio_num)
{
    return REG_GET_BIT(GPIO_PIN_MUX_REG[gpio_num], SLP_SEL) ? true : false;
}
/**************************************************************************
* Function: gpio_ll_sleep_pullup_dis
* Preconditions:
* Overview: En esta funcion se desactiva el pullup para un pin GPIO cuando esta en modo sleep
* Input: Recibe el numero de pin al cual se decea desactivar el pulldown
* Output:
*
*****************************************************************************/
static inline void gpio_ll_sleep_pullup_dis(gpio_dev_t *hw, gpio_num_t gpio_num)
{
    PIN_SLP_PULLUP_DISABLE(GPIO_PIN_MUX_REG[gpio_num]);
}
/**************************************************************************
* Function: gpio_ll_sleep_pullup_en
* Preconditions:
* Overview: En esta funcion se activa el pullup para un pin GPIO cuando se esta en el modo sleep
* Input: Recibe el numero de pin al cual se le decea activar el pullup
* Output:
*
*****************************************************************************/
static inline void gpio_ll_sleep_pullup_en(gpio_dev_t *hw, gpio_num_t gpio_num)
{
    PIN_SLP_PULLUP_ENABLE(GPIO_PIN_MUX_REG[gpio_num]);
}
/**************************************************************************
* Function: gpio_ll_sleep_pullup_is_enabled
* Preconditions:
* Overview: En esta funcion se retorna si es que el pullup esta activo en el pin GPIO entregado cuando se esta en el modo sleep
* Input: Recibe el numero de pin al cual se decea verificar
* Output: Entrega true o false segun sea el caso
*
*****************************************************************************/
/**
  * @brief Return slp-pull-up status on GPIO.
  *
  * @param hw Peripheral GPIO hardware instance address.
  * @param gpio_num GPIO number
  * @return if GPIO gpio_num`s SLP_PU is true
  */
static inline bool gpio_ll_sleep_pullup_is_enabled(gpio_dev_t *hw, gpio_num_t gpio_num)
{
    return REG_GET_BIT(GPIO_PIN_MUX_REG[gpio_num], SLP_PU) ? true : false;
}
/**************************************************************************
* Function: gpio_ll_sleep_pulldown_en
* Preconditions:
* Overview: En esta funcion se activa el pulldown para un pin GPIO cuando se esta en modo sleep
* Input: Recibe el numero de pin al cual se decea activar el pulldown
* Output:
*
*****************************************************************************/
static inline void gpio_ll_sleep_pulldown_en(gpio_dev_t *hw, gpio_num_t gpio_num)
{
    PIN_SLP_PULLDOWN_ENABLE(GPIO_PIN_MUX_REG[gpio_num]);
}
/**************************************************************************
* Function: gpio_ll_sleep_pulldown_dis
* Preconditions:
* Overview: En esta funcion se desactiva el pulldown para un pin GPIO cuando se esta en modo sleep
* Input: Recibe el numero de pin al cual se le decea desactivar e pulldown
* Output:
*
*****************************************************************************/
static inline void gpio_ll_sleep_pulldown_dis(gpio_dev_t *hw, gpio_num_t gpio_num)
{
    PIN_SLP_PULLDOWN_DISABLE(GPIO_PIN_MUX_REG[gpio_num]);
}
/**************************************************************************
* Function: gpio_ll_sleep_pulldown_is_enabled
* Preconditions:
* Overview: Esta funcion retorna si es que un pin GPIO tiene activo el pulldown al estar en modo sleep
* Input: Recibe el numero de pin al que se hara la verificacion
* Output: Entrega true o false segun sea el caso
*
*****************************************************************************/

static inline bool gpio_ll_sleep_pulldown_is_enabled(gpio_dev_t *hw, gpio_num_t gpio_num)
{
    return REG_GET_BIT(GPIO_PIN_MUX_REG[gpio_num], SLP_PD) ? true : false;
}
/**************************************************************************
* Function: gpio_ll_set_intr_type
* Preconditions:
* Overview: Esta funcion sirve para establecer que tipo de interrupcion tendra un pin GPIO
* Input: Recibe el numero de pin al cual se le queire modificar y el tipo de interrupcion que tendra el mismo,
* 		este se puede escojer dentre los establecidos en la enumeracion gpio_int_type_t
* Output:
*
*****************************************************************************/

static inline void gpio_ll_set_intr_type(gpio_dev_t *hw, gpio_num_t gpio_num, gpio_int_type_t intr_type)
{
    hw->pin[gpio_num].int_type = intr_type;
}
/**************************************************************************
* Function: gpio_ll_get_intr_status
* Preconditions:
* Overview: Esta funcion se actualiza el estado de la interrupcion
* Input: Recibe el core id y la direccion en donde se guarda el estado de la interrupcion
* Output:
*
*****************************************************************************/
static inline void gpio_ll_get_intr_status(gpio_dev_t *hw, uint32_t core_id, uint32_t *status)
{
    *status = (core_id == 0) ? hw->pcpu_int : hw->acpu_int;
}
/**************************************************************************
* Function: gpio_ll_get_intr_status_high
* Preconditions:
* Overview: En esta funcion se actualiza el estado de interrupcion de alto nivel
* Input: Recibe el core id y la direccion donde se guarda el estado de la interrupcion
* Output:
*
*****************************************************************************/
static inline void gpio_ll_get_intr_status_high(gpio_dev_t *hw, uint32_t core_id, uint32_t *status)
{
    *status = (core_id == 0) ? HAL_FORCE_READ_U32_REG_FIELD(hw->pcpu_int1, intr) : HAL_FORCE_READ_U32_REG_FIELD(hw->acpu_int1, intr);
}
/**************************************************************************
* Function: gpio_ll_clear_intr_status
* Preconditions:
* Overview: Esta funcion sirve para limpiar el estado de la interrupcion
* Input: Recibe el estado previo de la interrupcion
* Output:
*
*****************************************************************************/
static inline void gpio_ll_clear_intr_status(gpio_dev_t *hw, uint32_t mask)
{
    hw->status_w1tc = mask;
}
/**************************************************************************
* Function: gpio_ll_clear_intr_status_high
* Preconditions:
* Overview: Esta funcion sirve para limpiar el estado de la interrupcion de alto nivel
* Input: Recibe el estado previo de la interrupcion
* Output:
*
*****************************************************************************/
static inline void gpio_ll_clear_intr_status_high(gpio_dev_t *hw, uint32_t mask)
{
    HAL_FORCE_MODIFY_U32_REG_FIELD(hw->status1_w1tc, intr_st, mask);
}
/**************************************************************************
* Function: gpio_ll_intr_enable_on_core
* Preconditions:
* Overview: Esta funcion sirve para establecer en que bit del pin se establece la interrupcion
* Input: Recibe el core id y el numero de pin en donde se va a habilitar la interrupcion
* Output:
*
*****************************************************************************/
static inline void gpio_ll_intr_enable_on_core(gpio_dev_t *hw, uint32_t core_id, gpio_num_t gpio_num)
{
    if (core_id == 0) {
        hw->pin[gpio_num].int_ena = GPIO_LL_PRO_CPU_INTR_ENA;     //enable pro cpu intr
    } else {
        hw->pin[gpio_num].int_ena = GPIO_LL_APP_CPU_INTR_ENA;     //enable pro cpu intr
    }
}
/**************************************************************************
* Function: gpio_ll_intr_disable
* Preconditions:
* Overview: Esta funcion sirve para desactivar la interrupcion para un pin GPIO a elegir
* Input: Recibe el numero de pin al cual se le desactivara la interrupcion
* Output:
*
*****************************************************************************/
static inline void gpio_ll_intr_disable(gpio_dev_t *hw, gpio_num_t gpio_num)
{
    hw->pin[gpio_num].int_ena = 0;                             //disable GPIO intr
}
/**************************************************************************
* Function: gpio_ll_input_disable
* Preconditions:
* Overview: Esta funcion sirve para desactivar el que un pin GPIO funcione como entrada
* Input: Recibe el numero del pin a desactivar como entrada
* Output:
*
*****************************************************************************/
static inline void gpio_ll_input_disable(gpio_dev_t *hw, gpio_num_t gpio_num)
{
    PIN_INPUT_DISABLE(GPIO_PIN_MUX_REG[gpio_num]);
}
/**************************************************************************
* Function: gpio_ll_input_enable
* Preconditions:
* Overview: Esta funcion sirve para establecer que un pin GPIO funcionara como entrada
* Input: Recibe el numero de pin que sera usado como entrada
* Output:
*
*****************************************************************************/
static inline void gpio_ll_input_enable(gpio_dev_t *hw, gpio_num_t gpio_num)
{
    PIN_INPUT_ENABLE(GPIO_PIN_MUX_REG[gpio_num]);
}
/**************************************************************************
* Function: gpio_ll_output_disable
* Preconditions:
* Overview: Esta funcion sirve para desactivar el que un pin GPIO sea utilizado como salida
* Input: Recibe el numero de pin que dejara de ser utilizado como salida
* Output:
*
*****************************************************************************/
static inline void gpio_ll_output_disable(gpio_dev_t *hw, gpio_num_t gpio_num)
{
    if (gpio_num < 32) {
        hw->enable_w1tc = (0x1 << gpio_num);
    } else {
        HAL_FORCE_MODIFY_U32_REG_FIELD(hw->enable1_w1tc, data, (0x1 << (gpio_num - 32)));
    }

    // Ensure no other output signal is routed via GPIO matrix to this pin
    REG_WRITE(GPIO_FUNC0_OUT_SEL_CFG_REG + (gpio_num * 4),
              SIG_GPIO_OUT_IDX);
}
/**************************************************************************
* Function: gpio_ll_output_enable
* Preconditions:
* Overview: Esta funcion sirve para establecer que un pin GPIO sera utilizado como salida
* Input: Recibe el numero de pin que sera utilizado como salida
* Output:
*
*****************************************************************************/
static inline void gpio_ll_output_enable(gpio_dev_t *hw, gpio_num_t gpio_num)
{
    if (gpio_num < 32) {
        hw->enable_w1ts = (0x1 << gpio_num);
    } else {
        HAL_FORCE_MODIFY_U32_REG_FIELD(hw->enable1_w1ts, data, (0x1 << (gpio_num - 32)));
    }
}
/**************************************************************************
* Function: gpio_ll_sleep_input_disable
* Preconditions:
* Overview: Esta funcion sirve para desactivar el que un pin funcione como entrada al estar en modo sleep
* Input: Recibe en numero de pin que dejara de funcionar como entrada
* Output:
*
*****************************************************************************/
static inline void gpio_ll_sleep_input_disable(gpio_dev_t *hw, gpio_num_t gpio_num)
{
    PIN_SLP_INPUT_DISABLE(GPIO_PIN_MUX_REG[gpio_num]);
}
/**************************************************************************
* Function: gpio_ll_sleep_input_enable
* Preconditions:
* Overview: Esta funcion sirve para establecer que un pin sera utilizado como entrada en el modo sleep
* Input: Recibe el numero de pin que sera utilizado como entrada
* Output:
*
*****************************************************************************/
static inline void gpio_ll_sleep_input_enable(gpio_dev_t *hw, gpio_num_t gpio_num)
{
    PIN_SLP_INPUT_ENABLE(GPIO_PIN_MUX_REG[gpio_num]);
}
/**************************************************************************
* Function: gpio_ll_sleep_output_disable
* Preconditions:
* Overview: Esta funcion sirve para desactivar el que un pin GPIO sea utilizado como salida en el modo sleep
* Input: Recibe el numero de pin que dejara de ser utilizado como salida
* Output:
*
*****************************************************************************/
static inline void gpio_ll_sleep_output_disable(gpio_dev_t *hw, gpio_num_t gpio_num)
{
    PIN_SLP_OUTPUT_DISABLE(GPIO_PIN_MUX_REG[gpio_num]);
}
/**************************************************************************
* Function: gpio_ll_sleep_output_enable
* Preconditions:
* Overview: Esta funcion sirve para establecer que un pin GPIO sera utilizado como salida en el modo sleep
* Input: Recibe el numero de pin que sera utilizado como salida
* Output:
*
*****************************************************************************/
static inline void gpio_ll_sleep_output_enable(gpio_dev_t *hw, gpio_num_t gpio_num)
{
    PIN_SLP_OUTPUT_ENABLE(GPIO_PIN_MUX_REG[gpio_num]);
}
/**************************************************************************
* Function: gpio_ll_od_disable
* Preconditions:
* Overview: Esta funcion sirve para desactivar el que un pin GPIO funcione como Open-Drain
* Input: Recibe el nuemro del pin que dejara de ser usado como od
* Output:
*
*****************************************************************************/
static inline void gpio_ll_od_disable(gpio_dev_t *hw, gpio_num_t gpio_num)
{
    hw->pin[gpio_num].pad_driver = 0;
}
/**************************************************************************
* Function: gpio_ll_od_enable
* Preconditions:
* Overview: Esta funcion sirve para activar que un pin funcione como Open_Drain
* Input: Recibe el numero de pin que funcionara como od
* Output:
*
*****************************************************************************/
static inline void gpio_ll_od_enable(gpio_dev_t *hw, gpio_num_t gpio_num)
{
    hw->pin[gpio_num].pad_driver = 1;
}
/**************************************************************************
* Function: gpio_ll_set_level
* Preconditions:
* Overview: Esta funcion sirve para establecer que tendra el pin elegido en la salida
* Input: Recibe el numero de pin a comfigurar y el valor que tendra en la salida, ya sea 0 o 1
* Output:
*
*****************************************************************************/
static inline void gpio_ll_set_level(gpio_dev_t *hw, gpio_num_t gpio_num, uint32_t level)
{
    if (level) {
        if (gpio_num < 32) {
            hw->out_w1ts = (1 << gpio_num);
        } else {
            HAL_FORCE_MODIFY_U32_REG_FIELD(hw->out1_w1ts, data, (1 << (gpio_num - 32)));
        }
    } else {
        if (gpio_num < 32) {
            hw->out_w1tc = (1 << gpio_num);
        } else {
            HAL_FORCE_MODIFY_U32_REG_FIELD(hw->out1_w1tc, data, (1 << (gpio_num - 32)));
        }
    }
}
/**************************************************************************
* Function: gpio_ll_get_level
* Preconditions:
* Overview: Esta funcion sirve para leer el valor que tiene un pin establecido como entrada
* Input:  Recibe el numero del pin de entrada a leer
* Output:
*
*****************************************************************************/
static inline int gpio_ll_get_level(gpio_dev_t *hw, gpio_num_t gpio_num)
{
    if (gpio_num < 32) {
        return (hw->in >> gpio_num) & 0x1;
    } else {
        return (HAL_FORCE_READ_U32_REG_FIELD(hw->in1, data) >> (gpio_num - 32)) & 0x1;
    }
}
/**************************************************************************
* Function: gpio_ll_wakeup_enable
* Preconditions:
* Overview: Esta funcion sirve para activar el wakeup en un pin a elegir
* Input: Recibe el numero de pin y el tipo de interrupcion con que funcionara el wakeup
* Output:
*
*****************************************************************************/
static inline void gpio_ll_wakeup_enable(gpio_dev_t *hw, gpio_num_t gpio_num, gpio_int_type_t intr_type)
{
    hw->pin[gpio_num].int_type = intr_type;
    hw->pin[gpio_num].wakeup_enable = 0x1;
}
/**************************************************************************
* Function: gpio_ll_wakeup_disable
* Preconditions:
* Overview: Esta funcion sirve para desactivar el que un pin funcione como wakeup
* Input: Recibe el numero de pin a desactivar
* Output:
*
*****************************************************************************/
static inline void gpio_ll_wakeup_disable(gpio_dev_t *hw, gpio_num_t gpio_num)
{
    hw->pin[gpio_num].wakeup_enable = 0;
}
/**************************************************************************
* Function: gpio_ll_set_drive_capability
* Preconditions:
* Overview: Esta funcion sirve para establecer la fuerza del puerto
* Input: Recibe el numero de pin a configurar y la fuerza que se le asignara al mismo
* Output:
*
*****************************************************************************/
static inline void gpio_ll_set_drive_capability(gpio_dev_t *hw, gpio_num_t gpio_num, gpio_drive_cap_t strength)
{
    SET_PERI_REG_BITS(GPIO_PIN_MUX_REG[gpio_num], FUN_DRV_V, strength, FUN_DRV_S);
}
/**************************************************************************
* Function: gpio_ll_get_drive_capability
* Preconditions:
* Overview: Esta funcion sirve para obtener el nivel de fuerza de un pin
* Input: Recibe el numero de pin a vigilar y la direccion donde se guardara la fuerza de este
* Output:
*
*****************************************************************************/
static inline void gpio_ll_get_drive_capability(gpio_dev_t *hw, gpio_num_t gpio_num, gpio_drive_cap_t *strength)
{
    *strength = (gpio_drive_cap_t)GET_PERI_REG_BITS2(GPIO_PIN_MUX_REG[gpio_num], FUN_DRV_V, FUN_DRV_S);
}
/**************************************************************************
* Function: gpio_ll_deep_sleep_hold_en
* Preconditions:
* Overview: Esta funcion sirve para conservar la configuracion de todos los pines GPIO al estan en modo deep sleep
* Input:
* Output:
*
*****************************************************************************/
static inline void gpio_ll_deep_sleep_hold_en(gpio_dev_t *hw)
{
    SET_PERI_REG_MASK(RTC_CNTL_DIG_ISO_REG, RTC_CNTL_DG_PAD_AUTOHOLD_EN_M);
}
/**************************************************************************
* Function: gpio_ll_deep_sleep_hold_dis
* Preconditions:
* Overview: Esta funcion sirve para desactivar el que todos los pines converven su configuracion en el modo deep sleep
* Input:
* Output:
*
*****************************************************************************/
static inline void gpio_ll_deep_sleep_hold_dis(gpio_dev_t *hw)
{
    CLEAR_PERI_REG_MASK(RTC_CNTL_DIG_ISO_REG, RTC_CNTL_DG_PAD_AUTOHOLD_EN_M);
}
/**************************************************************************
* Function: gpio_ll_hold_en
* Preconditions:
* Overview: Esta funcion sirve para habilitar el que un pin funcione como hold
* Input: Recibe el numero de pin a configurar
* Output:
*
*****************************************************************************/
static inline void gpio_ll_hold_en(gpio_dev_t *hw, gpio_num_t gpio_num)
{
    SET_PERI_REG_MASK(RTC_IO_DIG_PAD_HOLD_REG, GPIO_HOLD_MASK[gpio_num]);
}
/**************************************************************************
* Function: gpio_ll_hold_dis
* Preconditions:
* Overview: Esta funcion sirve para desactivar el que un pin funcione como hold
* Input: Recibe el numero de pin a desactivar
* Output:
*
*****************************************************************************/
static inline void gpio_ll_hold_dis(gpio_dev_t *hw, gpio_num_t gpio_num)
{
    CLEAR_PERI_REG_MASK(RTC_IO_DIG_PAD_HOLD_REG, GPIO_HOLD_MASK[gpio_num]);
}
/**************************************************************************
* Function: gpio_ll_iomux_in
* Preconditions:
* Overview: Esta funcion sirve para establecer la comunicacion con un modulo externo a traves del IOMUX
* Input: Recibe el numero del pin a utilizar y el id del periferico
* Output:
*
*****************************************************************************/
static inline void gpio_ll_iomux_in(gpio_dev_t *hw, uint32_t gpio, uint32_t signal_idx)
{
    hw->func_in_sel_cfg[signal_idx].sig_in_sel = 0;
    PIN_INPUT_ENABLE(GPIO_PIN_MUX_REG[gpio]);
}
/**************************************************************************
* Function: gpio_ll_iomux_func_sel
* Preconditions:
* Overview: Esta funcion sirve para establecer que funcion trabajara en el IOMUX
* Input: Recibe numero del pin a configurar y la funcion a asignar
* Output:
*
*****************************************************************************/
static inline __attribute__((always_inline)) void gpio_ll_iomux_func_sel(uint32_t pin_name, uint32_t func)
{
    PIN_FUNC_SELECT(pin_name, func);
}
/**************************************************************************
* Function: gpio_ll_iomux_out
* Preconditions:
* Overview: Esta funcion sirve para establecer un pin como salida IOMUX, ademas de asignarle una funcion e invertir el resultado si fuese necesario
* Input: Recibe el numero de pin a configurar, la funcion, y si es que sera inverida la salida
* Output:
*
*****************************************************************************/
static inline void gpio_ll_iomux_out(gpio_dev_t *hw, uint8_t gpio_num, int func, uint32_t oen_inv)
{
    hw->func_out_sel_cfg[gpio_num].oen_sel = 0;
    hw->func_out_sel_cfg[gpio_num].oen_inv_sel = oen_inv;
    gpio_ll_iomux_func_sel(GPIO_PIN_MUX_REG[gpio_num], func);
}

#ifdef __cplusplus
}
#endif
