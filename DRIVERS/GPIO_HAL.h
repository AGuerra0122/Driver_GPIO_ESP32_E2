 // FileName:        GPIO_HAL.h
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
// Copyright 2015-2019 Espressif Systems (Shanghai) PTE LTD
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

/*******************************************************************************
 * NOTICE
 * The hal is not public api, don't use in application code.
 * See readme.md in hal/include/hal/readme.md
 ******************************************************************************/

// The HAL layer for GPIO

#pragma once

#include "../DRIVERS/GPIO_LL.h"
#include "soc/gpio_periph.h"
#include "soc/soc_caps.h"
#include "hal/gpio_types.h"

#ifdef CONFIG_LEGACY_INCLUDE_COMMON_HEADERS
#include "soc/rtc_io_reg.h"
#endif

#ifdef __cplusplus
extern "C" {
#endif
/**************************************************************************
* Function: Nombre de la función
* Preconditions: Qué funciones o declaraciones son previas al programa
* Overview: resumen del programa.
* Input: argumentos de entrada.
* Output: argumentos de salida
*
*****************************************************************************/
// Get GPIO hardware instance with giving gpio num
#define GPIO_HAL_GET_HW(num) GPIO_LL_GET_HW(num)

/**
 * Context that should be maintained by both the driver and the HAL
 */

typedef struct {
    gpio_dev_t *dev;
    uint32_t version;
} gpio_hal_context_t;
/**************************************************************************
* Function: gpio_hal_act_left_op
* Preconditions: gpio_ll_act_left_op
* Overview: Redefinicion de la activacion de salidas izquierdas.
*
*****************************************************************************/
#define gpio_hal_act_left_op(hal) gpio_ll_act_left_op((hal)->dev)
/**************************************************************************
* Function: gpio_hal_deact_left_op
* Preconditions: gpio_ll_deact_left_op
* Overview: Redefinicion de la desactivacion de salidas izquierdas.
*
*****************************************************************************/
#define gpio_hal_deact_left_op(hal) gpio_ll_deact_left_op((hal)->dev)
/**************************************************************************
* Function: gpio_hal_act_right_op
* Preconditions: gpio_ll_act_right_op
* Overview: Redefinicion de la activacion de salidas derechas.
*
*****************************************************************************/
#define gpio_hal_act_right_op(hal) gpio_ll_act_right_op((hal)->dev)
/**************************************************************************
* Function: gpio_hal_deact_right_op
* Preconditions: gpio_ll_deact_right_op
* Overview: Redefinicion de la desactivacion de salidas derechas.
*
*****************************************************************************/
#define gpio_hal_deact_right_op(hal) gpio_ll_deact_right_op((hal)->dev)
/**************************************************************************
* Function: gpio_hal_act_left_ip
* Preconditions: gpio_ll_act_left_ip
* Overview: Redefinicion de la activacion de entradas izquierdas.
*
*****************************************************************************/
#define gpio_hal_act_left_ip(hal) gpio_ll_act_left_ip((hal)->dev)
/**************************************************************************
* Function: gpio_hal_deact_left_ip
* Preconditions: gpio_ll_deact_left_ip
* Overview: Redefinicion de la desactivacion de entradas izquierdas.
*
*****************************************************************************/
#define gpio_hal_deact_left_ip(hal) gpio_ll_deact_left_ip((hal)->dev)
/**************************************************************************
* Function: gpio_hal_act_left__pulldown
* Preconditions: gpio_ll_act_left_pulldown
* Overview: Redefinicion de la activacion de resistencias de pulldown izquierdas.
*
*****************************************************************************/
#define gpio_hal_act_left_pulldown(hal) gpio_ll_act_left_pulldown((hal)->dev)
/**************************************************************************
* Function: gpio_hal_deact_left_pulldown
* Preconditions: gpio_ll_act_left_pulldown
* Overview: Redefinicion de la desactivacion de resistencias de pulldown izquierdas.
*
*****************************************************************************/
#define gpio_hal_deact_left_pulldown(hal) gpio_ll_deact_left_pulldown((hal)->dev)
/**************************************************************************
* Function: gpio_hal_act_left_pullup
* Preconditions: gpio_ll_act_left_pullup
* Overview: Redefinicion de la activacion de resistencias de pullup izquierdas.
*
*****************************************************************************/
#define gpio_hal_act_left_pullup(hal) gpio_ll_act_left_pullup((hal)->dev)
/**************************************************************************
* Function: gpio_hal_deact_left_pullup
* Preconditions: gpio_ll_deact_left_pullup
* Overview: Redefinicion de la desactivacion de resistencias de pullup izquierdas.
*
*****************************************************************************/
#define gpio_hal_deact_left_pullup(hal) gpio_ll_deact_left_pullup((hal)->dev)

/**************************************************************************
* Function: gpio_hal_act_right_ip
* Preconditions: gpio_ll_act_right_ip
* Overview: Redefinicion de la activacion de entradas derechas.
*
*****************************************************************************/
#define gpio_hal_act_right_ip(hal) gpio_ll_act_right_ip((hal)->dev)
/**************************************************************************
* Function: gpio_hal_deact_right_ip
* Preconditions: gpio_ll_deact_right_ip
* Overview: Redefinicion de la desactivacion de entradas derechas.
*
*****************************************************************************/
#define gpio_hal_deact_right_ip(hal) gpio_ll_deact_right_ip((hal)->dev)
/**************************************************************************
* Function: gpio_hal_act_right__pulldown
* Preconditions: gpio_ll_act_right_pulldown
* Overview: Redefinicion de la activacion de resistencias de pulldown derechas.
*
*****************************************************************************/
#define gpio_hal_act_right_pulldown(hal) gpio_ll_act_right_pulldown((hal)->dev)
/**************************************************************************
* Function: gpio_hal_deact_right_pulldown
* Preconditions: gpio_ll_act_right_pulldown
* Overview: Redefinicion de la desactivacion de resistencias de pulldown derechas.
*
*****************************************************************************/
#define gpio_hal_deact_right_pulldown(hal) gpio_ll_deact_right_pulldown((hal)->dev)
/**************************************************************************
* Function: gpio_hal_act_right_pullup
* Preconditions: gpio_ll_act_right_pullup
* Overview: Redefinicion de la activacion de resistencias de pullup derechas.
*
*****************************************************************************/
#define gpio_hal_act_right_pullup(hal) gpio_ll_act_right_pullup((hal)->dev)
/**************************************************************************
* Function: gpio_hal_deact_right_pullup
* Preconditions: gpio_ll_deact_right_pullup
* Overview: Redefinicion de la desactivacion de resistencias de pullup derechas.
*
*****************************************************************************/
#define gpio_hal_deact_right_pullup(hal) gpio_ll_deact_right_pullup((hal)->dev)
/**************************************************************************
* Function:gpio_hal_pullup_act
* Preconditions: gpio_ll_pullup_act
* Overview: Redefinicion de habilitacion de la resistencia de pull-up en el GPIO.
* Input: hal: Contexto de la capa HAL.
* 		 gpio_num: numero de GPIO.
*
*****************************************************************************/
#define gpio_hal_pullup_act(hal, gpio_num) gpio_ll_pullup_act((hal)->dev, gpio_num)
/**************************************************************************
* Function:gpio_hal_pullup_deact
* Preconditions: gpio_ll_pullup_deact
* Overview: Redefinicion de deshabilitacion de la resistencia de pull-up en el GPIO.
* Input: hal:Contexto de la capa HAL.
* 		 gpio_num: numero de GPIO.
*
*****************************************************************************/
#define gpio_hal_pullup_deact(hal, gpio_num) gpio_ll_pullup_deact((hal)->dev, gpio_num)
/**************************************************************************
* Function:gpio_hal_pulldown_act
* Preconditions: gpio_ll_pulldown_act
* Overview: Redefinicion de habilitacion de la resistencia de pull-down en el GPIO.
* Input: hal:Contexto de la capa HAL.
* 		 gpio_num: numero de GPIO.
*
*****************************************************************************/
#define gpio_hal_pulldown_act(hal, gpio_num) gpio_ll_pulldown_act((hal)->dev, gpio_num)
/**************************************************************************
* Function:gpio_hal_pulldown_deact
* Preconditions: gpio_ll_pulldown_deact
* Overview: Redefinicion de deshabilitacion la resistencia de pull-down en el GPIO.
* Input: hal:Contexto de la capa HAL.
* 		 gpio_num: numero de GPIO.
*
*****************************************************************************/
#define gpio_hal_pulldown_deact(hal, gpio_num) gpio_ll_pulldown_deact((hal)->dev, gpio_num)
/**************************************************************************
* Function: gpio_hal_set_intr_type
* Preconditions: gpio_ll_set_intr_type
* Overview: Redefinicion de preparacion del gatillo para el tipo de interrupcion.
* Input: hal: Contexto de la capa HAL.
* 		 gpio_num: Numero de GPIO
* 		 intr_type: Tipo de interrupcion
*
*****************************************************************************/
#define gpio_hal_set_intr_type(hal, gpio_num, intr_type) gpio_ll_set_intr_type((hal)->dev, gpio_num, intr_type)
/**************************************************************************
* Function: gpio_hal_get_intr_status
* Preconditions: gpio_ll_get_intr_status
* Overview: Redefincion de obtencion del estatus de interrupcion.
* Input: hal: Contexto de la capa HAL.
* 		 core id : ID de la interrupcion del nucleo
* 		 status: Estado de la interrupcion
*
*****************************************************************************/
#define gpio_hal_get_intr_status(hal, core_id, status) gpio_ll_get_intr_status((hal)->dev, core_id, status)
/**************************************************************************
* Function: gpio_hal_get_intr_status_high
* Preconditions: gpio_ll_get_intr_status_high
* Overview: Redefinicion de obtencion del estado alto de la interrupcion.
* Input: hal: Contexto de la capa HAL.
* 		 core id : ID de la interrupcion del nucleo
* 		 status: Estado de la interrupcion en alto
*
*****************************************************************************/
#define gpio_hal_get_intr_status_high(hal, core_id, status) gpio_ll_get_intr_status_high((hal)->dev, core_id, status)
/**************************************************************************
* Function: gpio_hal_clear_intr_status
* Preconditions: gpio_ll_clear_intr_status
* Overview: Redefinicion de limpiar el estado de la interrupcion.
* Input: hal: Contexto de la capa HAL.
* 		 mask: limpia la mascara del estado de la interrupcion
*
*****************************************************************************/
#define gpio_hal_clear_intr_status(hal, mask) gpio_ll_clear_intr_status((hal)->dev, mask)
/**************************************************************************
* Function: gpio_hal_clear_intr_status_high
* Preconditions: gpio_ll_clear_intr_status_high
* Overview: Redefinicion de limpiar el estado alto de la interrupcion.
* Input: hal: Contexto de la capa HAL.
* 		 mask: limpia la mascara del estado de la interrupcion
*
*****************************************************************************/
#define gpio_hal_clear_intr_status_high(hal, mask) gpio_ll_clear_intr_status_high((hal)->dev, mask)
/**************************************************************************
* Function: gpio_hal_intr_enable_on_core
* Overview: Habilita la señal de interrupcion del modulo GPIO.
* Input: hal: Contexto de la capa HAL.
* 		 gpio_num: Numero de GPIO.
* 		 core_id: Interrupcion habilitada en el CPU para el ID correspondiente
*
*****************************************************************************/
void gpio_hal_intr_enable_on_core(gpio_hal_context_t *hal, gpio_num_t gpio_num, uint32_t core_id);
/**************************************************************************
* Function: gpio_hal_intr_disable
* Overview: Deshabilita la señal de interrupcion del modulo GPIO.
* Input: hal: Contexto de la capa HAL.
* 		 gpio_num: Numero de GPIO.
*
*****************************************************************************/
void gpio_hal_intr_disable(gpio_hal_context_t *hal, gpio_num_t gpio_num);
/**************************************************************************
* Function: gpio_hal_input_disable
* Preconditions: gpio_ll_input_disable
* Overview: Redefinicion de funcion para desactivar el modo de entrada e GPIO.
* Input: hal: Contexto de la capa HAL.
* 		 gpio_num: Numero de GPIO.
*
*****************************************************************************/
#define gpio_hal_input_disable(hal, gpio_num) gpio_ll_input_disable((hal)->dev, gpio_num)
/**************************************************************************
* Function: gpio_hal_input_enable
* Preconditions: gpio_ll_input_enable
* Overview: Redefinicion de funcion para activar el modo de entrada en el GPIO.
* Input: hal: Contexto de la capa HAL.
* 		 gpio_num: Numero de GPIO.
*
*****************************************************************************/
#define gpio_hal_input_enable(hal, gpio_num) gpio_ll_input_enable((hal)->dev, gpio_num)
/**************************************************************************
* Function: gpio_hal_output_disable
* Preconditions: gpio_ll_output_disable
* Overview: Redefinicion de funcion para deshabilitar el modo de salida en GPIO.
* Input: hal: Contexto de la capa HAL.
* 		 gpio_num: Numero de GPIO.
*
*****************************************************************************/
#define gpio_hal_output_disable(hal, gpio_num) gpio_ll_output_disable((hal)->dev, gpio_num)
/**************************************************************************
* Function: gpio_hal_output_enable
* Preconditions: gpio_ll_output_enable
* Overview: Redefinicion de funcion para habilitar el modo de salida en GPIO.
* Input: hal: Contexto de la capa HAL.
* 		 gpio_num: Numero de GPIO.
*
*****************************************************************************/
#define gpio_hal_output_enable(hal, gpio_num) gpio_ll_output_enable((hal)->dev, gpio_num)
/**************************************************************************
* Function: gpio_hal_od_disable
* Preconditions: gpio_ll_od_disable
* Overview: Redefinicion de funcion para deshabilitar el modo open-drain en GPIO.
* Input: hal: Contexto de la capa HAL.
* 		 gpio_num: Numero de GPIO.
*
*****************************************************************************/
#define gpio_hal_od_disable(hal, gpio_num) gpio_ll_od_disable((hal)->dev, gpio_num)
/**************************************************************************
* Function: gpio_hal_od_enable
* Preconditions: gpio_ll_od_enable
* Overview: Redefinicion de funcion para habilitar el modo open-drain en GPIO.
* Input: hal: Contexto de la capa HAL.
* 		 gpio_num: Numero de GPIO.
*
*****************************************************************************/
#define gpio_hal_od_enable(hal, gpio_num) gpio_ll_od_enable((hal)->dev, gpio_num)
/**************************************************************************
* Function: gpio_hal_set_level
* Preconditions: gpio_ll_set_level
* Overview: Redefinicion de funcion para configurar el nivel de salida en GPIO.
* Input: hal: Contexto de la capa HAL.
* 		 gpio_num: Numero de GPIO.
* 		 level: Nivel de salida (0-bajo, 1-alto)
*
*****************************************************************************/
#define gpio_hal_set_level(hal, gpio_num, level) gpio_ll_set_level((hal)->dev, gpio_num, level)
/**************************************************************************
* Function: gpio_hal_get_level
* Preconditions: gpio_ll_get_level
* Overview: Redefinicion para funcion para ver el nivel de entrada de GPIO.
*Input: hal: Contexto de la capa HAL.
* 		 gpio_num: Numero de GPIO.
* Output: 0 si el nivel del GPIO es 0
* 		  1 si el nivel del GPIO es 1
*
*****************************************************************************/
#define gpio_hal_get_level(hal, gpio_num) gpio_ll_get_level((hal)->dev, gpio_num)
/**************************************************************************
* Function: gpio_hal_wakeup_enable
* Preconditions: gpio_ll_wakeup_enable
* Overview: Redefinicion de funcion de habilitacion de wake-up en GPIO.
*Input: hal: Contexto de la capa HAL.
* 		 gpio_num: Numero de GPIO.
* 		 intr_type: Typo de wake-up del GPIO.
*
*****************************************************************************/
#define gpio_hal_wakeup_enable(hal, gpio_num, intr_type) gpio_ll_wakeup_enable((hal)->dev, gpio_num, intr_type)
/**************************************************************************
* Function: gpio_hal_wakeup_disable
* Preconditions: gpio_ll_wakeup_disable
* Overview: Redefinicion de funcion de deshabilitacion de wake-up en GPIO.
*Input: hal: Contexto de la capa HAL.
* 		 gpio_num: Numero de GPIO.
*
*****************************************************************************/
#define gpio_hal_wakeup_disable(hal, gpio_num) gpio_ll_wakeup_disable((hal)->dev, gpio_num)
/**************************************************************************
* Function: gpio_hal_set_drive_capability
* Preconditions: gpio_ll_set_drive_capability
* Overview: Redefinicion de funcion que configura la capacidad del drive del GPIO.
* Input: hal: Contexto de la capa HAL.
* 		gpio_num: Numero de GPIO.
* 		strength: Capacidad de drive del pad
* Output: argumentos de salida
*
*****************************************************************************/
#define gpio_hal_set_drive_capability(hal, gpio_num, strength) gpio_ll_set_drive_capability((hal)->dev, gpio_num, strength)
/**************************************************************************
* Function: gpio_hal_get_drive_capability
* Preconditions: gpio_ll_get_drive_capability
* Overview: Redefinicion de funcion para obtener la capacidad del drive en GPIO.
* Input: hal: Contexto de la capa HAL.
* 		gpio_num: Numero de GPIO.
* 		strength: Apuntador par aceptar la capacidad de drive del pad.
*
*****************************************************************************/
#define gpio_hal_get_drive_capability(hal, gpio_num, strength) gpio_ll_get_drive_capability((hal)->dev, gpio_num, strength)
/**************************************************************************
* Function: gpio_hal_hold_en
* Preconditions: gpio_ll_hold_en
* Overview: Redefinicion de funcion que habilita la funcion para mantener el pad en GPIO.
* 			La funcion de mantener el pad funciona en modo de entrada y de salida, pero deben ser salidas con
* 			esa posibilidad.
* 			Si esta habilitado:
* 				-En modo de salida: El nivel de salida sera bloqueado forzosamente y no se podra cambiar
* 				-En modo de entrada: El valor de la lectura de entrada no cambiara sin importar los cambios
* 				en la entrada.
* 			El estado de las gpio digitales no puede ser mantenido durante el Deep-sleep, y se reanudara la funcion
* 			de mantener cuando el chip despierte del Deep-sleep. Si se necesita que el gpio digital se mantenga durante
* 			Deep-sleep sera necesario llamar a "gpio_deel_sleep_hold_en".
*
* 			Si se desconecta el chip o se llama a gpio_hold_disable se deshabilitara esta funcion.
* Input: hal: Contexto de la capa HAL.
* 		 gpio_num: Numero de GPIO.
*
*****************************************************************************/
#define gpio_hal_hold_en(hal, gpio_num) gpio_ll_hold_en((hal)->dev, gpio_num)
/**************************************************************************
* Function: gpio_hal_hold_dis
* Preconditions: gpio_ll_hold_dis
* Overview: Redefinicion de funcion que deshabilita la funcion para mantener el pad en GPIO.
* 			Cuando el chip se despierta de Deep-sleep, el gpio se configurara en el estado por defecto,
* 			por lo que, el gpio tendra una salida en el nivel por defecto si esta funcion se llama. Si no
* 			que el nivel cambie, el gpio debe ser configurado en un estado conocido antes de llamar esta funcion.
* Input: hal: Contexto de la capa HAL.
* 		 gpio_num: Numero de GPIO.
*
*****************************************************************************/
#define gpio_hal_hold_dis(hal, gpio_num) gpio_ll_hold_dis((hal)->dev, gpio_num)
/**************************************************************************
* Function: gpio_hal_deep_sleep_hold_en
* Preconditions: gpio_ll_deep_sleep_hold_en
* Overview: Redefinicion de funcion que habilita la funcion de mantener el pad en los GPIO digitales
* 			durante Deep-sleep.
* 			Cuando el chip esta en Deep-sleep, todos los gpio digitales mantendran el estado antes del sleep,
* 			y cuando el chip despuerte el estado de los gpio digitales no se mantendra. Hay que notar que la
* 			funcion de mantener el pad solo funciona cuando el chip esta en Deep-sleep, cuando no esta en ese
* 			modo, el estado de los gpio digitales puede ser cambiado incluso si se llamo esta funcion.
* 			Desconexion del chip o llamar a gpio_hol_dis deshabilitara esta funcion, de otra manera la funcion
* 			de mantener los gpio digitales funcionara.
* Input: hal: Contexto de la capa HAL.
*
*****************************************************************************/
#define gpio_hal_deep_sleep_hold_en(hal) gpio_ll_deep_sleep_hold_en((hal)->dev)
/**************************************************************************
* Function: gpio_hal_deep_sleep_hold_dis
* Preconditions: gpio_ll_deep_sleep_hold_dis
* Overview: Redefinicion de funcion que deshabilita la funcion de mantener los GPIO digitales durante Deep-sleep.
* Input: hal: Contexto de la capa HAL.
*
*****************************************************************************/
#define gpio_hal_deep_sleep_hold_dis(hal) gpio_ll_deep_sleep_hold_dis((hal)->dev)
/**************************************************************************
* Function: gpio_hal_iomux_in
* Preconditions: gpio_ll_iomux_in
* Overview: Redefinicion de la funcion de configuracion del pad de entrada para una señal de
* 			periferico a traves del IOMUX.
* Input: hal: Contexto de la capa HAL.
* 		 gpio_num: Numero de GPIO
* 		 signal_idx: Id de señal periferica para entrada.
*
*****************************************************************************/
#define gpio_hal_iomux_in(hal, gpio_num, signal_idx) gpio_ll_iomux_in((hal)->dev, gpio_num, signal_idx)
/**************************************************************************
* Function: gpio_hal_iomux_out
* Preconditions: gpio_ll_iomux_out
* Overview: Redefinicion de la funcion de configuracion del pad de salida para una señal de
* 			periferico a traves del IOMUX.
* Input: hal: Contexto de la capa HAL.
* 		 gpio_num: Numero de GPIO
* 		 func: El numero de funcion del pin periferico para el pin de salida.
* 		 oen_inv: True si se necesita que la habilitacion de salida se invierta, sino False
*
*****************************************************************************/
#define gpio_hal_iomux_out(hal, gpio_num, func, oen_inv) gpio_ll_iomux_out((hal)->dev, gpio_num, func, oen_inv)

#if SOC_GPIO_SUPPORT_FORCE_HOLD
/**************************************************************************
* Function: gpio_hal_force_hold_all
* Preconditions: gpio_ll_force_hold_all
* Overview: Redefinicion de funcion que forza a mantener todos los pads digitales y rtc.
* Input: hal: Contexto de la capa HAL.
*
*****************************************************************************/
#define gpio_hal_force_hold_all(hal) gpio_ll_force_hold_all((hal)->dev)
/**************************************************************************
* Function: gpio_hal_force_unhold_all
* Preconditions:gpio_ll_force_unhold_all
* Overview: Redefinicion de funcion que forza a soltar los pads gpio digitales y rtc.
* Input: hal: Contexto de la capa HAL.
*
*****************************************************************************/
#define gpio_hal_force_unhold_all() gpio_ll_force_unhold_all()
#endif

#if SOC_GPIO_SUPPORT_SLP_SWITCH
/**************************************************************************
* Function: gpio_hal_sleep_pullup_en
* Preconditions: gpio_ll_sleep_pullup_en
* Overview: Redefinicion de funcion que habilita el pull-up en GPIO cuando el sistema esta en sleep.
* Input: hal: Contexto de la capa HAL.
* 		 gpio_num: Numero de GPIO
*
*****************************************************************************/
#define gpio_hal_sleep_pullup_en(hal, gpio_num) gpio_ll_sleep_pullup_en((hal)->dev, gpio_num)
/**************************************************************************
* Function: gpio_hal_sleep_pullup_dis
* Preconditions: gpio_ll_sleep_pullup_dis
* Overview: Redefinicion de funcion que deshabilita el pull-ip en GPIO cuando el sistema esta en sleep.
* Input: hal: Contexto de la capa HAL.
* 		 gpio_num: Numero de GPIO
*
*****************************************************************************/
#define gpio_hal_sleep_pullup_dis(hal, gpio_num) gpio_ll_sleep_pullup_dis((hal)->dev, gpio_num)
/**************************************************************************
* Function: gpio_hal_sleep_pulldown_en
* Preconditions: gpio_ll_sleep_pulldown_en
* Overview: Redefinicion de funcion que habilita el pull-down en GPIO cuando el sistema esta en sleep.
* Input: hal: Contexto de la capa HAL.
* 		 gpio_num: Numero de GPIO
*
*****************************************************************************/
#define gpio_hal_sleep_pulldown_en(hal, gpio_num) gpio_ll_sleep_pulldown_en((hal)->dev, gpio_num)
/**************************************************************************
* Function: gpio_hal_sleep_pulldown_dis
* Preconditions: gpio_ll_sleep_pulldown_dis
* Overview: Redefinicion de funcion que deshabilita el pull-down en GPIO cuando el sistema esta en sleep.
* Input: hal: Contexto de la capa HAL.
* 		 gpio_num: Numero de GPIO
*
*****************************************************************************/
#define gpio_hal_sleep_pulldown_dis(hal, gpio_num) gpio_ll_sleep_pulldown_dis((hal)->dev, gpio_num)
/**************************************************************************
* Function: gpio_hal_sleep_sel_en
* Preconditions: gpio_ll_sleep_sel_en
* Overview: Redefinicion de funcion que habilita la seleccion de sleep en GPIO.
* Input: hal: Contexto de la capa HAL.
* 		 gpio_num: Numero de GPIO
*
*****************************************************************************/
#define gpio_hal_sleep_sel_en(hal, gpio_num) gpio_ll_sleep_sel_en((hal)->dev, gpio_num)
/**************************************************************************
* Function: gpio_hal_sleep_sel_dis
* Preconditions: gpio_ll_sleep_sel_dis
* Overview: Redefinicion de funcion que deshabilita la seleccion de sleep en GPIO.
* Input: hal: Contexto de la capa HAL.
* 		 gpio_num: Numero de GPIO
*
*****************************************************************************/
#define gpio_hal_sleep_sel_dis(hal, gpio_num) gpio_ll_sleep_sel_dis((hal)->dev, gpio_num)
/**************************************************************************
* Function: gpio_hal_sleep_input_disable
* Preconditions: gpio_ll_sleep_input_disable
* Overview: Redefinicion de funcion que deshabilita el modo de entrada en GPIO cuando el sistema esta en sleep.
* Input: hal: Contexto de la capa HAL.
* 		 gpio_num: Numero de GPIO
*
*****************************************************************************/
#define gpio_hal_sleep_input_disable(hal, gpio_num) gpio_ll_sleep_input_disable((hal)->dev, gpio_num)
/**************************************************************************
* Function: gpio_hal_sleep_input_enable
* Preconditions: gpio_ll_sleep_input_enable
* Overview: Redefinicion de funcion que habilita el modo de entrada en GPIO cuando el sistema esta en sleep.
* Input: hal: Contexto de la capa HAL.
* 		 gpio_num: Numero de GPIO
*
*****************************************************************************/
#define gpio_hal_sleep_input_enable(hal, gpio_num) gpio_ll_sleep_input_enable((hal)->dev, gpio_num)
/**************************************************************************
* Function: gpio_hal_sleep_output_disable
* Preconditions: gpio_ll_sleep_output_disable
* Overview: Redefinicion de funcion que deshabilita el modo de salida en GPIO cuando el sistema esta en sleep.
* Input: hal: Contexto de la capa HAL.
* 		 gpio_num: Numero de GPIO
*
*****************************************************************************/
#define gpio_hal_sleep_output_disable(hal, gpio_num) gpio_ll_sleep_output_disable((hal)->dev, gpio_num)
/**************************************************************************
* Function:gpio_hal_sleep_output_enable
* Preconditions: gpio_ll_sleep_output_enable
* Overview: Redefinicion de funcion que habilita el modo de salida en GPIO cuando el sistema esta en sleep.
* Input: hal: Contexto de la capa HAL.
* 		 gpio_num: Numero de GPIO
*
*****************************************************************************/
#define gpio_hal_sleep_output_enable(hal, gpio_num) gpio_ll_sleep_output_enable((hal)->dev, gpio_num)

#if CONFIG_GPIO_ESP32_SUPPORT_SWITCH_SLP_PULL
/**************************************************************************
* Function: gpio_hal_sleep_pupd_config_apply
* Overview: Funcion que aplica configuracion slp_pu/slp_pd. a fun_pu/fun_pd cuando el sistema esta en sleep.
* Input: hal: Contexto de la capa HAL.
* 		 gpio_num: Numero de GPIO
*
*****************************************************************************/
void gpio_hal_sleep_pupd_config_apply(gpio_hal_context_t *hal, gpio_num_t gpio_num);
/**************************************************************************
* Function: gpio_hal_sleep_pupd_config_unapply
* Overview: Funcion que restaura la configuracion fun_pu/fun_pd cuando el sistema despierta.
* Input: hal: Contexto de la capa HAL.
* 		 gpio_num: Numero de GPIO
*
*****************************************************************************/
void gpio_hal_sleep_pupd_config_unapply(gpio_hal_context_t *hal, gpio_num_t gpio_num);
#endif //CONFIG_GPIO_ESP32_SUPPORT_SWITCH_SLP_PULL
#endif //SOC_GPIO_SUPPORT_SLP_SWITCH

#if SOC_GPIO_SUPPORT_DEEPSLEEP_WAKEUP
/**************************************************************************
* Function: gpio_hal_deepsleep_wakeup_enable
* Preconditions: gpio_ll_deepsleep_wakeup_enable
* Overview: Redefinicion de funcion que habilita la funcion de wake-up de un Deep-sleep para el GPIO.
* Input: hal: Contexto de la capa HAL.
* 		 gpio_num: Numero de GPIO
* 		 intr_type: tipo de wake-up para el GPIO
*
*****************************************************************************/
#define gpio_hal_deepsleep_wakeup_enable(hal, gpio_num, intr_type) gpio_ll_deepsleep_wakeup_enable((hal)->dev, gpio_num, intr_type)
/**************************************************************************
* Function: gpio_hal_deepsleep_wakeup_disable
* Preconditions: gpio_ll_deepsleep_wakeup_disable
* Overview: Redefinicion de funcion que deshabilita la funcion de wake-up de un Deep-sleep para el GPIO.
* Input: hal: Contexto de la capa HAL.
* 		 gpio_num: Numero de GPIO
*
*****************************************************************************/
#define gpio_hal_deepsleep_wakeup_disable(hal, gpio_num) gpio_ll_deepsleep_wakeup_disable((hal)->dev, gpio_num)
/**************************************************************************
* Function: gpio_hal_is_valid_deepsleep_wakeup_gpio
* Overview: Definicion de funcion que juzga si el GPIO es valido para despertar el chip de un Deep-sleep.
* Input:  gpio_num: Numero de GPIO
*
*****************************************************************************/
#define gpio_hal_is_valid_deepsleep_wakeup_gpio(gpio_num) (gpio_num <= GPIO_NUM_5)

#endif //SOC_GPIO_SUPPORT_DEEPSLEEP_WAKEUP
/**************************************************************************
* Function: gpio_hal_iomux_func_sel
* Preconditions: gpio_ll_iomux_func_sel
* Overview: Redefinicion de funcion que selecciona una funcion para el pin en el IOMUX.
* Input: pin_name: El nombre del pin a configurar.
* 		 func: funcion a asignar al pin.
*
*****************************************************************************/
#define gpio_hal_iomux_func_sel(pin_name, func) gpio_ll_iomux_func_sel(pin_name, func)

#ifdef __cplusplus
}
#endif
