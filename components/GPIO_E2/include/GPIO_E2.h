 // FileName:        GPIO_E2.h
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

#pragma once

#include "sdkconfig.h"
#include "esp_err.h"
#include <stdbool.h>
#include "esp_intr_alloc.h"
#if !CONFIG_IDF_TARGET_LINUX
#include <esp_types.h>
#include <esp_bit_defs.h>
#include "esp_attr.h"
#include "soc/soc_caps.h"
#include "soc/gpio_periph.h"
#endif // !CONFIG_IDF_TARGET_LINUX
#include "hal/gpio_types.h"

// |================================= WARNING ====================================================== |
// | Including ROM header file in a PUBLIC API file will be REMOVED in the next major release (5.x). |
// | User should include "esp_rom_gpio.h" in their code if they have to use those ROM API.           |
// |================================================================================================ |
#if CONFIG_IDF_TARGET_ESP32
#include "esp32/rom/gpio.h"
#elif CONFIG_IDF_TARGET_ESP32S2
#include "esp32s2/rom/gpio.h"
#elif CONFIG_IDF_TARGET_ESP32S3
#include "esp32s3/rom/gpio.h"
#elif CONFIG_IDF_TARGET_ESP32C3
#include "esp32c3/rom/gpio.h"
#elif CONFIG_IDF_TARGET_ESP32S3
#include "esp32s3/rom/gpio.h"
#elif CONFIG_IDF_TARGET_ESP32H2
#include "esp32h2/rom/gpio.h"
#endif

#ifdef CONFIG_LEGACY_INCLUDE_COMMON_HEADERS
#include "soc/rtc_io_reg.h"
#endif

#ifdef __cplusplus
extern "C" {
#endif

#define GPIO_PIN_COUNT                      (SOC_GPIO_PIN_COUNT)
/// Check whether it is a valid GPIO number
#define GPIO_IS_VALID_GPIO(gpio_num)        (((1ULL << (gpio_num)) & SOC_GPIO_VALID_GPIO_MASK) != 0)
/// Check whether it can be a valid GPIO number of output mode
#define GPIO_IS_VALID_OUTPUT_GPIO(gpio_num) (((1ULL << (gpio_num)) & SOC_GPIO_VALID_OUTPUT_GPIO_MASK) != 0)


typedef intr_handle_t gpio_isr_handle_t;
/**************************************************************************
* Function: gpio_output_disable
* Overview: Funcion para desactivar un pin como salida
* Input: gpio_num
* Output: ESP_OK: Exitoso
*  		  ESP_ERR_INVALID_ARG: Error de parametro
*
*****************************************************************************/
esp_err_t gpio_output_disable(gpio_num_t gpio_num);
/**************************************************************************
* Function: gpio_output_enable
* Overview: Funcion para activar un pin como salida
* Input: gpio_num
* Output: ESP_OK: Exitoso
*  		  ESP_ERR_INVALID_ARG: Error de parametro
*
*****************************************************************************/
esp_err_t gpio_output_enable(gpio_num_t gpio_num);
/**************************************************************************
* Function: gpio_set_input_isr
* Overview: Funcion para configurar un pin como entrada con interrupcion
* Input: gpio_num y isr_type
* Output: ESP_OK: Exitoso
*  		  ESP_ERR_INVALID_ARG: Error de parametro
*
*****************************************************************************/
esp_err_t gpio_set_input_isr(gpio_num_t gpio_num, gpio_int_type_t isr_type);
/**************************************************************************
* Function: gpio_set_input_pullup
* Overview: Funcion para configurar un pin como entrada con pullup.
* Input: gpio_num.
* Output: ESP_OK: Exitoso
*  		  ESP_ERR_INVALID_ARG: Error de parametro
*
*****************************************************************************/
esp_err_t gpio_set_input_pullup(gpio_num_t gpio_num);
/**************************************************************************
* Function: gpio_set_input_pulldown
* Overview: Funcion para configurar un pin como entrada con pulldown.
* Input: gpio_num.
* Output: ret
*
*****************************************************************************/
esp_err_t gpio_set_input_pulldown(gpio_num_t gpio_num);
/**************************************************************************
* Function: gpio_set_output
* Overview: Funcion para configurar la salida de GPIO.
* Input: gpio_num.
* Output: ret
*
*****************************************************************************/
esp_err_t gpio_set_output(gpio_num_t gpio_num);
/**************************************************************************
* Function: gpio_act_left_op
* Preconditions: gpio_hal_act_left_op
* Overview: Funcion que activa el lado izquierdo del chip como salida.
* Output: ret
*
*****************************************************************************/
esp_err_t gpio_act_left_op(void);
/**************************************************************************
* Function: gpio_deact_left_op
* Preconditions: gpio_hal_deact_left_op
* Overview: Funcion que desactiva el lado izquierdo del chip como salida.
* Output: ESP_OK: Exitoso
*  		  ESP_ERR_INVALID_ARG: Error de parametro
*
*****************************************************************************/
esp_err_t gpio_deact_left_op(void);
/**************************************************************************
* Function: gpio_act_right_op
* Preconditions: gpio_hal_act_right_op
* Overview: Funcion que activa el lado derecho del chip como salida.
* Output: ESP_OK: Exitoso
*  		  ESP_ERR_INVALID_ARG: Error de parametro
*
*****************************************************************************/
esp_err_t gpio_act_right_op(void);
/**************************************************************************
* Function: gpio_deact_right_op
* Preconditions: gpio_hal_deact_right_op
* Overview: Funcion que desactiva el lado derecho del chip como salida.
* Output: ESP_OK: Exitoso
*  		  ESP_ERR_INVALID_ARG: Error de parametro
*
*****************************************************************************/
esp_err_t gpio_deact_right_op(void);
/**************************************************************************
* Function: gpio_act_left_ip
* Preconditions: gpio_hal_act_left_ip
* Overview: Funcion que activa el lado izquierdo del chip como entrada.
* Output: ESP_OK: Exitoso
*  		  ESP_ERR_INVALID_ARG: Error de parametro
*
*****************************************************************************/
esp_err_t gpio_act_left_ip(void);
/**************************************************************************
* Function: gpio_deact_left_ip
* Preconditions: gpio_hal_deact_left_ip
* Overview: Funcion que desactiva el lado izquierdo del chip como entrada.
* Output: ESP_OK: Exitoso
*  		  ESP_ERR_INVALID_ARG: Error de parametro
*
*****************************************************************************/
esp_err_t gpio_deact_left_ip(void);
/**************************************************************************
* Function: gpio_act_right_ip
* Preconditions: gpio_hal_act_right_ip
* Overview: Funcion que activa el lado derecho del chip como entrada.
* Output: ESP_OK: Exitoso
*  		  ESP_ERR_INVALID_ARG: Error de parametro
*
*****************************************************************************/
esp_err_t gpio_act_right_ip(void);
/**************************************************************************
* Function: gpio_deact_right_ip
* Preconditions: gpio_hal_deact_right_ip
* Overview: Funcion que desactiva el lado derecho del chip como entrada.
* Output: ESP_OK: Exitoso
*  		  ESP_ERR_INVALID_ARG: Error de parametro
*
*****************************************************************************/
esp_err_t gpio_deact_right_ip(void);
/**************************************************************************
 * Function: gpio_act_left_pulldown
 * Preconditions: gpio_hal_act_left_pulldown
 * Overview: Funcion que activa las resistencias de pulldown del lado izquierdo del chip.
 * Output: ESP_OK: Exitoso
 *  	   ESP_ERR_INVALID_ARG: Error de parametro
 *
 *****************************************************************************/
esp_err_t gpio_act_left_pulldown(void);
/**************************************************************************
 * Function: gpio_deact_left_pulldown
 * Preconditions: gpio_hal_deact_left_pulldown
 * Overview: Funcion que desactiva las resistencias de pulldown del lado izquierdo del chip.
 * Output: ESP_OK: Exitoso
 *  	   ESP_ERR_INVALID_ARG: Error de parametro
 *
 *****************************************************************************/
esp_err_t gpio_deact_left_pulldown(void);
/**************************************************************************
 * Function: gpio_act_right_pulldown
 * Preconditions: gpio_hal_act_right_pulldown
 * Overview: Funcion que activa las resistencias de pulldown del lado derecho del chip.
 * Output: ESP_OK: Exitoso
 *  	   ESP_ERR_INVALID_ARG: Error de parametro
 *
 *****************************************************************************/
esp_err_t gpio_act_right_pulldown(void);
/**************************************************************************
 * Function: gpio_deact_right_pulldown
 * Preconditions: gpio_hal_deact_right_pulldown
 * Overview: Funcion que desactiva las resistencias de pulldown del lado derecho del chip.
 * Output: ESP_OK: Exitoso
 *  	   ESP_ERR_INVALID_ARG: Error de parametro
 *
 *****************************************************************************/
esp_err_t gpio_deact_right_pulldown(void);
/**************************************************************************
 * Function: gpio_act_left_pullup
 * Preconditions: gpio_hal_act_left_pullup
 * Overview: Funcion que activa las resistencias de pullup del lado izquierdo del chip.
 * Output: ESP_OK: Exitoso
 *  	   ESP_ERR_INVALID_ARG: Error de parametro
 *
 *****************************************************************************/
esp_err_t gpio_act_left_pullup(void);
/**************************************************************************
 * Function: gpio_deact_left_pullup
 * Preconditions: gpio_hal_deact_left_pullup
 * Overview: Funcion que desactiva las resistencias de pullup del lado izquierdo del chip.
 * Output: ESP_OK: Exitoso
 *  	   ESP_ERR_INVALID_ARG: Error de parametro
 *
 *****************************************************************************/
esp_err_t gpio_deact_left_pullup(void);
/**************************************************************************
 * Function: gpio_act_right_pullup
 * Preconditions: gpio_hal_act_right_pullup
 * Overview: Funcion que activa las resistencias de pullup del lado derecho del chip.
 * Output: ESP_OK: Exitoso
 *  	   ESP_ERR_INVALID_ARG: Error de parametro
 *
 *****************************************************************************/
esp_err_t gpio_act_right_pullup(void);
/**************************************************************************
 * Function: gpio_deact_right_pullup
 * Preconditions: gpio_hal_deact_right_pullup
 * Overview: Funcion que desactiva las resistencias de pullup del lado derecho del chip.
 * Output: ESP_OK: Exitoso
 *  	   ESP_ERR_INVALID_ARG: Error de parametro
 *
 *****************************************************************************/
esp_err_t gpio_deact_right_pullup(void);
/**************************************************************************
* Function: gpio_reset_right
* Overview: Funcion que reinicia el lado derecho del chip.
* Output: ESP_OK: Exitoso
*  		  ESP_ERR_INVALID_ARG: Error de parametro
*
*****************************************************************************/
esp_err_t gpio_reset_right(void);
/**************************************************************************
* Function: gpio_reset_left
* Overview: Funcion que reinicia el lado izquierdo del chip.
* Output: ESP_OK: Exitoso
*  		  ESP_ERR_INVALID_ARG: Error de parametro
*
*****************************************************************************/
esp_err_t gpio_reset_left(void);
/**************************************************************************
* Function: gpio_config
* Overview: Configuracion comun del GPIO.
* 			Configura el modo, pull-up/pull-down,IntrType
* Input: pGPIOConfig: Apuntador a la estructura de configuracion del GPIO.
* Output: ESP_OK: Exitoso
* 		  ESP_ERR_INVALID_ARG: Error de parametro
*
*****************************************************************************/
esp_err_t gpio_config(const gpio_config_t *pGPIOConfig);
/**************************************************************************
* Function: gpio_reset_pin
* Overview: Funcion que regresa el GPIO a su estado inicial.
* 			Selecciona la funcion, habilitacion del pullup y deshabilita entrada y salida
* Input: gpio_num: Numero de GPIO.
* Output: ESP_OK: Exitoso
*  		  ESP_ERR_INVALID_ARG: Error de parametro
*
*****************************************************************************/
esp_err_t gpio_reset_pin(gpio_num_t gpio_num);
/**************************************************************************
* Function: gpio_set_intr_type
* Overview: Configuracion del tipo de interrupcion.
* Input: gpio_num: Numero de GPIO.
* 		 intr_type: Tipo de interrupcion
* Output: ESP_OK: Exitoso
* 		  ESP_ERR_INVALID_ARG: Error de parametro
*
*****************************************************************************/
esp_err_t gpio_set_intr_type(gpio_num_t gpio_num, gpio_int_type_t intr_type);
/**************************************************************************
* Function: gpio_intr_enable
* Overview: Funcion que habilita el modulo de interrupcion de GPIO.
* Input: gpio_num: Numero de GPIO.
* Output: ESP_OK: Exitoso
* 		  ESP_ERR_INVALID_ARG: Error de parametro
*
*****************************************************************************/
esp_err_t gpio_intr_enable(gpio_num_t gpio_num);
/**************************************************************************
* Function: gpio_intr_disable
* Overview: Deshabilita el modulo de interrupciones del GPIO.
* Input: gpio_num: Numero de GPIO.
* Output: ESP_OK: Exitoso
* 		  ESP_ERR_INVALID_ARG: Error de parametro
*
*****************************************************************************/
esp_err_t gpio_intr_disable(gpio_num_t gpio_num);
/**************************************************************************
* Function: gpio_set_level
* Overview: Configuracion del nivel de salida del GPIO.
* Input: gpio_num: Numero de GPIO.
* 		 level: Nivel de salida (0-bajo, 1-alto)
* Output: ESP_OK: Exitoso
* 		  ESP_ERR_INVALID_ARG: Error de parametro
*
*****************************************************************************/
esp_err_t gpio_set_level(gpio_num_t gpio_num, uint32_t level);
/**************************************************************************
* Function: gpio_get_level
* Overview: Obtencion del nivel de entrada el GPIO.
* Input: gpio_num: Numero de GPIO..
* Output: 0 cuando la entrada es 0
* 		  1 cuando la entrada es 1
*
*****************************************************************************/
int gpio_get_level(gpio_num_t gpio_num);
/**************************************************************************
* Function: gpio_set_direction
* Overview: Configura la direccion del GPIO, como output_only, input_only, output_and_input.
* Input: gpio_num: Numero de GPIO.
* 		 mode: Direccion de GPIO
* Output: ESP_OK: Exitoso
* 		  ESP_ERR_INVALID_ARG: Error de GPIO
*
*****************************************************************************/
esp_err_t gpio_set_direction(gpio_num_t gpio_num, gpio_mode_t mode);
/**************************************************************************
* Function: gpio_set_pull_mode
* Overview: Configura las resistencias de pull-up/pull-down para el GPIO.
* Input: gpio_num: Numero de GPIO.
* 		 pull: Modo de pull
* Output: ESP_OK: Exitoso
* 		  ESP_ERR_INVALID_ARG: Error de GPIO
*
*****************************************************************************/
esp_err_t gpio_set_pull_mode(gpio_num_t gpio_num, gpio_pull_mode_t pull);
/**************************************************************************
* Function: gpio_wakeup_enable
* Overview: Habilita la funcion de wake-up del GPIO.
* Input: gpio_num: Numero de GPIO.
* 		 intr_type: tipo de Wake_up
* Output: ESP_OK: Exitoso
* 		  ESP_ERR_INVALID_ARG: Error de GPIO
*
*****************************************************************************/
esp_err_t gpio_wakeup_enable(gpio_num_t gpio_num, gpio_int_type_t intr_type);
/**************************************************************************
* Function: gpio_wakeup_disable
* Overview: Deshabilita la funcion de Wake-up del GPIO.
* Input: gpio_num: Numero de GPIO
* Output: ESP_OK: Exitoso
* 		  ESP_ERR_INVALID_ARG: Error de GPIO
*
*****************************************************************************/
esp_err_t gpio_wakeup_disable(gpio_num_t gpio_num);
/**************************************************************************
* Function: gpio_isr_register
* Overview: Registra el handler de interrupciones del GPIO, que es un ISR. El handler estara adjunto
* 			al mismo nucleo del CPU en el que esta funcion este corriendo.
* 			La funcion ISR es llamada siempre que una interrupcion del GPIO ocurre.
* Input: fn: Handler de la funcion de interrupcion.
* 		 arg: Parametro para la funcion handler
* 		 intr_alloc_flags: Banderas usadas para colocar la interrupcion
* 		 handle: Apuntador para devolver handle.
* Output: ESP_OK: Exitoso
* 		  ESP_ERR_INVALID_ARG: Error de GPIO
* 		  ESP_ERR_NOT_FOUND_ No se encontro una interrupcion libre entre las banderas especificadas
*
*****************************************************************************/
esp_err_t gpio_isr_register(void (*fn)(void *), void *arg, int intr_alloc_flags, gpio_isr_handle_t *handle);
/**************************************************************************
* Function: gpio_pullup_en
* Overview: Habilita el pull-up del GPIO.
* Input: gpio_num: Numero de GPIO
* Output: ESP_OK: Exitoso
* 		  ESP_ERR_INVALID_ARG: Error de GPIO
*
*****************************************************************************/
esp_err_t gpio_pullup_en(gpio_num_t gpio_num);
/**************************************************************************
* Function: gpio_pullup_dis
* Overview: Deshabilita el pull-up en el GPIO.
* Input: gpio_num: Numero de GPIO
* Output: ESP_OK: Exitoso
* 		  ESP_ERR_INVALID_ARG: Error de GPIO
*
*****************************************************************************/
esp_err_t gpio_pullup_dis(gpio_num_t gpio_num);
/**************************************************************************
* Function: gpio_pulldown_en
* Overview: Habilita el pull-down en el GPIO.
* Input: gpio_num: Numero de GPIO.
* Output: ESP_OK: Exitoso
* 		  ESP_ERR_INVALID_ARG: Error de GPIO
*
*****************************************************************************/
esp_err_t gpio_pulldown_en(gpio_num_t gpio_num);
/**************************************************************************
* Function: gpio_pulldown_dis
* Overview: Deshabilita el pull-down en el GPIO.
* Input: gpio_num: Numero de GPIO.
* Output: ESP_OK: Exitoso
* 		  ESP_ERR_INVALID_ARG: Error de GPIO
*
*****************************************************************************/
esp_err_t gpio_pulldown_dis(gpio_num_t gpio_num);
/**************************************************************************
* Function: gpio_install_isr_service
* Overview: Instala el driver del handler del ISR para el GPIO, lo que permite handlers de interrupcion por pin
* 			Esta funcion es incompatible con gpio_isr_register()
* Input: intr_alloc_flags: Las banderas usadas para colocar la interrupcion.
* 		 ESP_INTR_FLAG_*: Valores
* Output: ESP_OK: Exitoso
*         ESP_ERR_NO_MEM: Memoria insuficiente para instalar este servicio
*         ESP_ERR_INVALID_STATE ISR: El servicio ya esta instalado
*         ESP_ERR_NOT_FOUND: No se encontro una interrupcion libre con las banderas especificadas
*         ESP_ERR_INVALID_ARG GPIO: Error
*
*****************************************************************************/
esp_err_t gpio_install_isr_service(int intr_alloc_flags);
/**************************************************************************
* Function: gpio_uninstall_isr_service
* Overview: Desinstala el driver del servicio de ISR del GPIO.
*
*****************************************************************************/
void gpio_uninstall_isr_service(void);
/**************************************************************************
* Function: gpio_isr_handler_add
* Overview: Añade el handler para el ISR del pin GPIO correspondiente.
* 			Llama esta funcion despues de usar gpio_install_isr_service() para instalar el driver
* 			del servicio handler del ISR.
* 			Los pines con el handler del ISR no necesitan ser declarados con IRAM_ATTR, a menos
* 			de que se pase la bandera ESP_INTR_FLAG_IRAM cuando se este colocando el ISR en el
* 			gpio_install_isr_service().
* 			El handler dle ISR se llamara desde una ISR. Asi se tendra un tamaño limite para el
* 			stack. Este limite es menor comparado con una interrupcion global del GPIO debido al
* 			nivel adicional de indireccion
* Input: gpio_num: Numero del GPIO.
* 		 isr_handler: Funcion del handler de ISR para el GPIO correspondiente.
* 		 args: parametro para el handler del ISR.
* Output: ESP_OK: Exitoso
* 		  ESP_ERR_INVALID_STATE: Estado equivocado, el servicio de ISR no se ha inicializado
* 		  ESP_ERR_INVALID_ARG: Error de parametro
*
*****************************************************************************/
esp_err_t gpio_isr_handler_add(gpio_num_t gpio_num, gpio_isr_t isr_handler, void *args);
/**************************************************************************
* Function: gpio_isr_handler_remove
* Overview: Remueve el handler del ISR para el pin GPIO correspondiente.
* Input: gpio_num: Numero del GPIO.
* Output: ESP_OK: Exitoso
* 		  ESP_ERR_INVALID_STATE: Estado equivocado, el servicio de ISR no se ha inicializado
* 		  ESP_ERR_INVALID_ARG: Error de parametro
*
*****************************************************************************/
esp_err_t gpio_isr_handler_remove(gpio_num_t gpio_num);
/**************************************************************************
* Function: gpio_set_drive_capability
* Overview: Configura la capacidad del pad drive.
* Input: gpio_num: Numero del GPIO.
* 		 strength: Capacidad del pad.
* Output: ESP_OK: Exitoso
* 		  ESP_ERR_INVALID_ARG: Error de parametro
*
*****************************************************************************/
esp_err_t gpio_set_drive_capability(gpio_num_t gpio_num, gpio_drive_cap_t strength);
/**************************************************************************
* Function: gpio_get_drive_capability
* Overview: Obtiene la capacidad del pad drive.
* Input: gpio_num: Numero del GPIO.
* 		 strength: Capacidad del pad.
* Output: ESP_OK: Exitoso
* 		  ESP_ERR_INVALID_ARG: Error de parametro
*
*****************************************************************************/
esp_err_t gpio_get_drive_capability(gpio_num_t gpio_num, gpio_drive_cap_t *strength);
/**************************************************************************
* Function: gpio_hold_en
* Overview: Activa la funcion de mantener el pad
* 			Esta funcion funciona tanto en modo de entrada y salida, pero debe ser una salida capaz de hacer esto
* 			Si se habilita el mantener el pad:
* 				En modo de salida: el nivel de salida se debera bloquear y no se podra cambiar
* 				En modo de entrada: el valor de lectura de entrada no cambiara, sin importar de los cambios de señal
* 			El estado de un gpio digital no puede mantenerse durante el Deep-sleep, y se reanudara la funcion de mantener
* 			cuando el chip despierte del Deep-sleep. Si el gpio digital tambien se debe mantener durante el Deep-sleep.
* Input: gpio_num: Numero del GPIO.
* Output: ESP_OK: Exitoso
* 		  ESP_ERR_NOT_SUPPORTED: No soporta la funcion de mantener el pad.
*
*****************************************************************************/
esp_err_t gpio_hold_en(gpio_num_t gpio_num);
/**************************************************************************
* Function: gpio_hold_dis
* Overview: Deshabilita la funcion de mantener el pad.
* 			Cuando el chip se reactiva de un Deep-sleep, el gpio se configurara en el estado default, asi, la salida
* 			del gpio sera el nivel por defecto si esta funcion es llamada. Si no se desea que el nivel cambie, el gpio debera ser
* 			configurado en un estado conocido antes de que esta funcion sea llamada.
* Input: gpio_num: Numero del GPIO.
* Output: ESP_OK: Exitoso
* 		  ESP_ERR_NOT_SUPPORTED: No soporta la funcion de mantener el pad.
*
*****************************************************************************/
esp_err_t gpio_hold_dis(gpio_num_t gpio_num);
/**************************************************************************
* Function: gpio_deep_sleep_hold_en
* Overview: Habilita la funcion de mantener en todos los gpio digitales durante Deep-sleep.
* 			Cuando el chip esta en el modo Deep-sleep, todos los gpio digitales mantendran el estado de antes del sleep
* 			y cuando el chip se reactive el estados de los gpio digitales no se mantendra.
*
* 			Si se corta la energia o se llama gpio_hold_dis deshabilitara esta funcion, de otra manera seguira funcionando

*
*****************************************************************************/
void gpio_deep_sleep_hold_en(void);
/**************************************************************************
* Function: gpio_deep_sleep_hold_dis
* Overview: Deshabilita la funcion de mantener el pad de todos los gpio digitales durante el Deep-sleep.
*
*****************************************************************************/
void gpio_deep_sleep_hold_dis(void);
/**************************************************************************
* Function: gpio_iomux_in
* Overview: Configura la entrada del pad para una señal periferica a traves del IOMUX.
* Input: gpio_num: Numero del GPIO.
* 		 signal_idx: Id de la señal periferica para la entrada.
*
*****************************************************************************/
void gpio_iomux_in(uint32_t gpio_num, uint32_t signal_idx);
/**************************************************************************
* Function: gpio_iomux_out
* Overview: Configura la salida periferica para un pad GPIO a traves de un IOMUX.
* Input: gpio_num: Numero del GPIO.
* 		 func: El numero de funcion del pin periferico al pin de salida
* 		 oen_inv: True en caso de que la habilitacion de salida deba ser invertida, de otro modo False.
* Output: argumentos de salida
*
*****************************************************************************/
void gpio_iomux_out(uint8_t gpio_num, int func, bool oen_inv);

#if SOC_GPIO_SUPPORT_FORCE_HOLD
/**
  * @brief Force hold digital and rtc gpio pad.
  * @note GPIO force hold, whether the chip in sleep mode or wakeup mode.
  * */
esp_err_t gpio_force_hold_all(void);

/**
  * @brief Force unhold digital and rtc gpio pad.
  * @note GPIO force unhold, whether the chip in sleep mode or wakeup mode.
  * */
esp_err_t gpio_force_unhold_all(void);
#endif

#if SOC_GPIO_SUPPORT_SLP_SWITCH
/**************************************************************************
* Function: gpio_sleep_sel_en
* Overview: Habilita SLP_SEL para cambiar el estado del GPIO en lightsleep.
* Input: gpio_num: Numero del GPIO.
* Output: ESP_OK: Exitoso
*
*****************************************************************************/
esp_err_t gpio_sleep_sel_en(gpio_num_t gpio_num);
/**************************************************************************
* Function: gpio_sleep_sel_dis
* Overview: Deshabiita SLP_SEL para cambiar el estado del GPIO automaticamente en lightsleep.
* Input: gpio_num: Numero del GPIO.
* Output: ESP_OK: Exitoso
*
*****************************************************************************/
esp_err_t gpio_sleep_sel_dis(gpio_num_t gpio_num);
/**************************************************************************
* Function:gpio_sleep_set_direction
* Overview:Configura la direccion del GPIO en el sleep
* 				Se puede configurar la direccion del GPIO como output_only, input_only, output_and_input
* Input: gpio_num: Numero del GPIO.
* 		 mode: Direccion del GPIO
* Output: ESP_OK: Exitoso
* 		  ESP_ERR_INVALID_ARG: Error de GPIO
*
*****************************************************************************/
esp_err_t gpio_sleep_set_direction(gpio_num_t gpio_num, gpio_mode_t mode);
/**************************************************************************
* Function: gpio_sleep_set_pull_mode
* Overview: Configura las resistencias del pull-up/pull-down durante el sleep.
* Input: gpio_num: Numero del GPIO.
* 		 pull: Modo de pull para el GPIO
* Output: ESP_OK: Exitoso
* 		  ESP_ERR_INVALID_ARG: Error de GPIO
*
*****************************************************************************/
esp_err_t gpio_sleep_set_pull_mode(gpio_num_t gpio_num, gpio_pull_mode_t pull);
#endif

#if SOC_GPIO_SUPPORT_DEEPSLEEP_WAKEUP

#define GPIO_IS_DEEP_SLEEP_WAKEUP_VALID_GPIO(gpio_num)    ((gpio_num >= 0) && \
                                                          (((1ULL << (gpio_num)) & SOC_GPIO_DEEP_SLEEP_WAKEUP_VALID_GPIO_MASK) != 0))
/**************************************************************************
* Function: gpio_deep_sleep_wakeup_enable
* Overview: Habilita la funcion de wake-up desde el deep-sleep para el GPIO.
* Input: gpio_num: Numero del GPIO.
* 		 intr_type: tipo de wake-up del GPIO
* Output: ESP_OK: Exitoso
* 		  ESP_ERR_INVALID_ARG: Error de GPIO
*
*****************************************************************************/
esp_err_t gpio_deep_sleep_wakeup_enable(gpio_num_t gpio_num, gpio_int_type_t intr_type);
/**************************************************************************
* Function: gpio_deep_sleep_wakeup_disable
* Overview: Deshabilita la funcion de wake-up desde el deep-sleep para el GPIO.
* Input: gpio_num: Numero del GPIO.
* Output: ESP_OK: Exitoso
* 		  ESP_ERR_INVALID_ARG: Error de GPIO
*
*****************************************************************************/
esp_err_t gpio_deep_sleep_wakeup_disable(gpio_num_t gpio_num);

#endif

#ifdef __cplusplus
}
#endif
