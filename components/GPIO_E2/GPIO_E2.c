 // FileName:        GPIO_E2.c
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
/*********Librerias necesarias**********/
#include <esp_types.h>
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "GPIO_E2.h"
#include "driver/rtc_io.h"
#include "soc/soc.h"
#include "soc/periph_defs.h"
#if !CONFIG_FREERTOS_UNICORE
#include "esp_ipc.h"
#endif

#include "soc/soc_caps.h"
#include "soc/gpio_periph.h"
#include "esp_log.h"
#include "esp_check.h"
#include "../../DRIVERS/GPIO_HAL.h"
#include "esp_rom_gpio.h"

static const char *GPIO_TAG = "gpio";
#define GPIO_CHECK(a, str, ret_val) ESP_RETURN_ON_FALSE(a, ret_val, GPIO_TAG, "%s", str)

#define GPIO_ISR_CORE_ID_UNINIT    (3)
//El valor default para SOC_GPIO_SUPPORT_RTC_INDEPENDENT es 0
#ifndef SOC_GPIO_SUPPORT_RTC_INDEPENDENT
#define SOC_GPIO_SUPPORT_RTC_INDEPENDENT 0
#endif

typedef struct {
    gpio_isr_t fn;   /*!< Funcion de isr */
    void *args;      /*!< args dela funcion de isr */
} gpio_isr_func_t;
// Usado por la llamada IPC para registrar la rutin del servicio de interrupcion
typedef struct {
    int source;               /*!< Source de ISR */
    int intr_alloc_flags;     /*!< ISR flag de alloc */
    void (*fn)(void*);        /*!< Funcion de ISR */
    void *arg;                /*!< args de funcion de ISR*/
    void *handle;             /*!< handle de ISR */
    esp_err_t ret;
} gpio_isr_alloc_t;

typedef struct {
    gpio_hal_context_t *gpio_hal;
    portMUX_TYPE gpio_spinlock;
    uint32_t isr_core_id;
    gpio_isr_func_t *gpio_isr_func;
    gpio_isr_handle_t gpio_isr_handle;
} gpio_context_t;
/**************************************************************************
* Function: gpio_hal_context_t
* Overview: Funcion que declara el contexto del HAL.
*
*****************************************************************************/
static gpio_hal_context_t _gpio_hal = {
    .dev = GPIO_HAL_GET_HW(GPIO_PORT_0)
};
/**************************************************************************
* Function: gpio_context_t
* Overview: Funcion que declara el contexto siendo este el hal, spinlock, core_id e isr_func.
* Input: argumentos de entrada.
* Output: argumentos de salida
*
*****************************************************************************/
static gpio_context_t gpio_context = {
    .gpio_hal = &_gpio_hal,
    .gpio_spinlock = portMUX_INITIALIZER_UNLOCKED,
    .isr_core_id = GPIO_ISR_CORE_ID_UNINIT,
    .gpio_isr_func = NULL,
};
/**************************************************************************
* Function: gpio_output_disable
* Preconditions: gpio_context_t
* Overview: Funcion que deshabilita el pin seleccionado como salida.
* Input: gpio_num.
* Output: ESP_OK
*
*****************************************************************************/
esp_err_t gpio_output_disable(gpio_num_t gpio_num)
{
    GPIO_CHECK(GPIO_IS_VALID_GPIO(gpio_num), "GPIO number error", ESP_ERR_INVALID_ARG);
    gpio_hal_output_disable(gpio_context.gpio_hal, gpio_num);
    return ESP_OK;
}
/**************************************************************************
* Function: gpio_output_enable
* Preconditions: gpio_context_t
* Overview: Funcion que habilita el pin seleccionado como salida.
* Input: gpio_num.
* Output: ESP_OK
*
*****************************************************************************/
esp_err_t gpio_output_enable(gpio_num_t gpio_num)
{
    GPIO_CHECK(GPIO_IS_VALID_OUTPUT_GPIO(gpio_num), "GPIO output gpio_num error", ESP_ERR_INVALID_ARG);
    gpio_hal_output_enable(gpio_context.gpio_hal, gpio_num);
    esp_rom_gpio_connect_out_signal(gpio_num, SIG_GPIO_OUT_IDX, false, false);
    return ESP_OK;
}
/**************************************************************************
* Function: gpio_set_output
* Overview: Funcion que configura el pin seleccionado como salida.
* Input: gpio_num.
* Output: ret
*
*****************************************************************************/
esp_err_t gpio_set_output(gpio_num_t gpio_num){
	   GPIO_CHECK(GPIO_IS_VALID_GPIO(gpio_num), "GPIO number error", ESP_ERR_INVALID_ARG);

	    if ((GPIO_IS_VALID_OUTPUT_GPIO(gpio_num) != true) && (GPIO_MODE_DEF_OUTPUT)) {
	        ESP_LOGE(GPIO_TAG, "io_num=%d can only be input", gpio_num);
	        return ESP_ERR_INVALID_ARG;
	    }
	    esp_err_t ret = ESP_OK;
	    gpio_output_enable(gpio_num);
	    return ret;
}
/**************************************************************************
* Function: gpio_set_input_pullup
* Overview: Funcion que configura el pin como entrada y activa el pullup.
* Input: gpio_num.
* Output: ret
*
*****************************************************************************/
esp_err_t gpio_set_input_pullup(gpio_num_t gpio_num){
	 GPIO_CHECK(GPIO_IS_VALID_GPIO(gpio_num), "GPIO number error", ESP_ERR_INVALID_ARG);

	    if ((GPIO_IS_VALID_OUTPUT_GPIO(gpio_num) != true) && (GPIO_MODE_DEF_INPUT & GPIO_MODE_DEF_OUTPUT)) {
	        ESP_LOGE(GPIO_TAG, "io_num=%d can only be input", gpio_num);
	        return ESP_ERR_INVALID_ARG;
	    }
	    esp_err_t ret = ESP_OK;
	    gpio_hal_input_enable(gpio_context.gpio_hal, gpio_num);

	    if (!rtc_gpio_is_valid_gpio(gpio_num) || SOC_GPIO_SUPPORT_RTC_INDEPENDENT) {
	        portENTER_CRITICAL(&gpio_context.gpio_spinlock);
	        gpio_hal_pullup_act(gpio_context.gpio_hal, gpio_num);
	        portEXIT_CRITICAL(&gpio_context.gpio_spinlock);
	    } else {
	#if SOC_RTCIO_INPUT_OUTPUT_SUPPORTED
	        rtc_gpio_pullup_en(gpio_num);
	#else
	        abort(); // This should be eliminated as unreachable, unless a programming error has occured
	#endif
	    }

	    return ret;
}
/**************************************************************************
* Function: gpio_set_input_pulldown
* Overview: Funcion que habilita el pin seleccionado como entrada y activa el pulldown.
* Input: gpio_num.
* Output: ret
*
*****************************************************************************/
esp_err_t gpio_set_input_pulldown(gpio_num_t gpio_num){
	 GPIO_CHECK(GPIO_IS_VALID_GPIO(gpio_num), "GPIO number error", ESP_ERR_INVALID_ARG);

	    if ((GPIO_IS_VALID_OUTPUT_GPIO(gpio_num) != true) && (GPIO_MODE_DEF_INPUT & GPIO_MODE_DEF_OUTPUT)) {
	        ESP_LOGE(GPIO_TAG, "io_num=%d can only be input", gpio_num);
	        return ESP_ERR_INVALID_ARG;
	    }
	    esp_err_t ret = ESP_OK;
	    gpio_hal_input_enable(gpio_context.gpio_hal, gpio_num);

	    if (!rtc_gpio_is_valid_gpio(gpio_num) || SOC_GPIO_SUPPORT_RTC_INDEPENDENT) {
	        portENTER_CRITICAL(&gpio_context.gpio_spinlock);
	        gpio_hal_pulldown_act(gpio_context.gpio_hal, gpio_num);
	        portEXIT_CRITICAL(&gpio_context.gpio_spinlock);
	    } else {
	#if SOC_RTCIO_INPUT_OUTPUT_SUPPORTED
	        rtc_gpio_pulldown_en(gpio_num);
	#else
	        abort(); // This should be eliminated as unreachable, unless a programming error has occured
	#endif
	    }

	    return ret;
}
/**************************************************************************
* Function: gpio_set_input_isr
* Overview: Funcion que configura el pin como entrada y permite configurar las interrupciones.
* Input: gpio_num
* 		 isr_type
* Output: ESP_OK
*
*****************************************************************************/
esp_err_t gpio_set_input_isr(gpio_num_t gpio_num, gpio_int_type_t isr_type){
	gpio_config_t pGPIOConfig;
	pGPIOConfig.pin_bit_mask = (1ULL << gpio_num);
	pGPIOConfig.mode = GPIO_MODE_DEF_INPUT;
	pGPIOConfig.pull_up_en = GPIO_PULLUP_DISABLE;
	pGPIOConfig.pull_down_en = GPIO_PULLDOWN_DISABLE;
	pGPIOConfig.intr_type = isr_type;

	gpio_config(&pGPIOConfig);

	return ESP_OK;
}

/**************************************************************************
* Function: gpio_act_left_op
* Preconditions: gpio_context_t
* Overview: Funcion que activa las salidas del lado izquierdo del chip.
* Output: ESP_OK
*
*****************************************************************************/
esp_err_t gpio_act_left_op(void){
	gpio_hal_act_left_op(gpio_context.gpio_hal);
	esp_rom_gpio_connect_out_signal(12, SIG_GPIO_OUT_IDX, false, false);
	esp_rom_gpio_connect_out_signal(13, SIG_GPIO_OUT_IDX, false, false);
	esp_rom_gpio_connect_out_signal(14, SIG_GPIO_OUT_IDX, false, false);
	esp_rom_gpio_connect_out_signal(25, SIG_GPIO_OUT_IDX, false, false);
	esp_rom_gpio_connect_out_signal(26, SIG_GPIO_OUT_IDX, false, false);
	esp_rom_gpio_connect_out_signal(27, SIG_GPIO_OUT_IDX, false, false);
	esp_rom_gpio_connect_out_signal(32, SIG_GPIO_OUT_IDX, false, false);
	esp_rom_gpio_connect_out_signal(33, SIG_GPIO_OUT_IDX, false, false);
	esp_rom_gpio_connect_out_signal(34, SIG_GPIO_OUT_IDX, false, false);
	esp_rom_gpio_connect_out_signal(35, SIG_GPIO_OUT_IDX, false, false);
	esp_rom_gpio_connect_out_signal(36, SIG_GPIO_OUT_IDX, false, false);
	esp_rom_gpio_connect_out_signal(39, SIG_GPIO_OUT_IDX, false, false);
	return ESP_OK;
}
/**************************************************************************
* Function: gpio_deact_left_op
* Preconditions: gpio_context_t
* Overview: Funcion que desactiva los pines del lado izquierdo del chip como salidas.
* Output: ESP_OK
*
*****************************************************************************/
esp_err_t gpio_deact_left_op(void){
	gpio_hal_deact_left_op(gpio_context.gpio_hal);
	return ESP_OK;
}
/**************************************************************************
* Function: gpio_act_right_op
* Preconditions: gpio_context_t
* Overview: Funcion que activa los pines del lado derecho del chip como salida.
* Output: ESP_OK
*
*****************************************************************************/
esp_err_t gpio_act_right_op(void){
	gpio_hal_act_right_op(gpio_context.gpio_hal);
	esp_rom_gpio_connect_out_signal(1, SIG_GPIO_OUT_IDX, false, false);
	esp_rom_gpio_connect_out_signal(2, SIG_GPIO_OUT_IDX, false, false);
	esp_rom_gpio_connect_out_signal(3, SIG_GPIO_OUT_IDX, false, false);
	esp_rom_gpio_connect_out_signal(4, SIG_GPIO_OUT_IDX, false, false);
	esp_rom_gpio_connect_out_signal(5, SIG_GPIO_OUT_IDX, false, false);
	esp_rom_gpio_connect_out_signal(15, SIG_GPIO_OUT_IDX, false, false);
	esp_rom_gpio_connect_out_signal(16, SIG_GPIO_OUT_IDX, false, false);
	esp_rom_gpio_connect_out_signal(17, SIG_GPIO_OUT_IDX, false, false);
	esp_rom_gpio_connect_out_signal(18, SIG_GPIO_OUT_IDX, false, false);
	esp_rom_gpio_connect_out_signal(19, SIG_GPIO_OUT_IDX, false, false);
	esp_rom_gpio_connect_out_signal(21, SIG_GPIO_OUT_IDX, false, false);
	esp_rom_gpio_connect_out_signal(22, SIG_GPIO_OUT_IDX, false, false);
	esp_rom_gpio_connect_out_signal(23, SIG_GPIO_OUT_IDX, false, false);
	return ESP_OK;
}
/**************************************************************************
* Function: gpio_deact_right_op
* Preconditions: gpio_context_t
* Overview: Funcion que desactiva las los pines del lado derecho del chip como salidas..
* Output: ESP_OK
*
*****************************************************************************/
esp_err_t gpio_deact_right_op(void){
	gpio_hal_deact_right_op(gpio_context.gpio_hal);
	return ESP_OK;
}
/**************************************************************************
* Function: gpio_act_left_ip
* Preconditions: gpio_context_t
* Overview: Funcion que activa los pines del lado izquierdo del chip como entradas.
* Output: ESP_OK
*
*****************************************************************************/
esp_err_t gpio_act_left_ip(void){
	gpio_hal_act_left_ip(gpio_context.gpio_hal);
	return ESP_OK;
}
/**************************************************************************
* Function: gpio_deact_left_ip
* Preconditions: gpio_context_t
* Overview: Funcion que desactiva los pines del lado izquierdo del chip como entradas.
* Output: ESP_OK
*
*****************************************************************************/
esp_err_t gpio_deact_left_ip(void){
	gpio_hal_deact_left_ip(gpio_context.gpio_hal);
	return ESP_OK;
}
/**************************************************************************
* Function: gpio_act_right_ip
* Preconditions: gpio_context_t
* Overview: Funcion que activa los pines del lado derecho del chip como entradas.
* Output: ESP_OK
*
*****************************************************************************/
esp_err_t gpio_act_right_ip(void){
	gpio_hal_act_right_ip(gpio_context.gpio_hal);
	return ESP_OK;
}
/**************************************************************************
* Function: gpio_deact_right_ip
* Preconditions: gpio_context_t
* Overview: Funcion que desactiva los pines del lado derecho del chip como entradas.
* Output: ESP_OK
*
*****************************************************************************/
esp_err_t gpio_deact_right_ip(void){
	gpio_hal_deact_right_ip(gpio_context.gpio_hal);
	return ESP_OK;
}
/**************************************************************************
* Function: gpio_act_left_pulldown
* Preconditions: gpio_context_t
* Overview: Funcion que activa las resistencias de pulldown del lado izquierdo del chip.
* Output: ESP_OK
*
*****************************************************************************/
esp_err_t gpio_act_left_pulldown(void){
	portENTER_CRITICAL(&gpio_context.gpio_spinlock);
	gpio_hal_act_left_pulldown(gpio_context.gpio_hal);
	portEXIT_CRITICAL(&gpio_context.gpio_spinlock);
	return ESP_OK;
}
/**************************************************************************
* Function: gpio_deact_left_pulldown
* Preconditions: gpio_context_t
* Overview: Funcion que desactiva las resistencias de pulldown del lado izquierdo del chip.
* Output: ESP_OK
*
*****************************************************************************/
esp_err_t gpio_deact_left_pulldown(void){
	portENTER_CRITICAL(&gpio_context.gpio_spinlock);
	gpio_hal_deact_left_pulldown(gpio_context.gpio_hal);
	portEXIT_CRITICAL(&gpio_context.gpio_spinlock);
	return ESP_OK;
}
/**************************************************************************
* Function: gpio_act_right_pulldown
* Preconditions: gpio_context_t
* Overview: Funcion que activa las resistencias de pulldown del lado derecho del chip.
* Output: ESP_OK
*
*****************************************************************************/
esp_err_t gpio_act_right_pulldown(void){
	portENTER_CRITICAL(&gpio_context.gpio_spinlock);
	gpio_hal_act_right_pulldown(gpio_context.gpio_hal);
	portEXIT_CRITICAL(&gpio_context.gpio_spinlock);
	return ESP_OK;
}
/**************************************************************************
* Function: gpio_deact_right_pulldown
* Preconditions: gpio_context_t
* Overview: Funcion que desactiva las resistencias de pulldown del lado derecho del chip.
* Output: ESP_OK
*
*****************************************************************************/
esp_err_t gpio_deact_right_pulldown(void){
	portENTER_CRITICAL(&gpio_context.gpio_spinlock);
	gpio_hal_deact_right_pulldown(gpio_context.gpio_hal);
	portEXIT_CRITICAL(&gpio_context.gpio_spinlock);
	return ESP_OK;
}
/**************************************************************************
* Function: gpio_act_left_pullup
* Preconditions: gpio_context_t
* Overview: Funcion que activa las resistencias de pullup del lado izquierdo del chip.
* Output: ESP_OK
*
*****************************************************************************/
esp_err_t gpio_act_left_pullup(void){
	portENTER_CRITICAL(&gpio_context.gpio_spinlock);
	gpio_hal_act_left_pullup(gpio_context.gpio_hal);
	portEXIT_CRITICAL(&gpio_context.gpio_spinlock);
	return ESP_OK;
}
/**************************************************************************
* Function: gpio_deact_left_pullup
* Preconditions: gpio_context_t
* Overview: Funcion que desactiva las resistencias de pullup del lado izquierdo del chip.
* Output: ESP_OK
*
*****************************************************************************/
esp_err_t gpio_deact_left_pullup(void){
	portENTER_CRITICAL(&gpio_context.gpio_spinlock);
	gpio_hal_deact_left_pullup(gpio_context.gpio_hal);
	portEXIT_CRITICAL(&gpio_context.gpio_spinlock);
	return ESP_OK;
}
/**************************************************************************
* Function: gpio_act_right_pullup
* Preconditions: gpio_context_t
* Overview: Funcion que activa las resistencias de pullup del lado derecho del chip.
* Output: ESP_OK
*
*****************************************************************************/
esp_err_t gpio_act_right_pullup(void){
	portENTER_CRITICAL(&gpio_context.gpio_spinlock);
	gpio_hal_act_right_pullup(gpio_context.gpio_hal);
	portEXIT_CRITICAL(&gpio_context.gpio_spinlock);
	return ESP_OK;
}
/**************************************************************************
* Function: gpio_deact_right_pullup
* Preconditions: gpio_context_t
* Overview: Funcion que desactiva las resistencias de pullup del lado derecho del chip.
* Output: ESP_OK
*
*****************************************************************************/
esp_err_t gpio_deact_right_pullup(){
	portENTER_CRITICAL(&gpio_context.gpio_spinlock);
	gpio_hal_deact_right_pullup(gpio_context.gpio_hal);
	portEXIT_CRITICAL(&gpio_context.gpio_spinlock);
	return ESP_OK;
}
/**************************************************************************
* Function: gpio_reset_right
* Overview: Funcion que resetea los pines del lado derecho del chip a su estado default.
* Output: ESP_OK
*
*****************************************************************************/
esp_err_t gpio_reset_right(void){
	assert(1);
    gpio_config_t cfg_1 = {
        .pin_bit_mask = BIT64(1),
        .mode = GPIO_MODE_DISABLE,
        //Por razones de ahorro de energia, el GPIO no deberia estar flotando, se selecciona pullup
        .pull_up_en = true,
        .pull_down_en = false,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&cfg_1);
    gpio_config_t cfg_2 = {
        .pin_bit_mask = BIT64(2),
        .mode = GPIO_MODE_DISABLE,
		//Por razones de ahorro de energia, el GPIO no deberia estar flotando, se selecciona pullup
        .pull_up_en = true,
        .pull_down_en = false,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&cfg_2);
    gpio_config_t cfg_3 = {
        .pin_bit_mask = BIT64(3),
        .mode = GPIO_MODE_DISABLE,
		//Por razones de ahorro de energia, el GPIO no deberia estar flotando, se selecciona pullup
        .pull_up_en = true,
        .pull_down_en = false,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&cfg_3);
    gpio_config_t cfg_4 = {
        .pin_bit_mask = BIT64(4),
        .mode = GPIO_MODE_DISABLE,
		//Por razones de ahorro de energia, el GPIO no deberia estar flotando, se selecciona pullup
        .pull_up_en = true,
        .pull_down_en = false,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&cfg_4);
    gpio_config_t cfg_5 = {
        .pin_bit_mask = BIT64(5),
        .mode = GPIO_MODE_DISABLE,
		//Por razones de ahorro de energia, el GPIO no deberia estar flotando, se selecciona pullup
        .pull_up_en = true,
        .pull_down_en = false,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&cfg_5);
    gpio_config_t cfg_15 = {
        .pin_bit_mask = BIT64(15),
        .mode = GPIO_MODE_DISABLE,
		//Por razones de ahorro de energia, el GPIO no deberia estar flotando, se selecciona pullup
        .pull_up_en = true,
        .pull_down_en = false,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&cfg_15);
    gpio_config_t cfg_16 = {
        .pin_bit_mask = BIT64(16),
        .mode = GPIO_MODE_DISABLE,
		//Por razones de ahorro de energia, el GPIO no deberia estar flotando, se selecciona pullup
        .pull_up_en = true,
        .pull_down_en = false,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&cfg_16);
    gpio_config_t cfg_17 = {
        .pin_bit_mask = BIT64(17),
        .mode = GPIO_MODE_DISABLE,
		//Por razones de ahorro de energia, el GPIO no deberia estar flotando, se selecciona pullup
        .pull_up_en = true,
        .pull_down_en = false,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&cfg_17);
    gpio_config_t cfg_18 = {
        .pin_bit_mask = BIT64(18),
        .mode = GPIO_MODE_DISABLE,
		//Por razones de ahorro de energia, el GPIO no deberia estar flotando, se selecciona pullup
        .pull_up_en = true,
        .pull_down_en = false,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&cfg_18);
    gpio_config_t cfg_19 = {
        .pin_bit_mask = BIT64(19),
        .mode = GPIO_MODE_DISABLE,
		//Por razones de ahorro de energia, el GPIO no deberia estar flotando, se selecciona pullup
        .pull_up_en = true,
        .pull_down_en = false,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&cfg_19);
    gpio_config_t cfg_21 = {
        .pin_bit_mask = BIT64(21),
        .mode = GPIO_MODE_DISABLE,
		//Por razones de ahorro de energia, el GPIO no deberia estar flotando, se selecciona pullup
        .pull_up_en = true,
        .pull_down_en = false,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&cfg_21);
    gpio_config_t cfg_22 = {
        .pin_bit_mask = BIT64(22),
        .mode = GPIO_MODE_DISABLE,
		//Por razones de ahorro de energia, el GPIO no deberia estar flotando, se selecciona pullup
        .pull_up_en = true,
        .pull_down_en = false,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&cfg_22);
    gpio_config_t cfg_23 = {
        .pin_bit_mask = BIT64(23),
        .mode = GPIO_MODE_DISABLE,
		//Por razones de ahorro de energia, el GPIO no deberia estar flotando, se selecciona pullup
        .pull_up_en = true,
        .pull_down_en = false,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&cfg_23);
	return ESP_OK;
}
/**************************************************************************
* Function: gpio_reset_izquierdo
* Overview: Funcion que resetea los pines del lado izquierdo del chip a su estado default.
* Output: ESP_OK
*
*****************************************************************************/
esp_err_t gpio_reset_left(void){
	assert(1);
    gpio_config_t cfg_12 = {
        .pin_bit_mask = BIT64(12),
        .mode = GPIO_MODE_DISABLE,
		//Por razones de ahorro de energia, el GPIO no deberia estar flotando, se selecciona pullup
        .pull_up_en = true,
        .pull_down_en = false,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&cfg_12);
    gpio_config_t cfg_13 = {
        .pin_bit_mask = BIT64(13),
        .mode = GPIO_MODE_DISABLE,
		//Por razones de ahorro de energia, el GPIO no deberia estar flotando, se selecciona pullup
        .pull_up_en = true,
        .pull_down_en = false,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&cfg_13);
    gpio_config_t cfg_14 = {
        .pin_bit_mask = BIT64(14),
        .mode = GPIO_MODE_DISABLE,
		//Por razones de ahorro de energia, el GPIO no deberia estar flotando, se selecciona pullup
        .pull_up_en = true,
        .pull_down_en = false,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&cfg_14);
    gpio_config_t cfg_25 = {
        .pin_bit_mask = BIT64(25),
        .mode = GPIO_MODE_DISABLE,
		//Por razones de ahorro de energia, el GPIO no deberia estar flotando, se selecciona pullup
        .pull_up_en = true,
        .pull_down_en = false,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&cfg_25);
    gpio_config_t cfg_26 = {
        .pin_bit_mask = BIT64(26),
        .mode = GPIO_MODE_DISABLE,
		//Por razones de ahorro de energia, el GPIO no deberia estar flotando, se selecciona pullup
        .pull_up_en = true,
        .pull_down_en = false,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&cfg_26);
    gpio_config_t cfg_27 = {
        .pin_bit_mask = BIT64(27),
        .mode = GPIO_MODE_DISABLE,
		//Por razones de ahorro de energia, el GPIO no deberia estar flotando, se selecciona pullup
        .pull_up_en = true,
        .pull_down_en = false,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&cfg_27);
    gpio_config_t cfg_32 = {
        .pin_bit_mask = BIT64(32),
        .mode = GPIO_MODE_DISABLE,
		//Por razones de ahorro de energia, el GPIO no deberia estar flotando, se selecciona pullup
        .pull_up_en = true,
        .pull_down_en = false,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&cfg_32);
    gpio_config_t cfg_33 = {
        .pin_bit_mask = BIT64(33),
        .mode = GPIO_MODE_DISABLE,
		//Por razones de ahorro de energia, el GPIO no deberia estar flotando, se selecciona pullup
        .pull_up_en = true,
        .pull_down_en = false,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&cfg_33);
    gpio_config_t cfg_34 = {
        .pin_bit_mask = BIT64(34),
        .mode = GPIO_MODE_DISABLE,
		//Por razones de ahorro de energia, el GPIO no deberia estar flotando, se selecciona pullup
        .pull_up_en = true,
        .pull_down_en = false,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&cfg_34);
    gpio_config_t cfg_35 = {
        .pin_bit_mask = BIT64(35),
        .mode = GPIO_MODE_DISABLE,
		//Por razones de ahorro de energia, el GPIO no deberia estar flotando, se selecciona pullup
        .pull_up_en = true,
        .pull_down_en = false,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&cfg_35);
    gpio_config_t cfg_36 = {
        .pin_bit_mask = BIT64(36),
        .mode = GPIO_MODE_DISABLE,
		//Por razones de ahorro de energia, el GPIO no deberia estar flotando, se selecciona pullup
        .pull_up_en = true,
        .pull_down_en = false,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&cfg_36);
    gpio_config_t cfg_39 = {
        .pin_bit_mask = BIT64(39),
        .mode = GPIO_MODE_DISABLE,
		//Por razones de ahorro de energia, el GPIO no deberia estar flotando, se selecciona pullup
        .pull_up_en = true,
        .pull_down_en = false,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&cfg_39);
	return ESP_OK;
}
/**************************************************************************
* Function: Nombre de la función
* Preconditions: Qué funciones o declaraciones son previas al programa
* Overview: resumen del programa.
* Input: argumentos de entrada.
* Output: argumentos de salida
*
*****************************************************************************/
esp_err_t gpio_pullup_en(gpio_num_t gpio_num)
{
    GPIO_CHECK(GPIO_IS_VALID_GPIO(gpio_num), "GPIO number error", ESP_ERR_INVALID_ARG);

    if (!rtc_gpio_is_valid_gpio(gpio_num) || SOC_GPIO_SUPPORT_RTC_INDEPENDENT) {
        portENTER_CRITICAL(&gpio_context.gpio_spinlock);
        gpio_hal_pullup_act(gpio_context.gpio_hal, gpio_num);
        portEXIT_CRITICAL(&gpio_context.gpio_spinlock);
    } else {
#if SOC_RTCIO_INPUT_OUTPUT_SUPPORTED
        rtc_gpio_pullup_en(gpio_num);
#else
        abort(); // This should be eliminated as unreachable, unless a programming error has occured
#endif
    }

    return ESP_OK;
}
/**************************************************************************
* Function: Nombre de la función
* Preconditions: Qué funciones o declaraciones son previas al programa
* Overview: resumen del programa.
* Input: argumentos de entrada.
* Output: argumentos de salida
*
*****************************************************************************/
esp_err_t gpio_pullup_dis(gpio_num_t gpio_num)
{
    GPIO_CHECK(GPIO_IS_VALID_GPIO(gpio_num), "GPIO number error", ESP_ERR_INVALID_ARG);

    if (!rtc_gpio_is_valid_gpio(gpio_num) || SOC_GPIO_SUPPORT_RTC_INDEPENDENT) {
        portENTER_CRITICAL(&gpio_context.gpio_spinlock);
        gpio_hal_pullup_deact(gpio_context.gpio_hal, gpio_num);
        portEXIT_CRITICAL(&gpio_context.gpio_spinlock);
    } else {
#if SOC_RTCIO_INPUT_OUTPUT_SUPPORTED
        rtc_gpio_pullup_dis(gpio_num);
#else
        abort(); // This should be eliminated as unreachable, unless a programming error has occured
#endif
    }

    return ESP_OK;
}
/**************************************************************************
* Function: Nombre de la función
* Preconditions: Qué funciones o declaraciones son previas al programa
* Overview: resumen del programa.
* Input: argumentos de entrada.
* Output: argumentos de salida
*
*****************************************************************************/
esp_err_t gpio_pulldown_en(gpio_num_t gpio_num)
{
    GPIO_CHECK(GPIO_IS_VALID_GPIO(gpio_num), "GPIO number error", ESP_ERR_INVALID_ARG);

    if (!rtc_gpio_is_valid_gpio(gpio_num) || SOC_GPIO_SUPPORT_RTC_INDEPENDENT) {
        portENTER_CRITICAL(&gpio_context.gpio_spinlock);
        gpio_hal_pulldown_act(gpio_context.gpio_hal, gpio_num);
        portEXIT_CRITICAL(&gpio_context.gpio_spinlock);
    } else {
#if SOC_RTCIO_INPUT_OUTPUT_SUPPORTED
        rtc_gpio_pulldown_en(gpio_num);
#else
        abort(); // This should be eliminated as unreachable, unless a programming error has occured
#endif
    }

    return ESP_OK;
}

esp_err_t gpio_pulldown_dis(gpio_num_t gpio_num)
{
    GPIO_CHECK(GPIO_IS_VALID_GPIO(gpio_num), "GPIO number error", ESP_ERR_INVALID_ARG);

    if (!rtc_gpio_is_valid_gpio(gpio_num) || SOC_GPIO_SUPPORT_RTC_INDEPENDENT) {
        portENTER_CRITICAL(&gpio_context.gpio_spinlock);
        gpio_hal_pulldown_deact(gpio_context.gpio_hal, gpio_num);
        portEXIT_CRITICAL(&gpio_context.gpio_spinlock);
    } else {
#if SOC_RTCIO_INPUT_OUTPUT_SUPPORTED
        rtc_gpio_pulldown_dis(gpio_num);
#else
        abort(); // This should be eliminated as unreachable, unless a programming error has occured
#endif
    }

    return ESP_OK;
}
/**************************************************************************
* Function: Nombre de la función
* Preconditions: Qué funciones o declaraciones son previas al programa
* Overview: resumen del programa.
* Input: argumentos de entrada.
* Output: argumentos de salida
*
*****************************************************************************/
esp_err_t gpio_set_intr_type(gpio_num_t gpio_num, gpio_int_type_t intr_type)
{
    GPIO_CHECK(GPIO_IS_VALID_GPIO(gpio_num), "GPIO number error", ESP_ERR_INVALID_ARG);
    GPIO_CHECK(intr_type < GPIO_INTR_MAX, "GPIO interrupt type error", ESP_ERR_INVALID_ARG);

    portENTER_CRITICAL(&gpio_context.gpio_spinlock);
    gpio_hal_set_intr_type(gpio_context.gpio_hal, gpio_num, intr_type);
    portEXIT_CRITICAL(&gpio_context.gpio_spinlock);
    return ESP_OK;
}
/**************************************************************************
* Function: Nombre de la función
* Preconditions: Qué funciones o declaraciones son previas al programa
* Overview: resumen del programa.
* Input: argumentos de entrada.
* Output: argumentos de salida
*
*****************************************************************************/
static esp_err_t gpio_intr_enable_on_core(gpio_num_t gpio_num, uint32_t core_id)
{
    GPIO_CHECK(GPIO_IS_VALID_GPIO(gpio_num), "GPIO number error", ESP_ERR_INVALID_ARG);
    gpio_hal_intr_enable_on_core(gpio_context.gpio_hal, gpio_num, core_id);
    return ESP_OK;
}
/**************************************************************************
* Function: Nombre de la función
* Preconditions: Qué funciones o declaraciones son previas al programa
* Overview: resumen del programa.
* Input: argumentos de entrada.
* Output: argumentos de salida
*
*****************************************************************************/
esp_err_t gpio_intr_enable(gpio_num_t gpio_num)
{
    GPIO_CHECK(GPIO_IS_VALID_GPIO(gpio_num), "GPIO number error", ESP_ERR_INVALID_ARG);
    portENTER_CRITICAL(&gpio_context.gpio_spinlock);
    if(gpio_context.isr_core_id == GPIO_ISR_CORE_ID_UNINIT) {
        gpio_context.isr_core_id = xPortGetCoreID();
    }
    portEXIT_CRITICAL(&gpio_context.gpio_spinlock);
    return gpio_intr_enable_on_core (gpio_num, gpio_context.isr_core_id);
}
/**************************************************************************
* Function: Nombre de la función
* Preconditions: Qué funciones o declaraciones son previas al programa
* Overview: resumen del programa.
* Input: argumentos de entrada.
* Output: argumentos de salida
*
*****************************************************************************/
esp_err_t gpio_intr_disable(gpio_num_t gpio_num)
{
    GPIO_CHECK(GPIO_IS_VALID_GPIO(gpio_num), "GPIO number error", ESP_ERR_INVALID_ARG);
    gpio_hal_intr_disable(gpio_context.gpio_hal, gpio_num);
    return ESP_OK;
}
/**************************************************************************
* Function: Nombre de la función
* Preconditions: Qué funciones o declaraciones son previas al programa
* Overview: resumen del programa.
* Input: argumentos de entrada.
* Output: argumentos de salida
*
*****************************************************************************/
static esp_err_t gpio_input_disable(gpio_num_t gpio_num)
{
    GPIO_CHECK(GPIO_IS_VALID_GPIO(gpio_num), "GPIO number error", ESP_ERR_INVALID_ARG);
    gpio_hal_input_disable(gpio_context.gpio_hal, gpio_num);
    return ESP_OK;
}
/**************************************************************************
* Function: Nombre de la función
* Preconditions: Qué funciones o declaraciones son previas al programa
* Overview: resumen del programa.
* Input: argumentos de entrada.
* Output: argumentos de salida
*
*****************************************************************************/
static esp_err_t gpio_input_enable(gpio_num_t gpio_num)
{
    GPIO_CHECK(GPIO_IS_VALID_GPIO(gpio_num), "GPIO number error", ESP_ERR_INVALID_ARG);
    gpio_hal_input_enable(gpio_context.gpio_hal, gpio_num);
    return ESP_OK;
}

/**************************************************************************
* Function: Nombre de la función
* Preconditions: Qué funciones o declaraciones son previas al programa
* Overview: resumen del programa.
* Input: argumentos de entrada.
* Output: argumentos de salida
*
*****************************************************************************/
static esp_err_t gpio_od_disable(gpio_num_t gpio_num)
{
    GPIO_CHECK(GPIO_IS_VALID_GPIO(gpio_num), "GPIO number error", ESP_ERR_INVALID_ARG);
    gpio_hal_od_disable(gpio_context.gpio_hal, gpio_num);
    return ESP_OK;
}
/**************************************************************************
* Function: Nombre de la función
* Preconditions: Qué funciones o declaraciones son previas al programa
* Overview: resumen del programa.
* Input: argumentos de entrada.
* Output: argumentos de salida
*
*****************************************************************************/
static esp_err_t gpio_od_enable(gpio_num_t gpio_num)
{
    GPIO_CHECK(GPIO_IS_VALID_GPIO(gpio_num), "GPIO number error", ESP_ERR_INVALID_ARG);
    gpio_hal_od_enable(gpio_context.gpio_hal, gpio_num);
    return ESP_OK;
}
/**************************************************************************
* Function: Nombre de la función
* Preconditions: Qué funciones o declaraciones son previas al programa
* Overview: resumen del programa.
* Input: argumentos de entrada.
* Output: argumentos de salida
*
*****************************************************************************/
esp_err_t gpio_set_level(gpio_num_t gpio_num, uint32_t level)
{
    GPIO_CHECK(GPIO_IS_VALID_OUTPUT_GPIO(gpio_num), "GPIO output gpio_num error", ESP_ERR_INVALID_ARG);
    gpio_hal_set_level(gpio_context.gpio_hal, gpio_num, level);
    return ESP_OK;
}
/**************************************************************************
* Function: Nombre de la función
* Preconditions: Qué funciones o declaraciones son previas al programa
* Overview: resumen del programa.
* Input: argumentos de entrada.
* Output: argumentos de salida
*
*****************************************************************************/
int gpio_get_level(gpio_num_t gpio_num)
{
    return gpio_hal_get_level(gpio_context.gpio_hal, gpio_num);
}
/**************************************************************************
* Function: Nombre de la función
* Preconditions: Qué funciones o declaraciones son previas al programa
* Overview: resumen del programa.
* Input: argumentos de entrada.
* Output: argumentos de salida
*
*****************************************************************************/
esp_err_t gpio_set_pull_mode(gpio_num_t gpio_num, gpio_pull_mode_t pull)
{
    GPIO_CHECK(GPIO_IS_VALID_GPIO(gpio_num), "GPIO number error", ESP_ERR_INVALID_ARG);
    GPIO_CHECK(pull <= GPIO_FLOATING, "GPIO pull mode error", ESP_ERR_INVALID_ARG);
    esp_err_t ret = ESP_OK;

    switch (pull) {
        case GPIO_PULLUP_ONLY:
            gpio_pulldown_dis(gpio_num);
            gpio_pullup_en(gpio_num);
            break;

        case GPIO_PULLDOWN_ONLY:
            gpio_pulldown_en(gpio_num);
            gpio_pullup_dis(gpio_num);
            break;

        case GPIO_PULLUP_PULLDOWN:
            gpio_pulldown_en(gpio_num);
            gpio_pullup_en(gpio_num);
            break;

        case GPIO_FLOATING:
            gpio_pulldown_dis(gpio_num);
            gpio_pullup_dis(gpio_num);
            break;

        default:
            ESP_LOGE(GPIO_TAG, "Unknown pull up/down mode,gpio_num=%u,pull=%u", gpio_num, pull);
            ret = ESP_ERR_INVALID_ARG;
            break;
    }

    return ret;
}
/**************************************************************************
* Function: Nombre de la función
* Preconditions: Qué funciones o declaraciones son previas al programa
* Overview: resumen del programa.
* Input: argumentos de entrada.
* Output: argumentos de salida
*
*****************************************************************************/
esp_err_t gpio_set_direction(gpio_num_t gpio_num, gpio_mode_t mode)
{
    GPIO_CHECK(GPIO_IS_VALID_GPIO(gpio_num), "GPIO number error", ESP_ERR_INVALID_ARG);

    if ((GPIO_IS_VALID_OUTPUT_GPIO(gpio_num) != true) && (mode & GPIO_MODE_DEF_OUTPUT)) {
        ESP_LOGE(GPIO_TAG, "io_num=%d can only be input", gpio_num);
        return ESP_ERR_INVALID_ARG;
    }

    esp_err_t ret = ESP_OK;

    if (mode & GPIO_MODE_DEF_INPUT) {
        gpio_input_enable(gpio_num);
    } else {
        gpio_input_disable(gpio_num);
    }

    if (mode & GPIO_MODE_DEF_OUTPUT) {
        gpio_output_enable(gpio_num);
    } else {
        gpio_output_disable(gpio_num);
    }

    if (mode & GPIO_MODE_DEF_OD) {
        gpio_od_enable(gpio_num);
    } else {
        gpio_od_disable(gpio_num);
    }

    return ret;
}
/**************************************************************************
* Function: Nombre de la función
* Preconditions: Qué funciones o declaraciones son previas al programa
* Overview: resumen del programa.
* Input: argumentos de entrada.
* Output: argumentos de salida
*
*****************************************************************************/
esp_err_t gpio_config(const gpio_config_t *pGPIOConfig)
{
    uint64_t gpio_pin_mask = (pGPIOConfig->pin_bit_mask);
    uint32_t io_reg = 0;
    uint32_t io_num = 0;
    uint8_t input_en = 0;
    uint8_t output_en = 0;
    uint8_t od_en = 0;
    uint8_t pu_en = 0;
    uint8_t pd_en = 0;

    if (pGPIOConfig->pin_bit_mask == 0 ||
        (pGPIOConfig->pin_bit_mask & ~SOC_GPIO_VALID_GPIO_MASK)) {
        ESP_LOGE(GPIO_TAG, "GPIO_PIN mask error ");
        return ESP_ERR_INVALID_ARG;
    }

    if ((pGPIOConfig->mode & GPIO_MODE_DEF_OUTPUT) &&
        (pGPIOConfig->pin_bit_mask & ~SOC_GPIO_VALID_OUTPUT_GPIO_MASK)) {
        ESP_LOGE(GPIO_TAG, "GPIO can only be used as input mode");
        return ESP_ERR_INVALID_ARG;
    }

    do {
        io_reg = GPIO_PIN_MUX_REG[io_num];

        if (((gpio_pin_mask >> io_num) & BIT(0))) {
            assert(io_reg != (intptr_t)NULL);

#if SOC_RTCIO_INPUT_OUTPUT_SUPPORTED
            if (rtc_gpio_is_valid_gpio(io_num)) {
                rtc_gpio_deinit(io_num);
            }
#endif

            if ((pGPIOConfig->mode) & GPIO_MODE_DEF_INPUT) {
                input_en = 1;
                gpio_input_enable(io_num);
            } else {
                gpio_input_disable(io_num);
            }

            if ((pGPIOConfig->mode) & GPIO_MODE_DEF_OD) {
                od_en = 1;
                gpio_od_enable(io_num);
            } else {
                gpio_od_disable(io_num);
            }

            if ((pGPIOConfig->mode) & GPIO_MODE_DEF_OUTPUT) {
                output_en = 1;
                gpio_output_enable(io_num);
            } else {
                gpio_output_disable(io_num);
            }

            if (pGPIOConfig->pull_up_en) {
                pu_en = 1;
                gpio_pullup_en(io_num);
            } else {
                gpio_pullup_dis(io_num);
            }

            if (pGPIOConfig->pull_down_en) {
                pd_en = 1;
                gpio_pulldown_en(io_num);
            } else {
                gpio_pulldown_dis(io_num);
            }

            ESP_LOGI(GPIO_TAG, "GPIO[%d]| InputEn: %d| OutputEn: %d| OpenDrain: %d| Pullup: %d| Pulldown: %d| Intr:%d ", io_num, input_en, output_en, od_en, pu_en, pd_en, pGPIOConfig->intr_type);
            gpio_set_intr_type(io_num, pGPIOConfig->intr_type);

            if (pGPIOConfig->intr_type) {
                gpio_intr_enable(io_num);
            } else {
                gpio_intr_disable(io_num);
            }

            /* By default, all the pins have to be configured as GPIO pins. */
            gpio_hal_iomux_func_sel(io_reg, PIN_FUNC_GPIO);
        }

        io_num++;
    } while (io_num < GPIO_PIN_COUNT);

    return ESP_OK;
}
/**************************************************************************
* Function: Nombre de la función
* Preconditions: Qué funciones o declaraciones son previas al programa
* Overview: resumen del programa.
* Input: argumentos de entrada.
* Output: argumentos de salida
*
*****************************************************************************/
esp_err_t gpio_reset_pin(gpio_num_t gpio_num)
{
    assert(gpio_num >= 0 && GPIO_IS_VALID_GPIO(gpio_num));
    gpio_config_t cfg = {
        .pin_bit_mask = BIT64(gpio_num),
        .mode = GPIO_MODE_DISABLE,
        //for powersave reasons, the GPIO should not be floating, select pullup
        .pull_up_en = true,
        .pull_down_en = false,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&cfg);
    return ESP_OK;
}
/**************************************************************************
* Function: Nombre de la función
* Preconditions: Qué funciones o declaraciones son previas al programa
* Overview: resumen del programa.
* Input: argumentos de entrada.
* Output: argumentos de salida
*
*****************************************************************************/
static inline void IRAM_ATTR gpio_isr_loop(uint32_t status, const uint32_t gpio_num_start)
{
    while (status) {
        int nbit = __builtin_ffs(status) - 1;
        status &= ~(1 << nbit);
        int gpio_num = gpio_num_start + nbit;

        if (gpio_context.gpio_isr_func[gpio_num].fn != NULL) {
            gpio_context.gpio_isr_func[gpio_num].fn(gpio_context.gpio_isr_func[gpio_num].args);
        }
    }
}
/**************************************************************************
* Function: Nombre de la función
* Preconditions: Qué funciones o declaraciones son previas al programa
* Overview: resumen del programa.
* Input: argumentos de entrada.
* Output: argumentos de salida
*
*****************************************************************************/
static void IRAM_ATTR gpio_intr_service(void *arg)
{
    //GPIO intr process
    if (gpio_context.gpio_isr_func == NULL) {
        return;
    }

    //read status to get interrupt status for GPIO0-31
    uint32_t gpio_intr_status;
    gpio_hal_get_intr_status(gpio_context.gpio_hal, gpio_context.isr_core_id, &gpio_intr_status);

    if (gpio_intr_status) {
        gpio_isr_loop(gpio_intr_status, 0);
        gpio_hal_clear_intr_status(gpio_context.gpio_hal, gpio_intr_status);
    }

    //read status1 to get interrupt status for GPIO32-39
    uint32_t gpio_intr_status_h;
    gpio_hal_get_intr_status_high(gpio_context.gpio_hal, gpio_context.isr_core_id, &gpio_intr_status_h);

    if (gpio_intr_status_h) {
        gpio_isr_loop(gpio_intr_status_h, 32);
        gpio_hal_clear_intr_status_high(gpio_context.gpio_hal, gpio_intr_status_h);
    }
}
/**************************************************************************
* Function: Nombre de la función
* Preconditions: Qué funciones o declaraciones son previas al programa
* Overview: resumen del programa.
* Input: argumentos de entrada.
* Output: argumentos de salida
*
*****************************************************************************/
esp_err_t gpio_install_isr_service(int intr_alloc_flags)
{
    GPIO_CHECK(gpio_context.gpio_isr_func == NULL, "GPIO isr service already installed", ESP_ERR_INVALID_STATE);
    esp_err_t ret;
    portENTER_CRITICAL(&gpio_context.gpio_spinlock);
    gpio_context.gpio_isr_func = (gpio_isr_func_t *) calloc(GPIO_NUM_MAX, sizeof(gpio_isr_func_t));
    portEXIT_CRITICAL(&gpio_context.gpio_spinlock);
    if (gpio_context.gpio_isr_func == NULL) {
        ret = ESP_ERR_NO_MEM;
    } else {
        ret = gpio_isr_register(gpio_intr_service, NULL, intr_alloc_flags, &gpio_context.gpio_isr_handle);
    }

    return ret;
}
/**************************************************************************
* Function: Nombre de la función
* Preconditions: Qué funciones o declaraciones son previas al programa
* Overview: resumen del programa.
* Input: argumentos de entrada.
* Output: argumentos de salida
*
*****************************************************************************/
esp_err_t gpio_isr_handler_add(gpio_num_t gpio_num, gpio_isr_t isr_handler, void *args)
{
    GPIO_CHECK(gpio_context.gpio_isr_func != NULL, "GPIO isr service is not installed, call gpio_install_isr_service() first", ESP_ERR_INVALID_STATE);
    GPIO_CHECK(GPIO_IS_VALID_GPIO(gpio_num), "GPIO number error", ESP_ERR_INVALID_ARG);
    portENTER_CRITICAL(&gpio_context.gpio_spinlock);
    gpio_intr_disable(gpio_num);
    if (gpio_context.gpio_isr_func) {
        gpio_context.gpio_isr_func[gpio_num].fn = isr_handler;
        gpio_context.gpio_isr_func[gpio_num].args = args;
    }
    gpio_intr_enable_on_core (gpio_num, esp_intr_get_cpu(gpio_context.gpio_isr_handle));
    portEXIT_CRITICAL(&gpio_context.gpio_spinlock);
    return ESP_OK;
}
/**************************************************************************
* Function: Nombre de la función
* Preconditions: Qué funciones o declaraciones son previas al programa
* Overview: resumen del programa.
* Input: argumentos de entrada.
* Output: argumentos de salida
*
*****************************************************************************/
esp_err_t gpio_isr_handler_remove(gpio_num_t gpio_num)
{
    GPIO_CHECK(gpio_context.gpio_isr_func != NULL, "GPIO isr service is not installed, call gpio_install_isr_service() first", ESP_ERR_INVALID_STATE);
    GPIO_CHECK(GPIO_IS_VALID_GPIO(gpio_num), "GPIO number error", ESP_ERR_INVALID_ARG);
    portENTER_CRITICAL(&gpio_context.gpio_spinlock);
    gpio_intr_disable(gpio_num);
    if (gpio_context.gpio_isr_func) {
        gpio_context.gpio_isr_func[gpio_num].fn = NULL;
        gpio_context.gpio_isr_func[gpio_num].args = NULL;
    }
    portEXIT_CRITICAL(&gpio_context.gpio_spinlock);
    return ESP_OK;
}
/**************************************************************************
* Function: Nombre de la función
* Preconditions: Qué funciones o declaraciones son previas al programa
* Overview: resumen del programa.
* Input: argumentos de entrada.
* Output: argumentos de salida
*
*****************************************************************************/
void gpio_uninstall_isr_service(void)
{
    gpio_isr_func_t *gpio_isr_func_free = NULL;
    gpio_isr_handle_t gpio_isr_handle_free = NULL;
    portENTER_CRITICAL(&gpio_context.gpio_spinlock);
    if (gpio_context.gpio_isr_func == NULL) {
        portEXIT_CRITICAL(&gpio_context.gpio_spinlock);
        return;
    }
    gpio_isr_func_free = gpio_context.gpio_isr_func;
    gpio_context.gpio_isr_func = NULL;
    gpio_isr_handle_free = gpio_context.gpio_isr_handle;
    gpio_context.gpio_isr_handle = NULL;
    gpio_context.isr_core_id = GPIO_ISR_CORE_ID_UNINIT;
    portEXIT_CRITICAL(&gpio_context.gpio_spinlock);
    esp_intr_free(gpio_isr_handle_free);
    free(gpio_isr_func_free);
    return;
}

/**************************************************************************
* Function: Nombre de la función
* Preconditions: Qué funciones o declaraciones son previas al programa
* Overview: resumen del programa.
* Input: argumentos de entrada.
* Output: argumentos de salida
*
*****************************************************************************/
static void gpio_isr_register_on_core_static(void *param)
{
    gpio_isr_alloc_t *p = (gpio_isr_alloc_t *)param;
    //We need to check the return value.
    p->ret = esp_intr_alloc(p->source, p->intr_alloc_flags, p->fn, p->arg, p->handle);
}
/**************************************************************************
* Function: Nombre de la función
* Preconditions: Qué funciones o declaraciones son previas al programa
* Overview: resumen del programa.
* Input: argumentos de entrada.
* Output: argumentos de salida
*
*****************************************************************************/
esp_err_t gpio_isr_register(void (*fn)(void *), void *arg, int intr_alloc_flags, gpio_isr_handle_t *handle)
{
    GPIO_CHECK(fn, "GPIO ISR null", ESP_ERR_INVALID_ARG);
    gpio_isr_alloc_t p;
    p.source = ETS_GPIO_INTR_SOURCE;
    p.intr_alloc_flags = intr_alloc_flags;
    p.fn = fn;
    p.arg = arg;
    p.handle = handle;
    portENTER_CRITICAL(&gpio_context.gpio_spinlock);
    if(gpio_context.isr_core_id == GPIO_ISR_CORE_ID_UNINIT) {
        gpio_context.isr_core_id = xPortGetCoreID();
    }
    portEXIT_CRITICAL(&gpio_context.gpio_spinlock);
    esp_err_t ret;
#if CONFIG_FREERTOS_UNICORE
    gpio_isr_register_on_core_static(&p);
    ret = ESP_OK;
#else /* CONFIG_FREERTOS_UNICORE */
    ret = esp_ipc_call_blocking(gpio_context.isr_core_id, gpio_isr_register_on_core_static, (void *)&p);
#endif /* !CONFIG_FREERTOS_UNICORE */
    if (ret != ESP_OK) {
        ESP_LOGE(GPIO_TAG, "esp_ipc_call_blocking failed (0x%x)", ret);
        return ESP_ERR_NOT_FOUND;
    }
    if (p.ret != ESP_OK) {
        ESP_LOGE(GPIO_TAG, "esp_intr_alloc failed (0x%x)", p.ret);
        return ESP_ERR_NOT_FOUND;
    }
    return ESP_OK;
}
/**************************************************************************
* Function: Nombre de la función
* Preconditions: Qué funciones o declaraciones son previas al programa
* Overview: resumen del programa.
* Input: argumentos de entrada.
* Output: argumentos de salida
*
*****************************************************************************/
esp_err_t gpio_wakeup_enable(gpio_num_t gpio_num, gpio_int_type_t intr_type)
{
    GPIO_CHECK(GPIO_IS_VALID_GPIO(gpio_num), "GPIO number error", ESP_ERR_INVALID_ARG);
    esp_err_t ret = ESP_OK;

    if ((intr_type == GPIO_INTR_LOW_LEVEL) || (intr_type == GPIO_INTR_HIGH_LEVEL)) {
#if SOC_RTCIO_WAKE_SUPPORTED
        if (rtc_gpio_is_valid_gpio(gpio_num)) {
            ret = rtc_gpio_wakeup_enable(gpio_num, intr_type);
        }
#endif
        portENTER_CRITICAL(&gpio_context.gpio_spinlock);
        gpio_hal_wakeup_enable(gpio_context.gpio_hal, gpio_num, intr_type);
#if SOC_GPIO_SUPPORT_SLP_SWITCH && CONFIG_ESP32C3_LIGHTSLEEP_GPIO_RESET_WORKAROUND
        gpio_hal_sleep_sel_dis(gpio_context.gpio_hal, gpio_num);
#endif
        portEXIT_CRITICAL(&gpio_context.gpio_spinlock);
    } else {
        ESP_LOGE(GPIO_TAG, "GPIO wakeup only supports level mode, but edge mode set. gpio_num:%u", gpio_num);
        ret = ESP_ERR_INVALID_ARG;
    }

    return ret;
}
/**************************************************************************
* Function: Nombre de la función
* Preconditions: Qué funciones o declaraciones son previas al programa
* Overview: resumen del programa.
* Input: argumentos de entrada.
* Output: argumentos de salida
*
*****************************************************************************/
esp_err_t gpio_wakeup_disable(gpio_num_t gpio_num)
{
    GPIO_CHECK(GPIO_IS_VALID_GPIO(gpio_num), "GPIO number error", ESP_ERR_INVALID_ARG);
    esp_err_t ret = ESP_OK;
#if SOC_RTCIO_WAKE_SUPPORTED
    if (rtc_gpio_is_valid_gpio(gpio_num)) {
        ret = rtc_gpio_wakeup_disable(gpio_num);
    }
#endif
    portENTER_CRITICAL(&gpio_context.gpio_spinlock);
    gpio_hal_wakeup_disable(gpio_context.gpio_hal, gpio_num);
#if SOC_GPIO_SUPPORT_SLP_SWITCH && CONFIG_ESP32C3_LIGHTSLEEP_GPIO_RESET_WORKAROUND
    gpio_hal_sleep_sel_en(gpio_context.gpio_hal, gpio_num);
#endif
    portEXIT_CRITICAL(&gpio_context.gpio_spinlock);
    return ret;
}
/**************************************************************************
* Function: Nombre de la función
* Preconditions: Qué funciones o declaraciones son previas al programa
* Overview: resumen del programa.
* Input: argumentos de entrada.
* Output: argumentos de salida
*
*****************************************************************************/
esp_err_t gpio_set_drive_capability(gpio_num_t gpio_num, gpio_drive_cap_t strength)
{
    GPIO_CHECK(GPIO_IS_VALID_OUTPUT_GPIO(gpio_num), "GPIO number error", ESP_ERR_INVALID_ARG);
    GPIO_CHECK(strength < GPIO_DRIVE_CAP_MAX, "GPIO drive capability error", ESP_ERR_INVALID_ARG);
    esp_err_t ret = ESP_OK;

    if (!rtc_gpio_is_valid_gpio(gpio_num) || SOC_GPIO_SUPPORT_RTC_INDEPENDENT) {
        portENTER_CRITICAL(&gpio_context.gpio_spinlock);
        gpio_hal_set_drive_capability(gpio_context.gpio_hal, gpio_num, strength);
        portEXIT_CRITICAL(&gpio_context.gpio_spinlock);
    } else {
#if SOC_RTCIO_INPUT_OUTPUT_SUPPORTED
        ret = rtc_gpio_set_drive_capability(gpio_num, strength);
#else
        abort(); // This should be eliminated as unreachable, unless a programming error has occured
#endif
    }

    return ret;
}
/**************************************************************************
* Function: Nombre de la función
* Preconditions: Qué funciones o declaraciones son previas al programa
* Overview: resumen del programa.
* Input: argumentos de entrada.
* Output: argumentos de salida
*
*****************************************************************************/
esp_err_t gpio_get_drive_capability(gpio_num_t gpio_num, gpio_drive_cap_t *strength)
{
    GPIO_CHECK(GPIO_IS_VALID_OUTPUT_GPIO(gpio_num), "GPIO number error", ESP_ERR_INVALID_ARG);
    GPIO_CHECK(strength != NULL, "GPIO drive capability pointer error", ESP_ERR_INVALID_ARG);
    esp_err_t ret = ESP_OK;

    if (!rtc_gpio_is_valid_gpio(gpio_num) || SOC_GPIO_SUPPORT_RTC_INDEPENDENT) {
        portENTER_CRITICAL(&gpio_context.gpio_spinlock);
        gpio_hal_get_drive_capability(gpio_context.gpio_hal, gpio_num, strength);
        portEXIT_CRITICAL(&gpio_context.gpio_spinlock);
    } else {
#if SOC_RTCIO_INPUT_OUTPUT_SUPPORTED
        ret = rtc_gpio_get_drive_capability(gpio_num, strength);
#else
        abort(); // This should be eliminated as unreachable, unless a programming error has occured
#endif
    }

    return ret;
}
/**************************************************************************
* Function: Nombre de la función
* Preconditions: Qué funciones o declaraciones son previas al programa
* Overview: resumen del programa.
* Input: argumentos de entrada.
* Output: argumentos de salida
*
*****************************************************************************/
esp_err_t gpio_hold_en(gpio_num_t gpio_num)
{
    GPIO_CHECK(GPIO_IS_VALID_OUTPUT_GPIO(gpio_num), "Only output-capable GPIO support this function", ESP_ERR_NOT_SUPPORTED);
    int ret = ESP_OK;

    if (rtc_gpio_is_valid_gpio(gpio_num)) {
#if SOC_RTCIO_HOLD_SUPPORTED
        ret = rtc_gpio_hold_en(gpio_num);
#endif
    } else if (GPIO_HOLD_MASK[gpio_num]) {
        portENTER_CRITICAL(&gpio_context.gpio_spinlock);
        gpio_hal_hold_en(gpio_context.gpio_hal, gpio_num);
        portEXIT_CRITICAL(&gpio_context.gpio_spinlock);
    } else {
        ret = ESP_ERR_NOT_SUPPORTED;
    }

    return ret;
}
/**************************************************************************
* Function: Nombre de la función
* Preconditions: Qué funciones o declaraciones son previas al programa
* Overview: resumen del programa.
* Input: argumentos de entrada.
* Output: argumentos de salida
*
*****************************************************************************/
esp_err_t gpio_hold_dis(gpio_num_t gpio_num)
{
    GPIO_CHECK(GPIO_IS_VALID_OUTPUT_GPIO(gpio_num), "Only output-capable GPIO support this function", ESP_ERR_NOT_SUPPORTED);
    int ret = ESP_OK;

    if (rtc_gpio_is_valid_gpio(gpio_num)) {
#if SOC_RTCIO_HOLD_SUPPORTED
        ret = rtc_gpio_hold_dis(gpio_num);
#endif
    }else if (GPIO_HOLD_MASK[gpio_num]) {
        portENTER_CRITICAL(&gpio_context.gpio_spinlock);
        gpio_hal_hold_dis(gpio_context.gpio_hal, gpio_num);
        portEXIT_CRITICAL(&gpio_context.gpio_spinlock);
    } else {
        ret = ESP_ERR_NOT_SUPPORTED;
    }

    return ret;
}
/**************************************************************************
* Function: Nombre de la función
* Preconditions: Qué funciones o declaraciones son previas al programa
* Overview: resumen del programa.
* Input: argumentos de entrada.
* Output: argumentos de salida
*
*****************************************************************************/
void gpio_deep_sleep_hold_en(void)
{
    portENTER_CRITICAL(&gpio_context.gpio_spinlock);
    gpio_hal_deep_sleep_hold_en(gpio_context.gpio_hal);
    portEXIT_CRITICAL(&gpio_context.gpio_spinlock);
}
/**************************************************************************
* Function: Nombre de la función
* Preconditions: Qué funciones o declaraciones son previas al programa
* Overview: resumen del programa.
* Input: argumentos de entrada.
* Output: argumentos de salida
*
*****************************************************************************/
void gpio_deep_sleep_hold_dis(void)
{
    portENTER_CRITICAL(&gpio_context.gpio_spinlock);
    gpio_hal_deep_sleep_hold_dis(gpio_context.gpio_hal);
    portEXIT_CRITICAL(&gpio_context.gpio_spinlock);
}

#if SOC_GPIO_SUPPORT_FORCE_HOLD

esp_err_t gpio_force_hold_all()
{
#if SOC_RTCIO_HOLD_SUPPORTED
    rtc_gpio_force_hold_all();
#endif
    portENTER_CRITICAL(&gpio_context.gpio_spinlock);
    gpio_hal_force_hold_all(gpio_context.gpio_hal);
    portEXIT_CRITICAL(&gpio_context.gpio_spinlock);
    return ESP_OK;
}

esp_err_t gpio_force_unhold_all()
{
#if SOC_RTCIO_HOLD_SUPPORTED
    rtc_gpio_force_hold_dis_all();
#endif
    portENTER_CRITICAL(&gpio_context.gpio_spinlock);
    gpio_hal_force_unhold_all();
    portEXIT_CRITICAL(&gpio_context.gpio_spinlock);
    return ESP_OK;
}
#endif
/**************************************************************************
* Function: Nombre de la función
* Preconditions: Qué funciones o declaraciones son previas al programa
* Overview: resumen del programa.
* Input: argumentos de entrada.
* Output: argumentos de salida
*
*****************************************************************************/
void gpio_iomux_in(uint32_t gpio, uint32_t signal_idx)
{
    gpio_hal_iomux_in(gpio_context.gpio_hal, gpio, signal_idx);
}
/**************************************************************************
* Function: Nombre de la función
* Preconditions: Qué funciones o declaraciones son previas al programa
* Overview: resumen del programa.
* Input: argumentos de entrada.
* Output: argumentos de salida
*
*****************************************************************************/
void gpio_iomux_out(uint8_t gpio_num, int func, bool oen_inv)
{
    gpio_hal_iomux_out(gpio_context.gpio_hal, gpio_num, func, (uint32_t)oen_inv);
}
/**************************************************************************
* Function: Nombre de la función
* Preconditions: Qué funciones o declaraciones son previas al programa
* Overview: resumen del programa.
* Input: argumentos de entrada.
* Output: argumentos de salida
*
*****************************************************************************/
#if SOC_GPIO_SUPPORT_SLP_SWITCH
static esp_err_t gpio_sleep_pullup_en(gpio_num_t gpio_num)
{
    GPIO_CHECK(GPIO_IS_VALID_GPIO(gpio_num), "GPIO number error", ESP_ERR_INVALID_ARG);

    portENTER_CRITICAL(&gpio_context.gpio_spinlock);
    gpio_hal_sleep_pullup_en(gpio_context.gpio_hal, gpio_num);
    portEXIT_CRITICAL(&gpio_context.gpio_spinlock);

    return ESP_OK;
}
/**************************************************************************
* Function: Nombre de la función
* Preconditions: Qué funciones o declaraciones son previas al programa
* Overview: resumen del programa.
* Input: argumentos de entrada.
* Output: argumentos de salida
*
*****************************************************************************/
static esp_err_t gpio_sleep_pullup_dis(gpio_num_t gpio_num)
{
    GPIO_CHECK(GPIO_IS_VALID_GPIO(gpio_num), "GPIO number error", ESP_ERR_INVALID_ARG);

    portENTER_CRITICAL(&gpio_context.gpio_spinlock);
    gpio_hal_sleep_pullup_dis(gpio_context.gpio_hal, gpio_num);
    portEXIT_CRITICAL(&gpio_context.gpio_spinlock);

    return ESP_OK;
}
/**************************************************************************
* Function: Nombre de la función
* Preconditions: Qué funciones o declaraciones son previas al programa
* Overview: resumen del programa.
* Input: argumentos de entrada.
* Output: argumentos de salida
*
*****************************************************************************/
static esp_err_t gpio_sleep_pulldown_en(gpio_num_t gpio_num)
{
    GPIO_CHECK(GPIO_IS_VALID_GPIO(gpio_num), "GPIO number error", ESP_ERR_INVALID_ARG);

    portENTER_CRITICAL(&gpio_context.gpio_spinlock);
    gpio_hal_sleep_pulldown_en(gpio_context.gpio_hal, gpio_num);
    portEXIT_CRITICAL(&gpio_context.gpio_spinlock);

    return ESP_OK;
}
/**************************************************************************
* Function: Nombre de la función
* Preconditions: Qué funciones o declaraciones son previas al programa
* Overview: resumen del programa.
* Input: argumentos de entrada.
* Output: argumentos de salida
*
*****************************************************************************/
static esp_err_t gpio_sleep_pulldown_dis(gpio_num_t gpio_num)
{
    GPIO_CHECK(GPIO_IS_VALID_GPIO(gpio_num), "GPIO number error", ESP_ERR_INVALID_ARG);

    portENTER_CRITICAL(&gpio_context.gpio_spinlock);
    gpio_hal_sleep_pulldown_dis(gpio_context.gpio_hal, gpio_num);
    portEXIT_CRITICAL(&gpio_context.gpio_spinlock);

    return ESP_OK;
}
/**************************************************************************
* Function: Nombre de la función
* Preconditions: Qué funciones o declaraciones son previas al programa
* Overview: resumen del programa.
* Input: argumentos de entrada.
* Output: argumentos de salida
*
*****************************************************************************/
static esp_err_t gpio_sleep_input_disable(gpio_num_t gpio_num)
{
    GPIO_CHECK(GPIO_IS_VALID_GPIO(gpio_num), "GPIO number error", ESP_ERR_INVALID_ARG);
    gpio_hal_sleep_input_disable(gpio_context.gpio_hal, gpio_num);
    return ESP_OK;
}
/**************************************************************************
* Function: Nombre de la función
* Preconditions: Qué funciones o declaraciones son previas al programa
* Overview: resumen del programa.
* Input: argumentos de entrada.
* Output: argumentos de salida
*
*****************************************************************************/
static esp_err_t gpio_sleep_input_enable(gpio_num_t gpio_num)
{
    GPIO_CHECK(GPIO_IS_VALID_GPIO(gpio_num), "GPIO number error", ESP_ERR_INVALID_ARG);
    gpio_hal_sleep_input_enable(gpio_context.gpio_hal, gpio_num);
    return ESP_OK;
}
/**************************************************************************
* Function: Nombre de la función
* Preconditions: Qué funciones o declaraciones son previas al programa
* Overview: resumen del programa.
* Input: argumentos de entrada.
* Output: argumentos de salida
*
*****************************************************************************/
static esp_err_t gpio_sleep_output_disable(gpio_num_t gpio_num)
{
    GPIO_CHECK(GPIO_IS_VALID_GPIO(gpio_num), "GPIO number error", ESP_ERR_INVALID_ARG);
    gpio_hal_sleep_output_disable(gpio_context.gpio_hal, gpio_num);
    return ESP_OK;
}
/**************************************************************************
* Function: Nombre de la función
* Preconditions: Qué funciones o declaraciones son previas al programa
* Overview: resumen del programa.
* Input: argumentos de entrada.
* Output: argumentos de salida
*
*****************************************************************************/
static esp_err_t gpio_sleep_output_enable(gpio_num_t gpio_num)
{
    GPIO_CHECK(GPIO_IS_VALID_OUTPUT_GPIO(gpio_num), "GPIO output gpio_num error", ESP_ERR_INVALID_ARG);
    gpio_hal_sleep_output_enable(gpio_context.gpio_hal, gpio_num);
    return ESP_OK;
}
/**************************************************************************
* Function: Nombre de la función
* Preconditions: Qué funciones o declaraciones son previas al programa
* Overview: resumen del programa.
* Input: argumentos de entrada.
* Output: argumentos de salida
*
*****************************************************************************/
esp_err_t gpio_sleep_set_direction(gpio_num_t gpio_num, gpio_mode_t mode)
{
    GPIO_CHECK(GPIO_IS_VALID_GPIO(gpio_num), "GPIO number error", ESP_ERR_INVALID_ARG);

    if ((GPIO_IS_VALID_OUTPUT_GPIO(gpio_num) != true) && (mode & GPIO_MODE_DEF_OUTPUT)) {
        ESP_LOGE(GPIO_TAG, "io_num=%d can only be input", gpio_num);
        return ESP_ERR_INVALID_ARG;
    }

    esp_err_t ret = ESP_OK;

    if (mode & GPIO_MODE_DEF_INPUT) {
        gpio_sleep_input_enable(gpio_num);
    } else {
        gpio_sleep_input_disable(gpio_num);
    }

    if (mode & GPIO_MODE_DEF_OUTPUT) {
        gpio_sleep_output_enable(gpio_num);
    } else {
        gpio_sleep_output_disable(gpio_num);
    }

    return ret;
}
/**************************************************************************
* Function: Nombre de la función
* Preconditions: Qué funciones o declaraciones son previas al programa
* Overview: resumen del programa.
* Input: argumentos de entrada.
* Output: argumentos de salida
*
*****************************************************************************/
esp_err_t gpio_sleep_set_pull_mode(gpio_num_t gpio_num, gpio_pull_mode_t pull)
{
    GPIO_CHECK(GPIO_IS_VALID_GPIO(gpio_num), "GPIO number error", ESP_ERR_INVALID_ARG);
    GPIO_CHECK(pull <= GPIO_FLOATING, "GPIO pull mode error", ESP_ERR_INVALID_ARG);
    esp_err_t ret = ESP_OK;

    switch (pull) {
        case GPIO_PULLUP_ONLY:
            gpio_sleep_pulldown_dis(gpio_num);
            gpio_sleep_pullup_en(gpio_num);
            break;

        case GPIO_PULLDOWN_ONLY:
            gpio_sleep_pulldown_en(gpio_num);
            gpio_sleep_pullup_dis(gpio_num);
            break;

        case GPIO_PULLUP_PULLDOWN:
            gpio_sleep_pulldown_en(gpio_num);
            gpio_sleep_pullup_en(gpio_num);
            break;

        case GPIO_FLOATING:
            gpio_sleep_pulldown_dis(gpio_num);
            gpio_sleep_pullup_dis(gpio_num);
            break;

        default:
            ESP_LOGE(GPIO_TAG, "Unknown pull up/down mode,gpio_num=%u,pull=%u", gpio_num, pull);
            ret = ESP_ERR_INVALID_ARG;
            break;
    }

    return ret;
}
/**************************************************************************
* Function: Nombre de la función
* Preconditions: Qué funciones o declaraciones son previas al programa
* Overview: resumen del programa.
* Input: argumentos de entrada.
* Output: argumentos de salida
*
*****************************************************************************/
esp_err_t gpio_sleep_sel_en(gpio_num_t gpio_num)
{
    GPIO_CHECK(GPIO_IS_VALID_GPIO(gpio_num), "GPIO number error", ESP_ERR_INVALID_ARG);

    portENTER_CRITICAL(&gpio_context.gpio_spinlock);
    gpio_hal_sleep_sel_en(gpio_context.gpio_hal, gpio_num);
    portEXIT_CRITICAL(&gpio_context.gpio_spinlock);

    return ESP_OK;
}
/**************************************************************************
* Function: Nombre de la función
* Preconditions: Qué funciones o declaraciones son previas al programa
* Overview: resumen del programa.
* Input: argumentos de entrada.
* Output: argumentos de salida
*
*****************************************************************************/
esp_err_t gpio_sleep_sel_dis(gpio_num_t gpio_num)
{
    GPIO_CHECK(GPIO_IS_VALID_GPIO(gpio_num), "GPIO number error", ESP_ERR_INVALID_ARG);

    portENTER_CRITICAL(&gpio_context.gpio_spinlock);
    gpio_hal_sleep_sel_dis(gpio_context.gpio_hal, gpio_num);
    portEXIT_CRITICAL(&gpio_context.gpio_spinlock);

    return ESP_OK;
}

#if CONFIG_GPIO_ESP32_SUPPORT_SWITCH_SLP_PULL
esp_err_t gpio_sleep_pupd_config_apply(gpio_num_t gpio_num)
{
    GPIO_CHECK(GPIO_IS_VALID_GPIO(gpio_num), "GPIO number error", ESP_ERR_INVALID_ARG);
    gpio_hal_sleep_pupd_config_apply(gpio_context.gpio_hal, gpio_num);
    return ESP_OK;
}

esp_err_t gpio_sleep_pupd_config_unapply(gpio_num_t gpio_num)
{
    GPIO_CHECK(GPIO_IS_VALID_GPIO(gpio_num), "GPIO number error", ESP_ERR_INVALID_ARG);
    gpio_hal_sleep_pupd_config_unapply(gpio_context.gpio_hal, gpio_num);
    return ESP_OK;
}
#endif // CONFIG_GPIO_ESP32_SUPPORT_SWITCH_SLP_PULL
#endif // SOC_GPIO_SUPPORT_SLP_SWITCH

#if SOC_GPIO_SUPPORT_DEEPSLEEP_WAKEUP
esp_err_t gpio_deep_sleep_wakeup_enable(gpio_num_t gpio_num, gpio_int_type_t intr_type)
{
    if (!gpio_hal_is_valid_deepsleep_wakeup_gpio(gpio_num)) {
        ESP_LOGE(GPIO_TAG, "GPIO %d does not support deep sleep wakeup", gpio_num);
        return ESP_ERR_INVALID_ARG;
    }
    if ((intr_type != GPIO_INTR_LOW_LEVEL) && (intr_type != GPIO_INTR_HIGH_LEVEL)) {
        ESP_LOGE(GPIO_TAG, "GPIO wakeup only supports level mode, but edge mode set. gpio_num:%u", gpio_num);
        return ESP_ERR_INVALID_ARG;
    }
    portENTER_CRITICAL(&gpio_context.gpio_spinlock);
    gpio_hal_deepsleep_wakeup_enable(gpio_context.gpio_hal, gpio_num, intr_type);
#if SOC_GPIO_SUPPORT_SLP_SWITCH && CONFIG_ESP32C3_LIGHTSLEEP_GPIO_RESET_WORKAROUND
    gpio_hal_sleep_sel_dis(gpio_context.gpio_hal, gpio_num);
#endif
    portEXIT_CRITICAL(&gpio_context.gpio_spinlock);
    return ESP_OK;
}

esp_err_t gpio_deep_sleep_wakeup_disable(gpio_num_t gpio_num)
{
    if (!gpio_hal_is_valid_deepsleep_wakeup_gpio(gpio_num)) {
        ESP_LOGE(GPIO_TAG, "GPIO %d does not support deep sleep wakeup", gpio_num);
        return ESP_ERR_INVALID_ARG;
    }
    portENTER_CRITICAL(&gpio_context.gpio_spinlock);
    gpio_hal_deepsleep_wakeup_disable(gpio_context.gpio_hal, gpio_num);
#if SOC_GPIO_SUPPORT_SLP_SWITCH && CONFIG_ESP32C3_LIGHTSLEEP_GPIO_RESET_WORKAROUND
    gpio_hal_sleep_sel_en(gpio_context.gpio_hal, gpio_num);
#endif
    portEXIT_CRITICAL(&gpio_context.gpio_spinlock);
    return ESP_OK;
}
#endif // SOC_GPIO_SUPPORT_DEEPSLEEP_WAKEUP
