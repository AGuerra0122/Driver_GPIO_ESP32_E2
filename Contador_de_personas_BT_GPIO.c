 // FileName:        example_spp_acceptor_demo.c
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
/***********Librerias necesarias***********/
#include <stdint.h>
#include <string.h>
#include <stdbool.h>
#include <stdio.h>
#include "nvs.h"
#include "nvs_flash.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_bt.h"
#include "esp_bt_main.h"
#include "esp_gap_bt_api.h"
#include "esp_bt_device.h"
#include "esp_spp_api.h"
#include "GPIO_E2.h"
#include "time.h"
#include "sys/time.h"

/***********Definiciones necesarias***********/
#define SPP_SERVER_NAME "SPP_SERVER"	//Nombre del servidor bluetooth
#define EXAMPLE_DEVICE_NAME "Contador de personas"	//Nombre con el que aparecera la tarjeta para el smart phone

// Configuracion para el Bluetooth
#define SPP_SHOW_DATA 0
#define SPP_SHOW_SPEED 1
#define SPP_SHOW_MODE SPP_SHOW_DATA
const char *tag = "Bluetooth";

static const esp_spp_mode_t esp_spp_mode = ESP_SPP_MODE_CB;
static const esp_spp_sec_t sec_mask = ESP_SPP_SEC_AUTHENTICATE;
static const esp_spp_role_t role_slave = ESP_SPP_ROLE_SLAVE;

/* Definicion de los leds */
#define led_lleno 14 // Led que indica que la instalacion esta llena
#define led_vacio 12 // Led que indica que aun queda espacio en la instalacion
#define led_luz 2 // Led que reprecenta la iluminacion del local

/* Definicion de los botones */
#define Bttn_Entrada 18 // Boton ubicado en la entrada del local
#define Bttn_Salida_Trasera 19 // Boton ubicado en la parte posterior del local

/* Variables de control en la aplicacion */
char spp_data[256]; // Arreglo a imprimir por medio del bluetooth
int32_t People=0; // Variable contadora de personas
bool flag=0; // Bandera para controlar el encencido y apagado de una luz

/* Funciones de inicializacion */
static uint32_t init_led(void); // Funcion para la configuracion de los leds
static uint32_t init_isr(void); // Funcion para la configuracion de los botones
static uint32_t init_bt(void); // Funcion para la configuracion del bluetooth

/* Funciones principales */
void Boton_entrada(void *args); // Funcion para leventar o bajar la bandera de la iluminacion con el boton de la entrada
void Boton_salida_Trasera(void *args); // Funcion para leventar o bajar la bandera de la iluminacion con el boton de la salida
static void esp_spp_cb(esp_spp_cb_event_t event, esp_spp_cb_param_t *param); // Funcion para el bluetooth cada que pasa un evento
void esp_bt_gap_cb(esp_bt_gap_cb_event_t event, esp_bt_gap_cb_param_t *param); // Funcion para buscar y conestarse con los dispositivos bluetooth

/**************************************************************************
* Function: app_main
* Overview: Esta es la aplicacion principal, aqui se inicalizan los leds, botones y el bluetooth
* Input:
* Output: Solo retorna un 0 en caso de una buena ejecucion
*
*****************************************************************************/
uint32_t app_main(void){

    init_led(); // Se llama a la funcion para inicializar los tres leds de la app
    init_isr(); // Se le llama a la funcion para inicializar a los botones que se usaran con interrupciones
    init_bt(); // Se llama a la funcion para inicializar el bluetooth

	if(People==32){ // Si la variable contadora de personas es igual a 32 enciende el led que indica que esta lleno
	    	gpio_set_level(led_lleno, 1);
	    	gpio_set_level(led_vacio, 0);
	}

	if(People<32){ // Si la variable contadora de personas es menor a 32 enciende el led que indica que aun hay espacio adentro
		  gpio_set_level(led_vacio, 1);
		  gpio_set_level(led_lleno, 0);
	}

    while(1){ // Ciclo infinito para el control del la iluminacion
    	if(flag == 0){// Si la bandera esta desactivada se apaga la iluminacion
    		gpio_set_level(led_luz, 0);
    	}
    	if(flag == 1){// Si la bandera esta activada se enciende la iluminacion
    		gpio_set_level(led_luz, 1);
    	}
    }

    return 0;
}
/**************************************************************************
* Function: init_led
* Overview: En esta funcion se configuran como salida los pines a utilizar para los leds
* Input:
* Output: Retorna ESP_OK que indica que todo esta en orden
*
*****************************************************************************/
static uint32_t init_led(void){
	gpio_reset_left(); // Se resetan todos los pines del lado izquierdo de la tarjeta
	gpio_set_output(led_lleno); // Se configura el pin led_lleno(14) como salida
	gpio_set_output(led_vacio); // Se configura el pin led_vacio(12) como salida
	gpio_reset_pin(led_luz); // Se resetea el pin led_luz(2)
	gpio_set_output(led_luz); // Se configura el pin led_luz(2) como salida

    ESP_LOGI(tag, "Init led completed"); // Se envia el mensaje que indica que se inicializaron correctamente los leds
    return ESP_OK;
}
/**************************************************************************
* Function: init_isr
* Overview: En esta funcion se configuran como entrada con interrupcion los pines a utilizar para los botones
* Input:
* Output: Retorna ESP_OK que indica que todo esta en orden
*
*****************************************************************************/
static uint32_t init_isr(void){
	gpio_reset_pin(Bttn_Entrada); // Se recetea el pin Bttn_Entrada(18)
	gpio_set_input_isr(Bttn_Entrada, GPIO_INTR_POSEDGE); // Se configura el pin Bttn_Entrada(18) como entrada con interrupcion en el flanco positivo
	gpio_reset_pin(Bttn_Salida_Trasera); // Se recetea el pin Bttn_Salida_Trasera(19)
	gpio_set_input_isr(Bttn_Salida_Trasera, GPIO_INTR_NEGEDGE); // Se configura el pin Bttn_Salida_Trasera(19) como entrada con interrupcion en el flanco negativo

	gpio_install_isr_service(0); // Se inicia el servicio de interrupciones
	gpio_isr_handler_add(Bttn_Entrada,Boton_entrada,NULL); // Se establece que la funcion Boton_entrada sera llamada con la interrupcion del boton Bttn_Entrada
	gpio_isr_handler_add(Bttn_Salida_Trasera,Boton_salida_Trasera,NULL); // Se establece que la funcion Boton_salida_Trasera sera llamada con la interrupcion del boton Bttn_Salida_Trasera

	ESP_LOGI(tag, "Init isr completed"); // Se envia el mensaje que indica que se inicializaron correctamente los botones
	return ESP_OK;
}
/**************************************************************************
* Function: init_bt
* Overview: En esta funcion se configura el bluetooth
* Input:
* Output: Retorna ESP_OK que indica que todo esta en orden
*
*****************************************************************************/
static uint32_t init_bt(void){
    esp_err_t ret = nvs_flash_init(); // Se inicializa la particion NVS para recervar memoria
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND){ // En caso de que exista un error en la inicializacion lo corrige y vuelve a inicializar la particion
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
    ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_BLE)); // Se realiza una liberacion de memoria
    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT(); // Se configura el controlador bluetooth en modo default
    if ((ret = esp_bt_controller_init(&bt_cfg)) != ESP_OK){ // Se inicializa el controlador bluetooth
        ESP_LOGE(tag, "%s initialize controller failed: %s\n", __func__, esp_err_to_name(ret));
        return ESP_FAIL;
    }
    if ((ret = esp_bt_controller_enable(ESP_BT_MODE_CLASSIC_BT)) != ESP_OK){ // Se activa el controlador bluetooth
        ESP_LOGE(tag, "%s enable controller failed: %s\n", __func__, esp_err_to_name(ret));
        return ESP_FAIL;
    }
    if ((ret = esp_bluedroid_init()) != ESP_OK){ // Se inicializa el bluedroid
        ESP_LOGE(tag, "%s initialize bluedroid failed: %s\n", __func__, esp_err_to_name(ret));
        return ESP_FAIL;
    }
    if ((ret = esp_bluedroid_enable()) != ESP_OK){ // Se activa el bluedroid
        ESP_LOGE(tag, "%s enable bluedroid failed: %s\n", __func__, esp_err_to_name(ret));
        return ESP_FAIL;
    }
    if ((ret = esp_bt_gap_register_callback(esp_bt_gap_cb)) != ESP_OK){ // Se establece la funcion que realizara la busqueda de dispocitivos
        ESP_LOGE(tag, "%s gap register failed: %s\n", __func__, esp_err_to_name(ret));
        return ESP_FAIL;
    }
    if ((ret = esp_spp_register_callback(esp_spp_cb)) != ESP_OK){ // Se establece la funcion que realizara la comunicacion entre la tarjeta y el dispositivo
        ESP_LOGE(tag, "%s spp register failed: %s\n", __func__, esp_err_to_name(ret));
        return ESP_FAIL;
    }
    if ((ret = esp_spp_init(esp_spp_mode)) != ESP_OK){// Se Inicializa la comunicacion entre la tarjeta y los dispocitivos
        ESP_LOGE(tag, "%s spp init failed: %s\n", __func__, esp_err_to_name(ret));
        return ESP_FAIL;
    }

#if (CONFIG_BT_SSP_ENABLED == true)
    /* Se establecen parametros default para la conexion por bluetooth */
    esp_bt_sp_param_t param_type = ESP_BT_SP_IOCAP_MODE;
    esp_bt_io_cap_t iocap = ESP_BT_IO_CAP_IO;
    esp_bt_gap_set_security_param(param_type, &iocap, sizeof(uint8_t));
#endif

    /* Se establecen los paremetros de default para la conexion Legacy */
    esp_bt_pin_type_t pin_type = ESP_BT_PIN_TYPE_VARIABLE;
    esp_bt_pin_code_t pin_code;
    esp_bt_gap_set_pin(pin_type, 0, pin_code);
    return ESP_OK;
}
/**************************************************************************
* Function: Boton_entrada
* Overview: Esta funcion es la asignada al boton Bttn_entrada la cual cambia el valor de una bandera segun sea el caso
* 			para encender o apagar la iluminacion
* Input:
* Output:
*****************************************************************************/
void Boton_entrada(void *args){
	if(flag == 1){// Si la bandera esta en alto la baja
		flag = 0;
	}
	else if(flag == 0){// Si la bandera esta en bajo la sube
		flag = 1;
	}
}
/**************************************************************************
* Function: Boton_salida_Trasera
* Overview: Esta funcion es la asignada al boton Bttn_salida_Trasera la cual cambia el valor de una bandera segun sea el caso
* 			para encender o apagar la iluminacion
* Input:
* Output:
*****************************************************************************/
void Boton_salida_Trasera(void *args){
	if(flag == 1){// Si la bandera esta en alto la baja
		flag = 0;
	}
	else if(flag == 0){// Si la bandera esta en bajo la sube
		flag = 1;
	}
}
/**************************************************************************
* Function: esp_spp_cb
* Overview: Esta funcion sirve la comunicacion a traves del bluetooth
* Input: Recibe un evento por parte del bluetooth y un apuntador a la union de los valores para la comunicacion
* Output:
*****************************************************************************/
static void esp_spp_cb(esp_spp_cb_event_t event, esp_spp_cb_param_t *param){

    switch (event){ // Switch para realizar distintas acciones de acuerdo a los eventos recibidos a traves del bluetooth
		case ESP_SPP_INIT_EVT: // Evento de iniciacion del bluetooth
			ESP_LOGI(tag, "ESP_SPP_INIT_EVT");
			esp_spp_start_srv(sec_mask, role_slave, 0, SPP_SERVER_NAME);
			break;

		case ESP_SPP_START_EVT: // Evento de inicio para poner disponible la targeta para conectarse con algun dispositivo
			ESP_LOGI(tag, "ESP_SPP_START_EVT"); //
			esp_bt_dev_set_device_name(EXAMPLE_DEVICE_NAME);
			esp_bt_gap_set_scan_mode(ESP_BT_CONNECTABLE, ESP_BT_GENERAL_DISCOVERABLE);
			break;

		case ESP_SPP_DATA_IND_EVT: // Evento para cuando se reciben datos atraves del bluetooth
			#if (SPP_SHOW_MODE == SPP_SHOW_DATA)
				ESP_LOGI(tag, "ESP_SPP_DATA_IND_EVT len=%d handle=%d",param->data_ind.len, param->data_ind.handle);
				esp_log_buffer_hex("", param->data_ind.data, param->data_ind.len);
				for (size_t i = 0; i < (param->data_ind.len) - 2; i++){ // Se realiza un for para examinar los datos que entraron por vez para decidir que hacer
																		// en este caso guardar el caracter en una variable y tomar una decicion con ayuda de un switch

					char value = param->data_ind.data[i]; // Se guarda el caracter leido en la variable char value

					switch (value){ // Switch para aumentar o disminur la cuenta de personas en el local
					case '+': // En caso de recibir un + se aumentara en uno la cuenta
						People++;

						if(People>=32){ // Si la cuenta es mayor o igual a 32 se hara lo siguiente
							People=32; // Se iguala la cuenta a 32
							param->data_ind.len=People; // Se pasa el valor de la cuenta a param->data_ind.len
							gpio_set_level(led_lleno, 1); // Al estar el local lleno se enciende el led led_lleno(color rojo)
							gpio_set_level(led_vacio, 0); // y se apaga el led led_vacio(color verde) que nos indica si aun hay espacio dentro del local
						}
						if(People<32){ // Si la cuenta es menor a 32 se hara lo siguiente
							param->data_ind.len=People; // Se pasa el valor de la cuenta a param->data_ind.len
							  gpio_set_level(led_vacio, 1); // Al no estar el local lleno se enciende el led led_vacio(color verde)
							  gpio_set_level(led_lleno, 0); // y se apaga el led led_lleno(color rojo) que nos indica que el local esta lleno
						}
						break;

					case '-': // En caso de recibir un - se restara uno a la cuenta de personas en el local
						People--;
						if(People<=0){ // Si la cuenta es menor o igual a 0 se asigna 0 al contador
							People=0;
						}
						param->data_ind.len=People; // Se pasa el valor de la cuenta a param->data_ind.len
						gpio_set_level(led_lleno, 0); // Al no estar el local lleno se apaga el led led_lleno(color rojo)
						gpio_set_level(led_vacio, 1); // y se enciende el led led_vacio(color verde)) que nos indica si aun hay espacio dentro del local
						break;

					default:
						break;
					}
				}
				if(People>=32){ // Si la cuenta de personas es igual o mayor a 32 se asigna al arreglo spp_data un mensaje indicando que el local esta lleno
					sprintf(spp_data, "Limite alcanzado %d, espere por favor\n\r", param->data_ind.len);
				}
				if(People<32){ // Si la cuenta de personas es menor a 32 simplemente se asigna al arreglo spp_data la cantidad de personas dentro del local
					sprintf(spp_data, "%d\n\r", param->data_ind.len);
				}

				esp_spp_write(param->data_ind.handle, strlen(spp_data), (uint8_t*)spp_data); // Se imprime por medio del bluetooth la cadena de caracteres spp_data
			#endif
			break;

		default:
			break;
	}
}
/**************************************************************************
* Function: esp_bt_gap_cb
* Overview: En esta funcion se establece la configuracion para permitir la conexion entre la tarjeta y dispositivos externos
* Input: Recibe un evento por parte del bluetooth y un apuntador a la union de los valores para la comunicacion
* Output:
*****************************************************************************/
void esp_bt_gap_cb(esp_bt_gap_cb_event_t event, esp_bt_gap_cb_param_t *param){
    switch (event){ // Switch para realizar distintas acciones de acuerdo a los eventos recibidos a traves del bluetooth
		case ESP_BT_GAP_AUTH_CMPL_EVT:{ // En este caso se verifica la autenticacion para la tarjeta
			if (param->auth_cmpl.stat == ESP_BT_STATUS_SUCCESS){
				ESP_LOGI(tag, "authentication success: %s", param->auth_cmpl.device_name);
				esp_log_buffer_hex(tag, param->auth_cmpl.bda, ESP_BD_ADDR_LEN);
			}
			else{
				ESP_LOGE(tag, "authentication failed, status:%d", param->auth_cmpl.stat);
			}
			break;
		}
		case ESP_BT_GAP_PIN_REQ_EVT:{ // En este caso se solicita un pin de verificacion para la conexion
			ESP_LOGI(tag, "ESP_BT_GAP_PIN_REQ_EVT min_16_digit:%d", param->pin_req.min_16_digit);
			if (param->pin_req.min_16_digit){ // En caso de que el pin sea de 16 digitos se envia lo siguiente:
				ESP_LOGI(tag, "Input pin code: 0000 0000 0000 0000");
				esp_bt_pin_code_t pin_code = {0};
				esp_bt_gap_pin_reply(param->pin_req.bda, true, 16, pin_code);
			}
			else{ // De lo contrario envia 1234 como pin de verificacion
				ESP_LOGI(tag, "Input pin code: 1234");
				esp_bt_pin_code_t pin_code;
				pin_code[0] = '1';
				pin_code[1] = '2';
				pin_code[2] = '3';
				pin_code[3] = '4';
				esp_bt_gap_pin_reply(param->pin_req.bda, true, 4, pin_code);
			}
			break;
		}

	#if (CONFIG_BT_SSP_ENABLED == true)
		case ESP_BT_GAP_CFM_REQ_EVT: // En caso de existir una conexion se verificara un numero entre ambos dispositivos
			ESP_LOGI(tag, "ESP_BT_GAP_CFM_REQ_EVT Please compare the numeric value: %d", param->cfm_req.num_val);
			esp_bt_gap_ssp_confirm_reply(param->cfm_req.bda, true);
			break;
		case ESP_BT_GAP_KEY_NOTIF_EVT: // En caso de ser necesario se envia una llave de acceso
			ESP_LOGI(tag, "ESP_BT_GAP_KEY_NOTIF_EVT passkey:%d", param->key_notif.passkey);
			break;
		case ESP_BT_GAP_KEY_REQ_EVT: // En caso de ser necesaria una llave de acceso se solicita la misma al usuario
			ESP_LOGI(tag, "ESP_BT_GAP_KEY_REQ_EVT Please enter passkey!");
			break;
	#endif

		case ESP_BT_GAP_MODE_CHG_EVT: // En caso de recivir un cambio de evento hace lo siguiente
			ESP_LOGI(tag, "ESP_BT_GAP_MODE_CHG_EVT mode:%d", param->mode_chg.mode);
			break;

		default:{
			ESP_LOGI(tag, "event: %d", event);
			break;
		}
	}
	return;
}
