/* Copyright 2017-2018, Eric Pernia
 * All rights reserved.
 *
 * This file is part of sAPI Library.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

/*==================[inlcusiones]============================================*/

// Includes de FreeRTOS
#include "FreeRTOS.h"
#include "FreeRTOSConfig.h"
#include "task.h"			//API de Tareas y Temporizaciones
#include "semphr.h"			//API de sincronizacion (sem y mutex)

// sAPI header
#include "sapi.h"
//#include "../../cese_spin_coater_freertos/inc/FreeRTOSConfig.h"

/*==================[definiciones y macros]==================================*/

typedef enum {OK_ASIGNACION_MEMORIA, ERROR_ASIGNACION_MEMORIA } estadoAsignacionMemoria_t;
typedef enum {UP,FALLING,DOWN,RISING} estadoBotonEmergencia_t;
/*==================[definiciones de datos internos]=========================*/

SemaphoreHandle_t 	eventoBotonEmergenciaPulsado;


/*==================[definiciones de datos externos]=========================*/

DEBUG_PRINT_ENABLE;

/*==================[declaraciones de funciones internas]====================*/




/*==================[declaraciones de funciones externas]====================*/

// Prototipo de funciones de la tarea

//Sensa el estado de TEC4, si se presiona apaga el motor inmediatamente
void myTask_BotonEmergencia(void* taskParmPtr);
//Espera los datos enviados desde la UART y los carga en el driver
void myTask_ReadWriteRegisterDriver(void* taskParmPtr);
//Lee los datos recibidos desde la UART
//Cuando envio datos analizar la siguientes opciones:
//1) Usar variables globales protegidas por mutex para enviar la informacion
//2)Usar colas de mensaje
void myTask_ControlUart(void* taskParmPtr);
//Analiza si el motor esta girando o no, Enciende LED3 si esta girando, Enciende LEDR si esta detenido
void myTask_SenialesLed(void* taskParmPtr);

/*==================[funcion principal]======================================*/

// FUNCION PRINCIPAL, PUNTO DE ENTRADA AL PROGRAMA LUEGO DE ENCENDIDO O RESET.
int main(void) {
	// ---------- CONFIGURACIONES ------------------------------
	// Inicializar y configurar la plataforma
	boardConfig();

	// UART for debug messages
	debugPrintConfigUart( UART_USB, 115200 );
	debugPrintlnString("Blinky con freeRTOS y sAPI.");

	// Creacion de Semaforos  con su respectivo control de errores debido a q el sistema reserva memoria de manera dinamica

	estadoAsignacionMemoria_t estadoAsignacionMemoria=0;
	//Al crear los semaforos los mismos arrancan tomados!!

	if( NULL == (eventoBotonEmergenciaPulsado=xSemaphoreCreateBinary())){
		estadoAsignacionMemoria=ERROR_ASIGNACION_MEMORIA;
		printf("Error en Asignacion de Semaforo 'eventoBotonEmergenciaPulsado' \r\n");
	}


	// Crear tarea myTask_BotonEmergencia
	xTaskCreate(myTask_BotonEmergencia,        			// Funcion de la tarea a ejecutar
			(const char *) "myTask_BotonEmergencia", 	// Nombre de la tarea como String amigable para el usuario
			configMINIMAL_STACK_SIZE * 2, 				// Cantidad de stack de la tarea
			0,                          				// Parametros de tarea
			tskIDLE_PRIORITY + 3 ,         				// Prioridad de la tarea
			0                         					// Puntero a la tarea creada en el sistema
			);

	// Crear tarea myTask_ReadWriteRegisterDriver
	xTaskCreate(myTask_ReadWriteRegisterDriver, // Funcion de la tarea a ejecutar
			(const char *) "myTask_ReadWriteRegisterDriver", // Nombre de la tarea como String amigable para el usuario
			configMINIMAL_STACK_SIZE * 2, 		// Cantidad de stack de la tarea
			0,                          				// Parametros de tarea
			tskIDLE_PRIORITY + 1,         				// Prioridad de la tarea
			0                         // Puntero a la tarea creada en el sistema
			);

	// Crear tarea myTask_ControlUart
	xTaskCreate(myTask_ControlUart,            // Funcion de la tarea a ejecutar
			(const char *) "myTask_ControlUart", // Nombre de la tarea como String amigable para el usuario
			configMINIMAL_STACK_SIZE * 2, 		// Cantidad de stack de la tarea
			0,                          				// Parametros de tarea
			tskIDLE_PRIORITY + 2,         				// Prioridad de la tarea
			0                         // Puntero a la tarea creada en el sistema
			);

	// Crear tarea myTask_SenialesLed
	xTaskCreate(myTask_SenialesLed,            // Funcion de la tarea a ejecutar
			(const char *) "myTask_SenialesLed", // Nombre de la tarea como String amigable para el usuario
			configMINIMAL_STACK_SIZE * 2, 		// Cantidad de stack de la tarea
			0,                          				// Parametros de tarea
			tskIDLE_PRIORITY + 4,         				// Prioridad de la tarea
			0                         // Puntero a la tarea creada en el sistema
			);

	// Iniciar scheduler   //Verifica que no hay error en la asignacion de memoria de los semaforos

	if( 0 == estadoAsignacionMemoria){
	vTaskStartScheduler();
	}
	else{
		printf("VTaskStartScheduler no se ha podido ejecutar por problemas en asignacion de memoria dinamica a semaforos!\r\n");
	}
	// ---------- REPETIR POR SIEMPRE --------------------------
	while ( TRUE) {
		// Si cae en este while 1 significa que no pudo iniciar el scheduler
	}

	// NO DEBE LLEGAR NUNCA AQUI, debido a que a este programa se ejecuta
	// directamenteno sobre un microcontroladore y no es llamado por ningun
	// Sistema Operativo, como en el caso de un programa para PC.
	return 0;
}

/*==================[definiciones de funciones internas]=====================*/

/*==================[definiciones de funciones externas]=====================*/

// Implementacion de funciones de las tareas
// Ordenadas de mayor a menor prioridad



void myTask_BotonEmergencia(void* taskParmPtr) {
	// ---------- CONFIGURACIONES ------------------------------
	estadoBotonEmergencia_t estado = UP;
	portTickType tiempoDesdeBotonAccionado;
	printf("Boton de emergencia\r\n");

	// Tarea periodica cada 10 ms  //Verificacion del accionamiento de TEC4 para apagar el motor de manera urgente
	portTickType xPeriodicity = 10 / portTICK_RATE_MS;
	portTickType xLastWakeTime = xTaskGetTickCount();

	// ---------- REPETIR POR SIEMPRE --------------------------
	while (TRUE) {
//		gpioToggle(LEDB);
//		printf("Blink!\r\n");
//		// Envia la tarea al estado bloqueado durante xPeriodicity (delay periodico)

		switch (estado) {
		case UP:
			if (0 == gpioRead(TEC2)) {
				estado = FALLING;
				tiempoDesdeBotonAccionado = xTaskGetTickCount(); //Guardamos el tiempo representado x Ticks del Sistema
			}
			break;
		case FALLING:
			if (0 == gpioRead(TEC2)) {
				if ((xTaskGetTickCount() - tiempoDesdeBotonAccionado)
						* portTICK_RATE_MS >= 20) { //Multiplico los tick x portTICK_RATE_MS para expresarlo en ms
					estado = DOWN;					//Ver que pasa con el desborde del contador  , supuestamente x mas que desborde da bien la cuenta, VERIFICAR!!
					//Notificacion de que ocurrio el evento para mandar a apagar el motor!!!!!!!
					printf("Ejecutar xSemaphoreGive\r\n");
					xSemaphoreGive(eventoBotonEmergenciaPulsado);
					printf("Ejecutado xSemaphoreGive\r\n");
				} else {
					estado = UP;
				}
			}
			break;
		case DOWN:
			if (1 == gpioRead(TEC2)) {
				estado = RISING;
				tiempoDesdeBotonAccionado = xTaskGetTickCount(); //Guardamos el tiempo representado x Ticks del Sistema
			}
			break;

		case RISING:
			if (1 == gpioRead(TEC2)) {
				if ((xTaskGetTickCount() - tiempoDesdeBotonAccionado)
						* portTICK_RATE_MS >= 20) {
					estado = UP;
					//No hay evento para luego de un apagado de emergencia, se deberia reiniciar la maquina o volver a setear la config del motor
				} else {
					estado = DOWN;
				}
			}
			break;
		default:
			break;
		}
		vTaskDelayUntil(&xLastWakeTime, xPeriodicity);
	}
}

void myTask_ReadWriteRegisterDriver(void* taskParmPtr) {
	// ---------- CONFIGURACIONES ------------------------------
	// ---------- REPETIR POR SIEMPRE --------------------------
	while (TRUE) {
	}
}

void myTask_ControlUart(void* taskParmPtr) {
	// ---------- CONFIGURACIONES ------------------------------
	// ---------- REPETIR POR SIEMPRE --------------------------
	while (TRUE) {
	}
}

//Tarea que trabaja por eventos, no es una tarea periodida, por eso no hay ningun delayuntil ,
//si la prioridad es muy alta, el evento se queda "PEGADO" y produce que no se ejecute la tarea BotonEmergencia

void myTask_SenialesLed(void* taskParmPtr) {
	// ---------- CONFIGURACIONES ------------------------------
	printf("Visualizacion a traves de los led del estado del sistema!\r\n");

//	//Al iniciar 	LEDR=ON motor detenido
//	gpioWrite(LEDR, ON);
//	//Al iniciar	LED=OFF motor detenido
//	gpioWrite(LED3, OFF);
//	// ---------- REPETIR POR SIEMPRE --------------------------
	while (TRUE) {
		//Uso xSemaphoreTake para enterarme de que ocurrio el evento y actuar en consecuencia
		//Sincronizacion con el evento de pulsacion del boton de emergencia!
		//Hacer control de errores por si no se acciona Nunca el boton, deberiamo detectar el desborde , detectar  pdFalse si no puede tomar el semaforo
		xSemaphoreTake(eventoBotonEmergenciaPulsado, portMAX_DELAY);
		printf("Tomado xSemaphoreTake\r\n");
		gpioWrite(LEDR,ON);

//TODO:	//Evento de motor en funcionamiento, LED3   Motor-On        LEDR	Motor-OFF
		//Evento de conexion bluetooth apareado  LEDB fijo      sin aparear con dispositivo Bluetooth parpadeando


	}
}

/*==================[fin del archivo]========================================*/
