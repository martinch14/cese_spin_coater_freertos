/*=====[APP]==========================================================
 * Copyright 2019 Gambarotta Martin Abel <magambarotta@mail.com>
 * All rights reserved.
 * License: license text or at least name and link
         (example: BSD-3-Clause <https://opensource.org/licenses/BSD-3-Clause>)
 *
 * Version: 1.0.0
 * Creation Date: 2019/06/16
 */
/*==================[inlcusiones]============================================*/

// Includes de FreeRTOS

#include "FreeRTOS.h"		//Motor del RTOS
#include "FreeRTOSConfig.h"	//Configuracion FreeRTOS
#include "task.h"			//API de control de tareas y temporizacion
#include "semphr.h"			//API de sincronizacion (sem y mutex)
#include "queue.h"			//API de Colas


// sAPI header
#include "sapi.h"
#include "board.h"

//	TMC API
#include "TMC4671/TMC4671_Register.h"

/*==================[definiciones y macros]==================================*/
#define MAX_QUEUE	18       //Valor maximo de datos en una estructura datosMotor_t
#define MIN_QUEUE	1        //Valor minimo de datos en una estructura datosMotor_t


typedef struct {
	uint8_t 	direccion;
	uint32_t 	valor;
}datosMotor_t;

//Arreglo de estructuras del tipo datoMotor_t

static  datosMotor_t	inicialConfig[MAX_QUEUE]={
	// Motor type &  PWM configuration
	{TMC4671_MOTOR_TYPE_N_POLE_PAIRS, 0x00030004},
	{TMC4671_PWM_POLARITIES, 0x00000000},
	{TMC4671_PWM_MAXCNT, 0x00000F9F},
	{TMC4671_PWM_BBM_H_BBM_L, 0x00000505},
	{TMC4671_PWM_SV_CHOP, 0x00000007},
	// ADC configuration
	{TMC4671_ADC_I_SELECT, 0x18000100},
	{TMC4671_dsADC_MCFG_B_MCFG_A, 0x00100010},
	{TMC4671_dsADC_MCLK_A, 0x20000000},
	{TMC4671_dsADC_MCLK_B, 0x00000000},
	{TMC4671_dsADC_MDEC_B_MDEC_A, 0x014E014E},
	{TMC4671_ADC_I0_SCALE_OFFSET, 0x01008001},
	{TMC4671_ADC_I1_SCALE_OFFSET, 0x01008001},
	// Open loop settings
	{TMC4671_OPENLOOP_MODE, 0x00000000},
	{TMC4671_OPENLOOP_ACCELERATION, 0x0000003C},
	{TMC4671_OPENLOOP_VELOCITY_TARGET, 0xFFFFFFF6},
	// Feedback selection
	{TMC4671_PHI_E_SELECTION, 0x00000002},
	{TMC4671_UQ_UD_EXT, 0x00000830},
	// ===== Open loop test drive =====
	// Switch to open loop velocity mode
	{TMC4671_MODE_RAMP_MODE_MOTION, 0x00000008}
};

static  datosMotor_t runConfig[1]={
//	{TMC4671_OPENLOOP_VELOCITY_TARGET, 0x0000003C}
	{TMC4671_OPENLOOP_VELOCITY_TARGET, 0x0000FFFF}
};
static  datosMotor_t	stopConfig[1]={
	{TMC4671_OPENLOOP_VELOCITY_TARGET, 0x00000000}
};
static  datosMotor_t	stopDriver[1]={
	{TMC4671_MOTION_MODE_STOPPED,0x00000001}
};



typedef enum {OK_ASIGNACION_MEMORIA, ERROR_ASIGNACION_MEMORIA } estadoAsignacionMemoria_t;
typedef enum {UP,FALLING,DOWN,RISING} estadoBotonEmergencia_t;

/*==================[definiciones de datos internos]=========================*/

QueueHandle_t xQueueColaMotor;
SemaphoreHandle_t xMutexPrint;
/*==================[definiciones de datos externos]=========================*/
DEBUG_PRINT_ENABLE;

/*==================[declaraciones de funciones internas]====================*/
//Funcion de inicializacion de interrupciones
void My_IRQ_Init(void);
//Funciones para el tratamiento de impresion por el recurso UART utilizando exclusion mutua
static void prvPrintfString(const char *pcString);
static void prvPrintfDireccionValor(uint8_t direccion, uint32_t valor);

/*==================[declaraciones de funciones externas]====================*/
// Prototipo de funcion de la tarea
void interpretacionComandosBluetooth( void* taskParmPtr );
void envioDatosMotor( void* taskParmPtr );

/*==================[funcion principal]======================================*/

// FUNCION PRINCIPAL, PUNTO DE ENTRADA AL PROGRAMA LUEGO DE ENCENDIDO O RESET.
int main(void)
{
   // ---------- CONFIGURACIONES ------------------------------
   // Inicializar y configurar la plataforma
   boardConfig();

   // UART for debug messages
   debugPrintConfigUart( UART_USB, 9600 );
   debugPrintlnString( "Ejecucion main()\r\n" );

   // Inicializacion UART-232 para la comunicacion Bluetooth con la aplicacion que envia los comandos
   uartConfig(UART_232,9600);

   // Inicializacion Interrupciones
   My_IRQ_Init();

   // Led para dar senial de vida
   gpioWrite( LED3, ON );

   // Tipo de datos para representar los estados de error en la asignacion de memoria
   estadoAsignacionMemoria_t estadoAsignacionMemoria=0;

   // Creacion de xMutexPrint para el tratamiento de impresion de la funcion printf
   // Se realiza el respectivo control de errores
   if( NULL == (xMutexPrint = xSemaphoreCreateMutex())){
	   estadoAsignacionMemoria=ERROR_ASIGNACION_MEMORIA;
//	   prvPrintfString("Error en Asignacion de Semaforo Mutex! \r\n");
	   printf("Error en Asignacion de Semaforo Mutex! \r\n");
   }

   // Creacion de la cola para envio y recepcion de datos de tipo datosMotor_t   (Direccion_Registo,Valor_Registro)
   // Se realiza control de Errores debido a que el sistema reserva memoria de manera dinamica

   if( NULL == (xQueueColaMotor = xQueueCreate(MAX_QUEUE, sizeof(datosMotor_t )))){
	   estadoAsignacionMemoria=ERROR_ASIGNACION_MEMORIA;
//	   prvPrintfString("Error en Asignacion de xQueueColaMotor!\r\n");
	   printf("Error en Asignacion de xQueueColaMotor!\r\n");
   }

   // Crear tarea en freeRTOS "interpretacionComandosBluetooth", enta tarea en la encargada de interpretar los comandos
   // recibidos por la UART-232, y en base al comando recibido llenar una cola de mensajes con los datos correspondientes
   // Su prioridad es 1, se ejecutara cuando la tareas tareas de mayor prioridad se bloqueen.
   xTaskCreate(
		   interpretacionComandosBluetooth,                     // Funcion de la tarea a ejecutar
      (const char *)"interpretacionComandosBluetooth",     		// Nombre de la tarea como String amigable para el usuario
      configMINIMAL_STACK_SIZE*2, // Cantidad de stack de la tarea
	  0,        				  // Puntero al primer elemento de la cola de configuracion del motor
      tskIDLE_PRIORITY+1,         // Prioridad de la tarea
      0                           // Puntero a la tarea creada en el sistema
   );


   // Creacion de tarea en freeRTOS "envioDatosMotor" (tarea de recepcion de datos de cola),  tiene una mayor prioridad que la tarea que envia los datos a la cola,
   //ya que en el momento en que se envian datos a la cola esta tarea se desbloqueara y sacara los datos de la cola, dejando la cola vacia.

   xTaskCreate(
		   envioDatosMotor,                     // Funcion de la tarea a ejecutar
      (const char *)"envioDatosMotor",     		// Nombre de la tarea como String amigable para el usuario
      configMINIMAL_STACK_SIZE*2, // Cantidad de stack de la tarea
      0,                          // Parametros de tarea
      tskIDLE_PRIORITY+2,         // Prioridad de la tarea
      0                           // Puntero a la tarea creada en el sistema
   );


   // Iniciar scheduler
   // Verifica que no hay error en la asignacion de memoria
   if( 0 == estadoAsignacionMemoria){
   vTaskStartScheduler();
   }
   	else{
//   		prvPrintfString("VTaskStartScheduler no se ha podido ejecutar por problemas en asignacion de Memoria Dinamica!\r\n");
   		printf("VTaskStartScheduler no se ha podido ejecutar por problemas en asignacion de Memoria Dinamica!\r\n");
   	}
   // ---------- REPETIR POR SIEMPRE --------------------------
   while( TRUE ) {
      // Si cae en este while 1 significa que no pudo iniciar el scheduler
   }

   // NO DEBE LLEGAR NUNCA AQUI, debido a que a este programa se ejecuta
   // directamenteno sobre un microcontroladore y no es llamado por ningun
   // Sistema Operativo, como en el caso de un programa para PC.
   return 0;
}

/*==================[definiciones de funciones internas]=====================*/

//Funcion para configurar interrupcion por GPIO de TEC 4 de la EDU-CIAA por flanco de bajada,
//
void My_IRQ_Init(){
	// Inicializacion Interrupcion de GPIO LPCOPEN
	Chip_PININT_Init(LPC_GPIO_PIN_INT);

	//Inicializacion del evento de Interrupcion
	//Mapeo de evento de interrupcion en la TEC 4 de la EDUCIAA -> Mapeado al Chanel 1 de Interrupcion de GPIO
	Chip_SCU_GPIOIntPinSel(0,1,9);
	//Configuracion para que el canal se active por flanco
	Chip_PININT_SetPinModeEdge(LPC_GPIO_PIN_INT, PININTCH0);
	//Configuracion para que el canal se active por flanco descendente
	Chip_PININT_EnableIntLow(LPC_GPIO_PIN_INT, PININTCH0);

	//Activacion de las interrupciones para el llamado al Handler
	NVIC_SetPriority(PIN_INT0_IRQn,2); 	// Configuracion en FreeRTOSConfig.h  de la definicion configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY  ==> 1
	NVIC_EnableIRQ(PIN_INT0_IRQn);

}

//  Se define una funcion interna con proteccion por mutex (exclusion mutua)
//  para el manejo del recurso de impresion de un string  a traves de la UART
//
static void prvPrintfString(const char *pcString){
	//Verificacion de que se pueda tomar el semaforo
	if(pdTRUE == xSemaphoreTake(xMutexPrint,portMAX_DELAY)){
		printf("%s",pcString);
		fflush(stdout);
	xSemaphoreGive(xMutexPrint);
	}
}

//  Se define una funcion interna con proteccion por mutex (exclusion mutua)
//  para el manejo del recurso de impresion de do valores de tipo  (uint8_t direccion , uint32_t valor)  a traves de la UART
//
static void prvPrintfDireccionValor(uint8_t  direccion, uint32_t valor){
	//Verificacion de que se pueda tomar el semaforo
	if(pdTRUE == xSemaphoreTake(xMutexPrint,portMAX_DELAY)){
		printf("Register:0x%.2X\t\t",direccion);
		printf("Value:0x%.8X\r\n",valor);
		fflush(stdout);
	xSemaphoreGive(xMutexPrint);
	}

}


/*==================[definiciones de funciones externas]=====================*/

// Implementacion de funcion "interpretacionComandosBluetooth", es una Maquina de Estados que
//	evalua el dato recibido por la UART-232 y datos de una estructura a traves de una cola
void interpretacionComandosBluetooth(void* taskParmPtr) {
	// ---------- CONFIGURACIONES ------------------------------
	uint8_t data_in,i;
	datosMotor_t  *inicial;
	datosMotor_t  *run;
	datosMotor_t  *stop;

	inicial = &inicialConfig[0];
	run		= &runConfig[0];
	stop	= &stopConfig[0];

	// ---------- REPETIR POR SIEMPRE --------------------------
	while (TRUE) {

		if(uartReadByte (UART_USB,&data_in)){
			gpioWrite(LED2, ON);
			switch  (data_in){
			case 'C':
				for(i=0;i<MAX_QUEUE;i++){
					//Envio las referencias del arreglo de estructura de datos del tipo datosMotor_t
					if (pdTRUE == xQueueSend(xQueueColaMotor,&inicial[i] , 0));
				}
				fflush(stdin);
				gpioWrite(LED2, OFF);
				break;
			case 'R':
				if (pdTRUE == xQueueSend(xQueueColaMotor, &run[0] , 0));
				fflush(stdin);
				gpioWrite(LED2, OFF);
				break;

			case 'S':
				if (pdTRUE == xQueueSend(xQueueColaMotor, &stop[0] , 0));
				fflush(stdin);
				gpioWrite(LED2, OFF);
				break;
			default:
				gpioWrite(LED2, OFF);
				break;
			}
		}
	}
}

// Implementacion de Tarea "envioDatosMotor" La misma recibe datos en una cola de estructuras de tipo datosMotor_t y los envia a traves de UART-USB,
// los datos recibidos provienden de la tarea "interpretacionComandosBluetooth" y tambien pueden venir de la rutina de interrupcion del tipo GPIO generada por la TEC4 de la EDUCIAA

void envioDatosMotor(void* taskParmPtr) {
	// ---------- CONFIGURACIONES ------------------------------
	// Tarea periodica cada 500 ms
	portTickType xPeriodicity = 500 / portTICK_RATE_MS;
	// Creacion variable del tipo datosMotor_t donde se cargaran los datos recibidos por la xQueueColaMotor
	datosMotor_t   datocola_estructura;

	// ---------- REPETIR POR SIEMPRE --------------------------
	while (TRUE) {

		if (xQueueColaMotor != 0){
			if(pdTRUE == xQueueReceive(xQueueColaMotor,&datocola_estructura,xPeriodicity)){

				prvPrintfDireccionValor(datocola_estructura.direccion,datocola_estructura.valor);
			}
		}
	}
}

//Implementacion del Handler de GPIO de TEC4 de EDUCIAA

void GPIO0_IRQHandler(void){
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;

	// Verificacion para ver si la interrupcion es la esperada
	if(Chip_PININT_GetFallStates(LPC_GPIO_PIN_INT) & PININTCH0 ){
	// Se borra el respectivo flag de la interrupcion
	Chip_PININT_ClearIntStatus(LPC_GPIO_PIN_INT, PININTCH0);
	// Se envian los datos de la estructura stopDriver a la tarea "envioDatosMotor" que se encargara de enviarlos por UART-USB
	if (pdTRUE ==  xQueueSendFromISR( xQueueColaMotor, &stopDriver[0], &xHigherPriorityTaskWoken ));
	}

	portYIELD_FROM_ISR( xHigherPriorityTaskWoken );

}



/*==================[fin del archivo]========================================*/
