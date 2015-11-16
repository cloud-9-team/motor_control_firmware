/* Copyright 2014, Mariano Cerdeiro
 * Copyright 2014, Pablo Ridolfi
 * Copyright 2014, Juan Cecconi
 * Copyright 2014, Gustavo Muro
 *
 * This file is part of CIAA Firmware.
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
 *
 */


/*==================[inclusions]=============================================*/
#include "chip.h"
#include "os.h"               /* <= operating system header */
#include "ciaaPOSIX_stdio.h"  /* <= device handler header */
#include "ciaaPOSIX_string.h" /* <= string header */
#include "ciaak.h"            /* <= ciaa kernel header */
#include "esp8266.h"         /* <= own header */
#include "pwm.h"
#include "parser.h"
#include "encoder.h"
#include "StringUtils.h"

#if 0
volatile uint32_t * DWT_CTRL = (uint32_t *)0xE0001000;
volatile uint32_t * DWT_CYCCNT = (uint32_t *)0xE0001004;

volatile static uint32_t execTime;

void DWTStart (void)
{
	*DWT_CTRL |= 1;
	*DWT_CYCCNT = 0;
}

uint32_t DWTStop (void)
{
	uint32_t cycles = *DWT_CYCCNT;

	cycles = ((uint64_t)1000000 * cycles) / SystemCoreClock;
	if (cycles > execTime)
		execTime = cycles;

	return (cycles);
}
#endif

/*==================[macros and definitions]=================================*/

#define IPD_BUFFER_LENGTH   (256)
#define MOTOR_COUNT			(2)

/*==================[internal data declaration]==============================*/

/*==================[internal functions declaration]=========================*/

uint32_t sendCommand(const char * command, uint8_t * buf, uint32_t size);

/*==================[internal data definition]===============================*/
/** \brief File descriptor for digital input ports
 *
 * Device path /dev/dio/in/0
 */
static int32_t fd_in;

/** \brief File descriptor for digital output ports
 *
 * Device path /dev/dio/out/0
 */
static int32_t fd_out;

/** \brief File descriptor of the USB uart
 *
 * Device path /dev/serial/uart/1
 */
static int32_t fd_uart1;

/** \brief File descriptor of the RS232 uart
 *
 * Device path /dev/serial/uart/2
 */
static int32_t fd_uart2;


static uint8_t bufferIPD[IPD_BUFFER_LENGTH];
static PARSER_DATA_IPD_T parserIPD_data;
static const Parser parserIPD = {AT_MSG_IPD, &parserIPD_data};

static PARSER_DATA_DUTYCYCLE_T parserDutyCycle_data;
static const Parser parserDutyCycle = {USER_DUTYCYCLE, &parserDutyCycle_data};

uint8_t * buffer;
uint16_t gpio_buffer = 0;
uint32_t length;
uint32_t actualLength = 0;
uint32_t activacionesSinLeer;
uint32_t activacionesSinLeer1;
uint8_t waitingResponse = 0;

int8_t buf2[512];

MotorControlData  lastDutyCycle[MOTOR_COUNT];


/*==================[external data definition]===============================*/

/*==================[internal functions definition]==========================*/

/*==================[external functions definition]==========================*/
/** \brief Main function
 *
 * This is the main entry point of the software.
 *
 * \returns 0
 *
 * \remarks This function never returns. Return value is only to avoid compiler
 *          warnings or errors.
 */
int main(void)
{
	/* Starts the operating system in the Application Mode 1 */
	/* This example has only one Application Mode */
	StartOS(AppMode1);

	/* StartOs shall never returns, but to avoid compiler warnings or errors
	 * 0 is returned */
	return 0;
}

/** \brief Error Hook function
 *
 * This function is called from the os if an os interface (API) returns an
 * error. Is for debugging proposes. If called this function triggers a
 * ShutdownOs which ends in a while(1).
 *
 * The values:
 *    OSErrorGetServiceId
 *    OSErrorGetParam1
 *    OSErrorGetParam2
 *    OSErrorGetParam3
 *    OSErrorGetRet
 *
 * will provide you the interface, the input parameters and the returned value.
 * For more details see the OSEK specification:
 * http://portal.osek-vdx.org/files/pdf/specs/os223.pdf
 *
 */
void ErrorHook(void)
{
	ciaaPOSIX_printf("ErrorHook was called\n");
	ciaaPOSIX_printf("Service: %d, P1: %d, P2: %d, P3: %d, RET: %d\n", OSErrorGetServiceId(), OSErrorGetParam1(), OSErrorGetParam2(), OSErrorGetParam3(), OSErrorGetRet());
	ShutdownOS(0);
}

/** \brief Initial task
 *
 * This task is started automatically in the application mode 1.
 */
TASK(InitTask)
{
	/* init CIAA kernel and devices */
	ciaak_start();

	/* open CIAA digital inputs */
	fd_in = ciaaPOSIX_open("/dev/dio/in/0", ciaaPOSIX_O_RDONLY);

	/* open CIAA digital outputs */
	fd_out = ciaaPOSIX_open("/dev/dio/out/0", ciaaPOSIX_O_RDWR);

	/* open UART connected to USB bridge (FT2232) */
	fd_uart1 = ciaaPOSIX_open("/dev/serial/uart/1", ciaaPOSIX_O_RDWR | ciaaPOSIX_O_NONBLOCK);

	/* open UART connected to RS232 connector */
	fd_uart2 = ciaaPOSIX_open("/dev/serial/uart/2", ciaaPOSIX_O_RDWR | ciaaPOSIX_O_NONBLOCK);

	/* change baud rate */
	ciaaPOSIX_ioctl(fd_uart1, ciaaPOSIX_IOCTL_SET_BAUDRATE, (void *)ciaaBAUDRATE_115200);
	ciaaPOSIX_ioctl(fd_uart2, ciaaPOSIX_IOCTL_SET_BAUDRATE, (void *)ciaaBAUDRATE_115200);

	/* change FIFO TRIGGER LEVEL for uart usb */
	ciaaPOSIX_ioctl(fd_uart1, ciaaPOSIX_IOCTL_SET_FIFO_TRIGGER_LEVEL, (void *)ciaaFIFO_TRIGGER_LEVEL3);

	/* Starts PWM configuration */
	ciaaPWM_init();

	parser_initModule();

	parser_init(&parserIPD);
	parserIPD_data.results.bufferLength = IPD_BUFFER_LENGTH;
	parserIPD_data.results.buffer = bufferIPD;

	parser_init(&parserDutyCycle);

	encoder_init();

	/* Configuracion de los GPIO de salida. */
	ciaaPOSIX_read(fd_out,&gpio_buffer,2);
	/* Se guardan los valores que vienen por defecto en la lectura y se setean en 0
	 * 	aquellos que luego seran cambiados en la escritura. */
	gpio_buffer &= ~(ENABLE12|ENABLE34|ESP8266_EN|ESP8266_RST);
	/* Habilitacion de los enable, /reset y chip_enable del puente H. */
	gpio_buffer ^= (ENABLE12|ENABLE34|ESP8266_EN|ESP8266_RST);
	ciaaPOSIX_write(fd_out,&gpio_buffer,2);

	SetRelAlarm(ActivatePeriodicTask, 10, 5);
	SetRelAlarm(ActivateSendStatusTask, 250, 1000);
	ActivateTask(SerialEchoTask);

	/* end InitTask */
	TerminateTask();
}

uint32_t sendCommand(const char * command, uint8_t * buf, uint32_t size){
	uint8_t len = ciaaPOSIX_strlen(command);

	ciaaPOSIX_write(fd_uart2, command, len);

	waitingResponse = 1;
	buffer = buf;
	length = size;
	actualLength = 0;

	while (ciaaPOSIX_strncmp(buf, command, len) != 0 && actualLength < len);

	// Si no empieza con el comando tirar error...
	if (ciaaPOSIX_strncmp(buf, command, len) != 0){
		//return 0;
	}

	while (activacionesSinLeer < 50 * 2 * 4); // Espera 2 segundos durante los cuales no se haya recibido datos

	// Se supone que se tiene la respuesta completa

	buf[actualLength++] = '\0';
	waitingResponse = 0;

	return actualLength;
}



TASK(SerialEchoTask)
{
	int8_t buf[20];    // buffer for uart operation
	int32_t ret;       // return value variable for posix calls
	uint32_t i;

	for (i = 0; i < 0xFFFFFF; i++);

	sendCommand("AT+RST\r\n", buf2, 512);

	sendCommand("AT+CWMODE=2\r\n", buf2, 512);

	sendCommand("AT+CWSAP=\"wifi\",\"12345678\",11,3\r\n", buf2, 512);

	sendCommand("AT+CIPMUX=1\r\n", buf2, 512);

	sendCommand("AT+CIPSERVER=1,8080\r\n", buf2, 512);

	while(1)
	{
		// wait for any character ...
		ret = ciaaPOSIX_read(fd_uart1, buf, 20);

		if(ret > 0)
		{
			// also write them to the other device
			ciaaPOSIX_write(fd_uart2, buf, ret);
		}

	}
}

TASK(SendStatusTask){
	uint8_t buffer6[20] = "\r\nRPM: xxxx";

	uintToString(encoder_getAverageRPM(1), 4, &(buffer6[7]));
	ciaaPOSIX_write(fd_uart1, buffer6, ciaaPOSIX_strlen(buffer6));

	TerminateTask();
}

TASK(PeriodicTask)
{
	int8_t buf[32];
	int32_t ret;
	uint8_t i;
	uint16_t j;
	ParserStatus ret1;

	ret = ciaaPOSIX_read(fd_uart2, buf, 32);
	if (waitingResponse){
		if(ret > 0)
		{
			ciaaPOSIX_write(fd_uart1, buf, ret);
			for (i = 0; i < ret; i++){
				buffer[actualLength++] = buf[i];
			}
			activacionesSinLeer = 0;
		}
		else{
			activacionesSinLeer++;
		}
	}
	else{
		if(ret > 0)
		{
			ciaaPOSIX_write(fd_uart1, buf, ret);

			for (i = 0; i < ret; i++){

				ret1 = parser_tryMatch(&parserIPD, buf[i]);

				if (ret1 == STATUS_COMPLETE){

					/* Pongo el n�mero de motor en un valor incorrecto. Esto es para evitar
					 * actualizar las salidas PWM si es que no se recibi� un comando de
					 * usuario para ello.
					 */
					for (j = 0; j < MOTOR_COUNT; j++){
						lastDutyCycle[j].motorID = MOTOR_COUNT + 1;
					}

					for (j = 0; j < (parserIPD_data.results.payloadLength > parserIPD_data.results.bufferLength ? parserIPD_data.results.bufferLength : parserIPD_data.results.payloadLength); j++){
						if (parser_tryMatch(&parserDutyCycle, bufferIPD[j]) == STATUS_COMPLETE &&
								parserDutyCycle_data.results.motorID < MOTOR_COUNT){

							lastDutyCycle[parserDutyCycle_data.results.motorID] = parserDutyCycle_data.results;

						}
					}

					for (j = 0; j < MOTOR_COUNT; j++){
						ciaaPWM_updateMotor(lastDutyCycle[j]);
					}

				}

			}

			activacionesSinLeer = 0;
		}
	}
/*
	activacionesSinLeer1++;
	if (activacionesSinLeer1 >= 20*10){
		OSEK_TASK_SendStatusTask();
		activacionesSinLeer1 = 0;
	}
*/
	TerminateTask();
}


/** @} doxygen end group definition */
/** @} doxygen end group definition */
/** @} doxygen end group definition */
/*==================[end of file]============================================*/

