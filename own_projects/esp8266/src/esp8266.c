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
#include "os.h"               /* <= operating system header */
#include "ciaaPOSIX_stdio.h"  /* <= device handler header */
#include "ciaaPOSIX_string.h" /* <= string header */
#include "ciaak.h"            /* <= ciaa kernel header */
#include "esp8266.h"         /* <= own header */
#include "pwm.h"


/*==================[macros and definitions]=================================*/

/*==================[internal data declaration]==============================*/

/*==================[internal functions declaration]=========================*/

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

uint8_t * buffer;
uint32_t length;
uint32_t actualLength = 0;
uint32_t activacionesSinLeer;
uint8_t waitingResponse = 0;

int8_t buf2[512];

uint8_t  tmpMotorNumber;
uint8_t  tmpDutyCycle;
uint8_t  lastDutyCycle[2];
uint8_t  lastMotorNumber[2];


/*==================[external data definition]===============================*/

/*==================[internal functions definition]==========================*/

uint32_t sendCommand(const char * command, uint8_t * buf, uint32_t size);
MatchStatus tryMatchDutyCycle(char c);
void updateMotor(uint8_t dutyCycle);
Direction getDirection(uint8_t dutyCycle);

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

	/* Starts encoders configuration */
	// Chip_PININT_Init(LPC_GPIO_PIN_INT);
	// interruptConfiguration(0,2,8);

	SetRelAlarm(ActivatePeriodicTask, 10, 5);
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


TASK(PeriodicTask)
{
	int8_t buf[20];
	int32_t ret;
	uint8_t i, j;
	MatchStatus ret1, ret2, ret3;
	IPD_Command * lastIPD;
	uint8_t tmp;

	ret = ciaaPOSIX_read(fd_uart2, buf, 20);
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

				ret1 = tryMatchIPD(buf[i]);
				if (ret1 == COMPLETE){
					lastIPD = getLastIPD();

					for (j = 0; j < lastIPD->payloadLength; j++){
						if (tryMatchDutyCycle(lastIPD->buffer[j]) == COMPLETE){
							lastDutyCycle[tmpMotorNumber] = tmpDutyCycle;
						}
					}

					updateMotor(lastDutyCycle[0]);
					//updateMotor(lastDutyCycle[1]);

					/*
					if (ciaaPOSIX_strncmp(lastIPD->buffer, "PRENDER", lastIPD->payloadLength) == 0){
						tmp = 0b00111111;
						ciaaPOSIX_write(fd_out, &tmp, 1);
					}
					else if (ciaaPOSIX_strncmp(lastIPD->buffer, "APAGAR", lastIPD->payloadLength) == 0){
						tmp = 0;
						ciaaPOSIX_write(fd_out, &tmp, 1);
					}
*/
					//ciaaPOSIX_printf("ID de conexion: %d\r\nLongitud: %d\r\nPayload: %s\r\n", lastIPD.connectionID, lastIPD.payloadLength, lastIPD.buffer);
				}

				ret2 = tryMatchConnectionOpen(buf[i]);
				if (ret2 == COMPLETE){
					//ciaaPOSIX_printf("Conexion abierta: %d\r\n", lastClientConnected);
				}

				ret3 = tryMatchConnectionClosed(buf[i]);
				if (ret3 == COMPLETE){
					//ciaaPOSIX_printf("Conexion cerrada: %d\r\n", lastClientDisconnected);
				}

			}

			activacionesSinLeer = 0;
		}
	}

	TerminateTask();
}

Direction getDirection(uint8_t dutyCycle)
{
	if(dutyCycle >= 0 && dutyCycle <= 100)
		return ATRAS;
	else
		return ADELANTE;
}

void updateMotor(uint8_t dutyCycle)
{
	uint8_t foo = 0;
	Direction dir = getDirection(dutyCycle);
	if(dir == ADELANTE){
		ciaaPOSIX_write(getFileDescriptor(2),&foo, 1);
		dutyCycle -= 100;
		ciaaPOSIX_write(getFileDescriptor(0),&dutyCycle, 1);
	}else{
		dutyCycle = 100 - dutyCycle;
		ciaaPOSIX_write(getFileDescriptor(0),&foo, 1);
		ciaaPOSIX_write(getFileDescriptor(2),&dutyCycle, 1);
	}
}

static Status estado = S0;
static uint16_t msgPos = 0;
uint8_t dataLength;

MatchStatus tryMatchDutyCycle(char c){
	MatchStatus ret = NOT_MATCHES;

	switch(estado){
	case S0: /* Caracter separador entre el valor anterior y
        			el Ciclo de Trabajo */
		if (c == '%'){
			estado = S1;
			ret = INCOMPLETE;
		}
		break;
	case S1: /* Lee la longitud de la cantidad de caracteres que
         	 	 	 contiene el mensaje siguiente */
		if (c >= '1' && c <= '9'){
			dataLength = c - '0';
			estado = S2;
			ret = INCOMPLETE;
		}
		break;
	case S2: /* Lee la longitud de la cantidad de caracteres que
	         	 	 	 contiene el mensaje siguiente */
			if (c >= '0' && c <= '9'){
				tmpMotorNumber = c - '0';
				dataLength--;
				tmpDutyCycle = 0;
				estado = S3;
				ret = INCOMPLETE;
			}
			break;

	case S3: /* Matcheo de ',' que sucede al ID de conexi�n */
		if (c >= '0' && c <= '9' && dataLength > 0){
			dataLength--;
			tmpDutyCycle = (tmpDutyCycle * 10) + c - '0';

			if (dataLength == 0)
				ret = COMPLETE;
			else
				ret = INCOMPLETE;
		}
		break;
	default:
		break;
	}

	if (ret == NOT_MATCHES || ret == COMPLETE){
		estado = S0;
		msgPos = 0;
	}

	return ret;
}

/** @} doxygen end group definition */
/** @} doxygen end group definition */
/** @} doxygen end group definition */
/*==================[end of file]============================================*/

