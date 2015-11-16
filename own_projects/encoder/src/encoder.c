#include "encoder.h"
#include "chip.h"
#include "ciaaPOSIX_stdio.h"  /* <= device handler header */
#include "ciaaPOSIX_string.h"
#include "ciaak.h"            /* <= ciaa kernel header */
#include "os.h"               /* <= operating system header */

uint32_t fd_in;
uint8_t temp;
uint8_t ledActual = LED_RGB_R;
int16_t X_pos = 0; // From 0 to WIDTH
unsigned int count = 0;
static unsigned char length = 0;
static unsigned char buffer[10];
static unsigned int  countValues[ARRAYSIZE];

#define SYS_WRITEC	(0x03)
#define SYS_WRITE0	(0x04)
#define SYS_READC	(0x07)
#define SYS_CLOCK	(0x10)

static void DEBUG_write_char(unsigned char c){
	register int reg0 asm("r0");
	register int reg1 asm("r1");
	reg0 = SYS_WRITEC;
	reg1 = (int)&c;
	asm("BKPT 0xAB");
}

static void DEBUG_write_string(char *c){
	register int reg0 asm("r0");
	register int reg1 asm("r1");
	reg0 = SYS_WRITE0;
	reg1 = (int)c;
	asm("BKPT 0xAB");
}

static char DEBUG_read_char(void){
	register int reg0 asm("r0");
	register int reg1 asm("r1");
	reg0 = SYS_READC;
	reg1 = 0;
	asm("BKPT 0xAB");
	return reg0;
}

static char DEBUG_get_clock(void){
	register int reg0 asm("r0");
	register int reg1 asm("r1");
	reg0 = SYS_READC;
	reg1 = 0;
	asm("BKPT 0xAB");
	return reg0;
}


//int main(void)
//{
//	StartOS(AppMode1);
//	return 0;
//}

//void ErrorHook(void)
//{
//	ciaaPOSIX_printf("ErrorHook was called\n");
//	ciaaPOSIX_printf("Service: %d, P1: %d, P2: %d, P3: %d, RET: %d\n", OSErrorGetServiceId(), OSErrorGetParam1(), OSErrorGetParam2(), OSErrorGetParam3(), OSErrorGetRet());
//	ShutdownOS(0);
//}

void interruptConfiguration(uint32_t channel, uint8_t port, uint8_t pin){
	Chip_SCU_GPIOIntPinSel(channel, port, pin); // GPIO2[8]
	Chip_PININT_SetPinModeEdge(LPC_GPIO_PIN_INT,(1 << channel));
	Chip_PININT_EnableIntLow(LPC_GPIO_PIN_INT,(1 << channel));
}

/** \brief Initial task
 *
 * This task is started automatically in the application mode 1.
 */
TASK(InitTask)
{

	/* init CIAA kernel and devices */
	ciaak_start();

	Chip_PININT_Init(LPC_GPIO_PIN_INT);

//	Chip_SCU_PinMux(6,12,MD_PUP|MD_EZI|MD_ZI,FUNC0); /* GPIO2[8] -> GPIO8 en EDU-CIAA */
//	Chip_GPIO_SetDir(LPC_GPIO_PORT, 2, (1<<8), 0);

//	Chip_SCU_GPIOIntPinSel(0, 2, 8); // GPIO2[8]
	// Hay hasta 8 fuentes de interrupcion simultaneas para los GPIOs.
	// Cada una debe tener un identificador.

	//Chip_PININT_SetPinModeLevel(LPC_GPIO_PIN_INT, PININTCH0);
	//Chip_PININT_EnableIntHigh(LPC_GPIO_PIN_INT, PININTCH0);
//	Chip_PININT_SetPinModeEdge(LPC_GPIO_PIN_INT, PININTCH0);
//	Chip_PININT_EnableIntLow(LPC_GPIO_PIN_INT, PININTCH0);

 	interruptConfiguration(0,2,8);
	fd_in = ciaaPOSIX_open("/dev/dio/in/0", ciaaPOSIX_O_RDONLY);

	SetRelAlarm(ActivatePeriodicTask, 350, PERIOD);

	/* terminate task */
	TerminateTask();
}

ISR(GPIO0_IRQHandler)
{
	Chip_PININT_ClearIntStatus(LPC_GPIO_PIN_INT, PININTCH0);
}

ISR(GPIO1_IRQHandler)
{
	Chip_PININT_ClearIntStatus(LPC_GPIO_PIN_INT, PININTCH1);
}

static unsigned char digitCount(unsigned int number){
	unsigned char count = 0;

	do {
		count++;
		number /= 10;
	} while (number != 0);

	return count;
}

static unsigned char * uintToString(unsigned int valor, unsigned char minCantDigitos, unsigned char * str){
	unsigned char cantDigitos = 1;
	char i;

	cantDigitos = digitCount(valor);

	if (minCantDigitos > cantDigitos)
		cantDigitos = minCantDigitos;

	for (i = cantDigitos; i > 0; i--){
		str[i-1] = '0' + valor % 10;
		valor /= 10;
	}

	str += cantDigitos;
	*str = '\0';

	return str;
}

static unsigned int countToRPM(unsigned int count){
	return (count/APERTURES)*(60*PERIOD)/10;
}

void addToArray(unsigned int count){
	countValues[length] = count;
	length = (length + 1) % ARRAYSIZE;
}

unsigned int averageCount(void){
	unsigned char index = 0;
	unsigned int sum = 0;
	for(index = 0; index < ARRAYSIZE; index++){
		sum = sum + countValues[index];
	}
	return sum/ARRAYSIZE;
}

TASK(PeriodicTask)
{
	unsigned char * p;
	unsigned char tmp;

	count = countToRPM(count);
	addToArray(count);
	count = averageCount();

	p = uintToString(count, 1, buffer);
	*p++ = '\r';
	*p++ = '\n';
	*p 	 = '\0';
	DEBUG_write_string(buffer);

	count = 0;

	ciaaPOSIX_read(fd_in, &tmp, 1);
	if (tmp & 0b00000001){
//		count = 0;
	}

	TerminateTask();
}
