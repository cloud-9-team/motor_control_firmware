#include "pwm.h"
#include "chip.h"
#include "ciaaPOSIX_stdio.h"  /* <= device handler header */
#include "ciaaPOSIX_string.h"
#include "ciaak.h"            /* <= ciaa kernel header */
#include "os.h"               /* <= operating system header */
#include "StringUtils.h"
#include "ciaaDriverPwm.h"
#include "ciaaDriverPwm_Internal.h"


#define MIN_SPEED_HZ	4
#define MAX_SPEED_HZ	100
#define MIN_DUTY_CYCLE	40
#define MAX_DUTY_CYCLE	95

#define DIR_NEGATIVE	0
#define DIR_POSITIVE	1

#define MIN_PERIOD_MS	(1000 / MAX_SPEED_HZ)
#define MAX_PERIOD_MS	(1000 / MIN_SPEED_HZ)
#define STEP_SIZE 		((MAX_DUTY_CYCLE - MIN_DUTY_CYCLE) / ((1000 / MIN_SPEED_HZ) / 10))

uint8_t cantTicks = MAX_PERIOD_MS / MIN_PERIOD_MS;
uint8_t cuentaActual = 0;
uint8_t cuentaPaso = 0;
uint8_t direction = DIR_POSITIVE;
// uint8_t min_duty_cycle = 40;
uint32_t fd_pwm0;
uint32_t fd_pwm1;
uint32_t fd_pwm2;
uint32_t fd_pwm3;

uint32_t fd_in;
uint32_t fd_uart1;
uint8_t percent = MIN_DUTY_CYCLE;
char buffer[17] = "Duty cycle: xxx\r";


int main(void)
{
	StartOS(AppMode1);
	return 0;
}

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
	uint8_t min_duty_cycle = 80;
	/* init CIAA kernel and devices */
	ciaak_start();

	/* open UART connected to USB bridge (FT2232) */
	fd_uart1 = ciaaPOSIX_open("/dev/serial/uart/1", ciaaPOSIX_O_RDWR);

	/* change baud rate for uart usb */
	ciaaPOSIX_ioctl(fd_uart1, ciaaPOSIX_IOCTL_SET_BAUDRATE, (void *)ciaaBAUDRATE_115200);

	/* change FIFO TRIGGER LEVEL for uart usb */
	ciaaPOSIX_ioctl(fd_uart1, ciaaPOSIX_IOCTL_SET_FIFO_TRIGGER_LEVEL, (void *)ciaaFIFO_TRIGGER_LEVEL3);


	/* Opening of PWM channels and initilization */
	fd_pwm0 = ciaaPOSIX_open("/dev/dio/pwm/0",SCT_PWM_PIN_1A);
	fd_pwm1 = ciaaPOSIX_open("/dev/dio/pwm/1",SCT_PWM_PIN_4A);
	fd_pwm2 = ciaaPOSIX_open("/dev/dio/pwm/2",SCT_PWM_PIN_2A);
	fd_pwm3 = ciaaPOSIX_open("/dev/dio/pwm/3",SCT_PWM_PIN_3A);

	ciaaPOSIX_write(fd_pwm0,&min_duty_cycle,1);
	ciaaPOSIX_write(fd_pwm1,&min_duty_cycle,1);
	ciaaPOSIX_write(fd_pwm2,&min_duty_cycle,1);
	ciaaPOSIX_write(fd_pwm3,&min_duty_cycle,1);


	/* Initialize the SCT as PWM and set frequency */
	// Chip_SCTPWM_Init(SCT_PWM);
	// Chip_SCTPWM_SetRate(SCT_PWM, SCT_PWM_RATE);

	/* SCT output pin function selection */
	// Chip_SCU_PinMuxSet(7, 4, (SCU_MODE_INACT | SCU_MODE_FUNC1));
	// Chip_SCU_PinMuxSet(4, 3, (SCU_MODE_INACT | SCU_MODE_FUNC1));
	// Chip_SCU_PinMuxSet(4, 2, (SCU_MODE_INACT | SCU_MODE_FUNC1));
	// Chip_SCU_PinMuxSet(1, 5, (SCU_MODE_INACT | SCU_MODE_FUNC1));

	/* Associate output pin with match register */
	// Chip_SCTPWM_SetOutPin(SCT_PWM, SCT_PWM_CHANNEL_1A, SCT_PWM_PIN_1A);
	// Chip_SCTPWM_SetOutPin(SCT_PWM, SCT_PWM_CHANNEL_2A, SCT_PWM_PIN_2A);
	// Chip_SCTPWM_SetOutPin(SCT_PWM, SCT_PWM_CHANNEL_3A, SCT_PWM_PIN_3A);
	// Chip_SCTPWM_SetOutPin(SCT_PWM, SCT_PWM_CHANNEL_4A, SCT_PWM_PIN_4A);

	/* Start with minimum duty cycle */
	// Chip_SCTPWM_SetDutyCycle(SCT_PWM, SCT_PWM_CHANNEL_1A, Chip_SCTPWM_PercentageToTicks(SCT_PWM, MIN_DUTY_CYCLE));
	// Chip_SCTPWM_SetDutyCycle(SCT_PWM, SCT_PWM_CHANNEL_2A, Chip_SCTPWM_PercentageToTicks(SCT_PWM, MIN_DUTY_CYCLE));
	// Chip_SCTPWM_SetDutyCycle(SCT_PWM, SCT_PWM_CHANNEL_3A, Chip_SCTPWM_PercentageToTicks(SCT_PWM, MIN_DUTY_CYCLE));
	// Chip_SCTPWM_SetDutyCycle(SCT_PWM, SCT_PWM_CHANNEL_4A, Chip_SCTPWM_PercentageToTicks(SCT_PWM, MIN_DUTY_CYCLE));

	// Chip_SCTPWM_Start(SCT_PWM);

	fd_in = ciaaPOSIX_open("/dev/dio/in/0", ciaaPOSIX_O_RDWR);

	SetRelAlarm(ActivatePeriodicTask, 350, MIN_PERIOD_MS);

	/* terminate task */
	TerminateTask();
}

static void resetCount(void){
	cantTicks = MAX_PERIOD_MS / MIN_PERIOD_MS;
	cuentaActual = 0;
	cuentaPaso = 0;
}

TASK(PeriodicTask)
{
	uint8_t tmp;
	uint8_t oldPercent = percent;

	ciaaPOSIX_read(fd_in, &tmp, 1);

	if ((tmp & TEC_1) == TEC_1){

		if (direction == DIR_NEGATIVE){
			resetCount();
			direction = DIR_POSITIVE;
		}

		cuentaActual++;
		if (cuentaActual >= cantTicks){
			if (percent < MAX_DUTY_CYCLE){
				percent++;
			}
			cuentaActual = 0;
		}

		cuentaPaso++;
		if (cuentaPaso >= STEP_SIZE){
			if (cantTicks > 1)
				cantTicks--;
			cuentaPaso = 0;
		}

	}
	else{

		if (direction == DIR_POSITIVE){
			resetCount();
			direction = DIR_NEGATIVE;
		}

		cuentaActual++;
		if (cuentaActual >= cantTicks){
			if (percent > MIN_DUTY_CYCLE){
				percent--;
			}
			cuentaActual = 0;
		}

		cuentaPaso++;
		if (cuentaPaso >= STEP_SIZE){
			if (cantTicks > 1)
				cantTicks--;
			cuentaPaso = 0;
		}
	}

	if (oldPercent != percent){
		ciaaPOSIX_write(fd_pwm0,&percent,1);
		uintToString(percent, 3, (uint8_t *)&(buffer[12]));
		ciaaPOSIX_write(fd_uart1, buffer, 15);
		ciaaPOSIX_write(fd_uart1, "\r", 1);
	}

	TerminateTask();
}
