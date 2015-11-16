#include "pwm.h"

#include "chip.h"

uint32_t fd_pwm0;
uint32_t fd_pwm1;
uint32_t fd_pwm2;
uint32_t fd_pwm3;

/** \brief PWM init
 *
 * Initializes all four SCT channels for motor control.
 */
void ciaaPWM_init(void)
{
	uint8_t min_duty_cycle = 0;
	/* Initialize the SCT as PWM */
	// ciaaDriverPwm_init();

	/* Opening of PWM channels and initilization */
	fd_pwm0 = ciaaPOSIX_open("/dev/dio/pwm/0",SCT_PWM_PIN_1A);
	fd_pwm1 = ciaaPOSIX_open("/dev/dio/pwm/1",SCT_PWM_PIN_4A);
	fd_pwm2 = ciaaPOSIX_open("/dev/dio/pwm/2",SCT_PWM_PIN_2A);
	fd_pwm3 = ciaaPOSIX_open("/dev/dio/pwm/3",SCT_PWM_PIN_3A);

	// ciaaDriverPwm_open("pwm/0",ciaaDevices_getDevice("pwm/0"),SCT_PWM_PIN_1A);
	// ciaaDriverPwm_open("pwm/1",ciaaDevices_getDevice("pwm/1"),SCT_PWM_PIN_4A);
	// ciaaDriverPwm_open("pwm/2",ciaaDevices_getDevice("pwm/2"),SCT_PWM_PIN_2A);
	// ciaaDriverPwm_open("pwm/3",ciaaDevices_getDevice("pwm/3"),SCT_PWM_PIN_3A);

	ciaaPOSIX_write(fd_pwm0,&min_duty_cycle,1);
	ciaaPOSIX_write(fd_pwm1,&min_duty_cycle,1);
	ciaaPOSIX_write(fd_pwm2,&min_duty_cycle,1);
	ciaaPOSIX_write(fd_pwm3,&min_duty_cycle,1);

	// ciaaDriverPwm_write(ciaaDevices_getDevice("/dev/dio/pwm/0"),(uint8_t *)MIN_DUTY_CYCLE,0);
	// ciaaDriverPwm_write(ciaaDevices_getDevice("/dev/dio/pwm/1"),(uint8_t *)MIN_DUTY_CYCLE,0);
	// ciaaDriverPwm_write(ciaaDevices_getDevice("/dev/dio/pwm/2"),(uint8_t *)MIN_DUTY_CYCLE,0);
	// ciaaDriverPwm_write(ciaaDevices_getDevice("/dev/dio/pwm/3"),(uint8_t *)MIN_DUTY_CYCLE,0);
}

uint32_t getFileDescriptor(uint8_t fd_number){
	switch(fd_number){
	case 0:
		return fd_pwm0;
		break;
	case 1:
		return fd_pwm1;
		break;
	case 2:
		return fd_pwm2;
		break;
	case 3:
		return fd_pwm3;
		break;
	default:
		return -1;
	}
}
