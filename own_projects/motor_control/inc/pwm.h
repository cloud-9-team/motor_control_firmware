#ifndef __PWM_H_
#define __PWM_H_

/*==================[inclusions]=============================================*/

#include "ciaaPOSIX_stdio.h"  /* <= device handler header */

/*==================[macros]=================================================*/

#define SCT_PWM_PIN_1A		13		/* COUT13 controls 1A pin in L293D -> T_COL1 (P7_4) */
#define SCT_PWM_PIN_2A		3		/* COUT3  controls 2A pin in L293D -> T_FIL3 (P4_3) */
#define SCT_PWM_PIN_3A		0		/* COUT0  controls 3A pin in L293D -> T_FIL2 (P4_2) */
#define SCT_PWM_PIN_4A		10		/* COUT10 controls 4A pin in L293D -> T_COL0 (P1_5) */
#define SCT_PWM_CHANNEL_1A	1		/* Index of 1A PWM */
#define SCT_PWM_CHANNEL_2A	2		/* Index of 2A PWM */
#define SCT_PWM_CHANNEL_3A	3		/* Index of 3A PWM */
#define SCT_PWM_CHANNEL_4A	4		/* Index of 4A PWM */
#define SCT_PWM_RATE		10000	/* PWM frequency 10 KHz */
#define MIN_DUTY_CYCLE		0

/*==================[typedef]================================================*/

typedef enum {DIR_FORWARD, DIR_BACKWARD} MotorDirection;

typedef struct {
    uint8_t			motorID;
	uint8_t			dutyCycle;
	MotorDirection	direction;
} MotorControlData;

/*==================[external data declaration]==============================*/

/*==================[external functions declaration]=========================*/

extern void ciaaPWM_init(void);
extern void ciaaPWM_updateMotor(MotorControlData data);

#endif /* __PWM_H_ */
