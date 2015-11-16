#ifndef __ENCODER_H_
#define __ENCODER_H_

/*==================[inclusions]=============================================*/

#include "ciaaPOSIX_stdio.h"  /* <= device handler header */

/*==================[macros]=================================================*/

#define AVERAGE_WINDOW_SIZE		(10)
#define SAMPLE_PERIOD			(100)
#define ENCODER_COUNT			(2)

/*==================[typedef]================================================*/

/*==================[external data declaration]==============================*/

/*==================[external functions declaration]=========================*/

extern void encoder_init(void);
extern uint32_t encoder_getAverageRPM(uint8_t encoderID);

#endif /* __ENCODER_H_ */
