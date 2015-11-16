#ifndef __ENCODER_H_
#define __ENCODER_H_


#ifdef __cplusplus
extern "C" {
#endif

#define LED_RGB_R	(1 << 0)
#define LED_RGB_G	(1 << 1)
#define LED_RGB_B	(1 << 2)
#define LED_RGB		(LED_RGB_R | LED_RGB_G | LED_RGB_B)
#define LED_1		(1 << 3)
#define LED_2		(1 << 4)
#define LED_3		(1 << 5)
#define LEDS		(LED_RGB | LED_1 | LED_2 | LED_3)
#define APERTURES	2
#define ARRAYSIZE	10
#define PERIOD		100

#include "ciaaPOSIX_stdio.h"  	/* <= device handler header */

void interruptConfiguration(uint32_t channel, uint8_t port, uint8_t pin);
static unsigned char digitCount(unsigned int number);
static unsigned char * uintToString(unsigned int valor, unsigned char minCantDigitos, unsigned char * str);
static unsigned int countToRPM(unsigned int count);
void addToArray(unsigned int count);
unsigned int averageCount(void);

#ifdef __cplusplus
}
#endif

#endif /* __ENCODER_H_ */
