/*==================[inclusions]=============================================*/

#include "encoder.h"
#include "chip.h"
#include "os.h"               /* <= operating system header */

/*==================[macros and definitions]=================================*/

#define GPIO_INTERRUPT	(LPC_GPIO_PIN_INT)

/*==================[internal data declaration]==============================*/

typedef struct {
	uint16_t	rpmValues[AVERAGE_WINDOW_SIZE];
	uint16_t	currentCount;
	uint8_t		nextIndex;
} encoderSampleData;

typedef struct {
	uint8_t	interruptChannel;
	uint8_t	portNumber;
	uint8_t	pinNumber;
	uint8_t diskSlotsNumber;
} encoderConfigData;


/*==================[internal functions declaration]=========================*/

static inline void encoder_configInterrupt(uint8_t intChannel, uint8_t portNum, uint8_t pinNum);
static inline uint32_t interruptCountToRPM(uint32_t intCount, uint8_t slots);
static uint32_t sampleAverage(uint8_t encoderID);


/*==================[internal data definition]===============================*/

static uint32_t fd_in;
static encoderSampleData encoder[ENCODER_COUNT];

static const encoderConfigData encoder0 = {
		0,			// Interrupt channel
		3, 0,		// GPIO0 -> GPIO3[0]
		2			// Number of slots in encoder disk
};

static const encoderConfigData encoder1 = {
		1,			// Interrupt channel
		3, 13,		// TEC_COL2 -> GPIO3[13]
		2			// Number of slots in encoder disk
};

static const encoderConfigData * encoderConfig[ENCODER_COUNT] = {
		&encoder0,
		&encoder1
};


/*==================[external data definition]===============================*/

/*==================[internal functions definition]==========================*/

static inline void encoder_configInterrupt(uint8_t intChannel, uint8_t portNum, uint8_t pinNum){
	Chip_SCU_GPIOIntPinSel(intChannel, portNum, pinNum);
	Chip_PININT_SetPinModeEdge(GPIO_INTERRUPT, PININTCH(intChannel));
	Chip_PININT_EnableIntLow(GPIO_INTERRUPT, PININTCH(intChannel));
}


static inline uint32_t interruptCountToRPM(uint32_t intCount, uint8_t slots){
	return (intCount * 1000 * 60) / (slots * SAMPLE_PERIOD);
}


static uint32_t sampleAverage(uint8_t encoderID){
	uint8_t index = 0;
	uint32_t sum = 0;
	for(index = 0; index < AVERAGE_WINDOW_SIZE; index++){
		sum = sum + encoder[encoderID].rpmValues[index];
	}
	return sum/AVERAGE_WINDOW_SIZE;
}


/*==================[external functions definition]==========================*/

extern void encoder_init(void)
{
	uint8_t encoderID;

	Chip_PININT_Init(GPIO_INTERRUPT);

//	Chip_SCU_PinMux(6,1,MD_PUP|MD_EZI|MD_ZI,FUNC0); /* GPIO3[0] -> GPIO0 en EDU-CIAA */
//	Chip_GPIO_SetDir(LPC_GPIO_PORT, 3, (1<<0), 0);
//
//	Chip_SCU_PinMux(7,5,MD_PUP|MD_EZI|MD_ZI,FUNC0); /* GPIO3[13] -> TEC_COL2 en EDU-CIAA */
//	Chip_GPIO_SetDir(LPC_GPIO_PORT, 3, (1<<13), 0);

	for (encoderID = 0; encoderID < ENCODER_COUNT; encoderID++){
		encoder_configInterrupt(encoderConfig[encoderID]->interruptChannel,
								encoderConfig[encoderID]->portNumber,
								encoderConfig[encoderID]->pinNumber);
	}


	fd_in = ciaaPOSIX_open("/dev/dio/in/0", ciaaPOSIX_O_RDONLY);

	SetRelAlarm(ActivateEncoderTask, 350, SAMPLE_PERIOD);
}

extern uint32_t encoder_getAverageRPM(uint8_t encoderID){

	if (encoderID < ENCODER_COUNT){
		return sampleAverage(encoderID);
	}
	else{
		return 0;
	}

}

TASK(EncoderTask)
{
	uint8_t encoderID;

	for (encoderID = 0; encoderID < ENCODER_COUNT; encoderID++){
		encoder[encoderID].rpmValues[encoder[encoderID].nextIndex] = interruptCountToRPM(encoder[encoderID].currentCount,
																						 encoderConfig[encoderID]->diskSlotsNumber);
		encoder[encoderID].nextIndex = (encoder[encoderID].nextIndex + 1) % AVERAGE_WINDOW_SIZE;

		encoder[encoderID].currentCount = 0;
	}

	TerminateTask();
}

ISR(GPIO0_IRQHandler)
{
	encoder[0].currentCount++;
	Chip_PININT_ClearIntStatus(GPIO_INTERRUPT, PININTCH(encoderConfig[0]->interruptChannel));
}

ISR(GPIO1_IRQHandler)
{
	encoder[1].currentCount++;
	Chip_PININT_ClearIntStatus(GPIO_INTERRUPT, PININTCH(encoderConfig[1]->interruptChannel));
}

