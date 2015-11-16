#ifndef _DUTYCYCLE_H_
#define _DUTYCYCLE_H_

/*==================[inclusions]=============================================*/

#include "../parser.h"
#include "../pwm.h"

/*==================[macros]=================================================*/

#undef PARSER_NAME
#define PARSER_NAME             dutyCycle
#define PARSER_INTERNALDATA_T   PARSER_INTERNALDATA_TYPE(PARSER_NAME)
#define PARSER_RESULTS_T        PARSER_RESULTS_TYPE(PARSER_NAME)
#define PARSER_DATA_T           PARSER_DATA_TYPE(PARSER_NAME)
#define PARSER_DATA_DUTYCYCLE_T       PARSER_DATA_TYPE(dutyCycle)
#define PARSER_RESULTS_DUTYCYCLE_T    PARSER_RESULTS_TYPE(dutyCycle)

/*==================[typedef]================================================*/

typedef struct {
    ParserStatus	parserState;
    fsmStatus		state;
    uint16_t		dataLength;
} PARSER_INTERNALDATA_T;

typedef MotorControlData PARSER_RESULTS_T;


typedef struct {
    PARSER_INTERNALDATA_T   internalData;
    PARSER_RESULTS_T        results;
} PARSER_DATA_T;

/*==================[external data declaration]==============================*/

/*==================[external functions declaration]=========================*/

extern void parser_dutyCycleModule_init(void);

#endif // _DUTYCYCLE_H_
