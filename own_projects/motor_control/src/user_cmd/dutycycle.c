/*==================[inclusions]=============================================*/

#include "dutycycle.h"

/*==================[macros and definitions]=================================*/

/*==================[internal data declaration]==============================*/

/*==================[internal functions declaration]=========================*/

static void init(void* parserData);
static ParserStatus getStatus(void* parserData);
static void tryMatch(void* parserData, uint8_t newChar);
static void tryMatch_internal(	PARSER_INTERNALDATA_T * internalData,
								PARSER_RESULTS_T * results,
								uint8_t newChar);
static void* getResults(void* parserData);
static uint8_t parseReceivedDutyCycle(PARSER_RESULTS_T * results);

/*==================[internal data definition]===============================*/

static const ParserDefinition parserDef =
{
    &init,
    &getStatus,
    &tryMatch,
    &getResults,
    USER_DUTYCYCLE
};

/*==================[external data definition]===============================*/

/*==================[internal functions definition]==========================*/

static void init(void* parserData){
    PARSER_DATA_T * p = parserData;
    p->internalData.state = S0;
    p->internalData.dataLength = 0;
    p->internalData.parserState = STATUS_INITIALIZED;
}

static ParserStatus getStatus(void* parserData){
    return (((PARSER_DATA_T*)parserData)->internalData.parserState);
}

static void tryMatch(void* parserData, uint8_t newChar){
    PARSER_DATA_T * p = parserData;
    tryMatch_internal(&(p->internalData), &(p->results), newChar);
}

static void tryMatch_internal(	PARSER_INTERNALDATA_T * internalData,
								PARSER_RESULTS_T * results, uint8_t newChar){

	ParserStatus ret = STATUS_NOT_MATCHES;

	switch(internalData->state){
		case S0: /* Carácter separador entre el valor anterior y
					el Ciclo de Trabajo */
			if (newChar == '%'){
				internalData->state = S1;
				ret = STATUS_INCOMPLETE;
			}
			break;
		case S1: /* Lee la longitud de la cantidad de caracteres que
					contiene el mensaje siguiente: de 1 a 9 caracteres */
			if (newChar >= '1' && newChar <= '9'){
				internalData->dataLength = newChar - '0';
				internalData->state = S2;

				ret = STATUS_INCOMPLETE;
			}
			break;
		case S2: /* Lee el número de motor: de 0 a 9. */
			if (newChar >= '0' && newChar <= '9'){

				results->motorID = newChar - '0';
				results->dutyCycle = 0;

				internalData->dataLength--;
				internalData->state = S3;

				ret = STATUS_INCOMPLETE;
			}
			break;

		case S3: /* Se leen caracteres hasta leer la cantidad
					anteriormente especificada. Estos representan
					el ciclo de trabajo.
					Formato: de 0 a 200, de 100 a 200 se incrementa
					en un sentido, de 100 a 0 en el otro. */
			if (newChar >= '0' && newChar <= '9' && internalData->dataLength > 0){

				results->dutyCycle = (results->dutyCycle * 10) + newChar - '0';

				internalData->dataLength--;
				if (internalData->dataLength == 0){

					if (parseReceivedDutyCycle(results))
						ret = STATUS_COMPLETE;
					else
						ret = STATUS_NOT_MATCHES;

				}
				else
					ret = STATUS_INCOMPLETE;
			}
			break;
		default:
			break;
	}

	if (ret == STATUS_NOT_MATCHES || ret == STATUS_COMPLETE){
		internalData->state = S0;
	}

    internalData->parserState = ret;
}

static uint8_t parseReceivedDutyCycle(PARSER_RESULTS_T * results){
	if (results->dutyCycle >= 0 && results->dutyCycle <= 200){
		if (results->dutyCycle < 100){
			results->direction = DIR_BACKWARD;
			results->dutyCycle = 100 - results->dutyCycle;
		}
		else{
			results->direction = DIR_FORWARD;
			results->dutyCycle -= 100;
		}

		return 1;
	}

	return 0;
}

static void* getResults(void* parserData){
    return &(((PARSER_DATA_T*)parserData)->results);
}

/*==================[external functions definition]==========================*/

extern void parser_dutyCycleModule_init(void){
    parser_add(&parserDef);
}
