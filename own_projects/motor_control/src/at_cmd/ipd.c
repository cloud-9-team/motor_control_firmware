/*==================[inclusions]=============================================*/

#include "ipd.h"

/*==================[macros and definitions]=================================*/

/*==================[internal data declaration]==============================*/

/*==================[internal functions declaration]=========================*/

static void init(void* parserData);
static ParserStatus getStatus(void* parserData);
static void tryMatch(void* parserData, uint8_t newChar);
static void tryMatch_internal(PARSER_INTERNALDATA_T * internalData,
		PARSER_RESULTS_T * results,
		uint8_t newChar);
static void* getResults(void* parserData);

/*==================[internal data definition]===============================*/

static const ParserDefinition parserDef =
{
		&init,
		&getStatus,
		&tryMatch,
		&getResults,
		AT_MSG_IPD
};

/*==================[external data definition]===============================*/

/*==================[internal functions definition]==========================*/

static void init(void* parserData){
	PARSER_DATA_T * p = parserData;
	p->internalData.state = S0;
	p->internalData.readPos = 0;
	p->internalData.writePos = 0;
	p->internalData.parserState = STATUS_INITIALIZED;
}

static ParserStatus getStatus(void* parserData){
	return (((PARSER_DATA_T*)parserData)->internalData.parserState);
}

static void tryMatch(void* parserData, uint8_t newChar){
	PARSER_DATA_T * p = parserData;
	tryMatch_internal(&(p->internalData), &(p->results), newChar);
}

static void tryMatch_internal(PARSER_INTERNALDATA_T * internalData, PARSER_RESULTS_T * results, uint8_t newChar){
	ParserStatus ret = STATUS_NOT_MATCHES;

	switch(internalData->state){
	case S0: /* Leer caracteres que comienzan en el mensaje */
		if (newChar == "+IPD,"[internalData->readPos]){
			internalData->readPos++;
			ret = STATUS_INCOMPLETE;
			if ("+IPD,"[internalData->readPos] == '\0'){
				internalData->state = S1; /* TODO Si CIPMUX es 0, saltar a S3 */
			}
		}
		break;
	case S1: /* Matcheo de ID de conexión, de 0 a 4 */
		if (newChar >= '0' && newChar <= '4'){
			results->connectionID = newChar - '0';
			internalData->state = S2;
			ret = STATUS_INCOMPLETE;
		}
		break;
	case S2: /* Matcheo de ',' que sucede al ID de conexión */
		if (newChar == ','){
			internalData->state = S3;
			results->payloadLength = 0;
			ret = STATUS_INCOMPLETE;
		}
		break;
	case S3: /* Matcheo de longitud del mensaje (payload) */
		if (newChar == ':' && results->payloadLength > 0){
			internalData->state = S4;
			ret = STATUS_INCOMPLETE;
		}
		else if (newChar >= '0' && newChar <= '9'){
			results->payloadLength = (results->payloadLength * 10) + (newChar - '0');
			ret = STATUS_INCOMPLETE;
		}
		break;
	case S4: /* Lectura del payload */
		/* Escribe sólo si hay lugar en el buffer. */
		/* Si se llena, se continua leyendo caracteres, sin almacenarlos, hasta leer
		 * la cantidad especificada por la longitud del mensaje (payloadLength).
		 * El usuario debe comprobar si el buffer tuvo suficiente espacio para
		 * guardar todo el mensaje.
		 */
		if (internalData->writePos < results->bufferLength){
			results->buffer[internalData->writePos] = newChar;
		}

		internalData->writePos++;
		if (results->payloadLength == internalData->writePos){
			ret = STATUS_COMPLETE;
		}
		else{
			ret = STATUS_INCOMPLETE;
		}

		break;
	default:
		break;
	}

	if (ret == STATUS_NOT_MATCHES || ret == STATUS_COMPLETE){
		internalData->state = S0;
		internalData->readPos = 0;
		internalData->writePos = 0;
	}

	internalData->parserState = ret;
}

static void* getResults(void* parserData){
	return &(((PARSER_DATA_T*)parserData)->results);
}

/*==================[external functions definition]==========================*/

extern void parser_ipdModule_init(void){
	parser_add(&parserDef);
}
