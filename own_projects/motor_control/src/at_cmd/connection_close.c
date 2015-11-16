/*==================[inclusions]=============================================*/

#include "connection_close.h"

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
    AT_MSG_CONNECTION_CLOSE
};

/*==================[external data definition]===============================*/

/*==================[internal functions definition]==========================*/

static void init(void* parserData){
    PARSER_DATA_T * p = parserData;
    p->internalData.state = S0;
    p->internalData.readPos = 0;
    p->internalData.parserState = STATUS_INITIALIZED;
}

static ParserStatus getStatus(void* parserData){
    return (((PARSER_DATA_T*)parserData)->internalData.parserState);
}

static void tryMatch(void* parserData, uint8_t newChar){
    PARSER_DATA_T * p = parserData;
    tryMatch_internal(&(p->internalData), &(p->results), newChar);
}


static void tryMatch_internal(PARSER_INTERNALDATA_T * internalData,
                              PARSER_RESULTS_T * results,
                              uint8_t newChar){

    ParserStatus ret = STATUS_NOT_MATCHES;

    switch(internalData->state){
        case S0: /* Matcheo de ID de conexión, de 0 a 4 */
            if (newChar >= '0' && newChar <= '4'){
                results->connectionID = newChar - '0';
                internalData->state = S1;
                ret = STATUS_INCOMPLETE;
            }
            break;
        case S1: /* Matcheo de la cadena ",CLOSED" que debe sucederse */
            if (newChar == ",CLOSED"[internalData->readPos]){
                internalData->readPos++;
                if (",CLOSED"[internalData->readPos] == '\0'){
                    ret = STATUS_COMPLETE;
                }
                else{
                    ret = STATUS_INCOMPLETE;
                }
            }
            break;
        default:
        	break;
    }

    if (ret == STATUS_NOT_MATCHES || ret == STATUS_COMPLETE){
        internalData->state = S0;
        internalData->readPos = 0;
    }

    internalData->parserState = ret;
}

static void* getResults(void* parserData){
    return &(((PARSER_DATA_T*)parserData)->results);
}

/*==================[external functions definition]==========================*/

extern void parser_connectionCloseModule_init(void){
    parser_add(&parserDef);
}
