#ifndef _CONNECTION_CLOSE_H_
#define _CONNECTION_CLOSE_H_

/*==================[inclusions]=============================================*/

#include "../parser.h"

/*==================[macros]=================================================*/

#undef PARSER_NAME
#define PARSER_NAME              connectionClose
#define PARSER_INTERNALDATA_T    PARSER_INTERNALDATA_TYPE(PARSER_NAME)
#define PARSER_RESULTS_T         PARSER_RESULTS_TYPE(PARSER_NAME)
#define PARSER_DATA_T            PARSER_DATA_TYPE(PARSER_NAME)
#define PARSER_DATA_CONNECTIONCLOSE_T       PARSER_DATA_TYPE(connectionClose)
#define PARSER_RESULTS_CONNECTIONCLOSE_T    PARSER_RESULTS_TYPE(connectionClose)

/*==================[typedef]================================================*/

typedef struct {
    ParserStatus    parserState;
    fsmStatus       state;
    uint8_t         readPos;
} PARSER_INTERNALDATA_T;

typedef struct {
    uint8_t connectionID;
} PARSER_RESULTS_T;


typedef struct {
    PARSER_INTERNALDATA_T  internalData;
    PARSER_RESULTS_T       results;
} PARSER_DATA_T;

/*==================[external data declaration]==============================*/

/*==================[external functions declaration]=========================*/

extern void parser_connectionCloseModule_init(void);

#endif // _CONNECTION_CLOSE_H_
