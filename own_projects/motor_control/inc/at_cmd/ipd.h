#ifndef _IPD_H_
#define _IPD_H_

/*==================[inclusions]=============================================*/

#include "../parser.h"

/*==================[macros]=================================================*/

#undef PARSER_NAME
#define PARSER_NAME             IPD
#define PARSER_INTERNALDATA_T   PARSER_INTERNALDATA_TYPE(PARSER_NAME)
#define PARSER_RESULTS_T        PARSER_RESULTS_TYPE(PARSER_NAME)
#define PARSER_DATA_T           PARSER_DATA_TYPE(PARSER_NAME)
#define PARSER_DATA_IPD_T       PARSER_DATA_TYPE(IPD)
#define PARSER_RESULTS_IPD_T    PARSER_RESULTS_TYPE(IPD)

/*==================[typedef]================================================*/

typedef struct {
    ParserStatus    parserState;
    fsmStatus       state;
    uint16_t        readPos;
    uint16_t        writePos;
} PARSER_INTERNALDATA_T;

typedef struct {
    uint8_t *   buffer;
    uint16_t    bufferLength;
    uint16_t    payloadLength;
    uint8_t     connectionID;
} PARSER_RESULTS_T;


typedef struct {
    PARSER_INTERNALDATA_T   internalData;
    PARSER_RESULTS_T        results;
} PARSER_DATA_T;

/*==================[external data declaration]==============================*/

/*==================[external functions declaration]=========================*/

extern void parser_ipdModule_init(void);

#endif // _IPD_H_
