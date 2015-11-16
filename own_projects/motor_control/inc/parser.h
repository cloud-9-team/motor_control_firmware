
#ifndef _PARSER_H_
#define _PARSER_H_

/*==================[inclusions]=============================================*/

#include "ciaaPOSIX_stdio.h"

/*==================[macros]=================================================*/

#define __PARSER_INTERNALDATA_TYPE(name)    parser_ ## name ## _internalData
#define __PARSER_RESULTS_TYPE(name)         parser_ ## name ## _results
#define __PARSER_DATA_TYPE(name)            parser_ ## name ## _data

#define PARSER_INTERNALDATA_TYPE(name)      __PARSER_INTERNALDATA_TYPE(name)
#define PARSER_RESULTS_TYPE(name)           __PARSER_RESULTS_TYPE(name)
#define PARSER_DATA_TYPE(name)              __PARSER_DATA_TYPE(name)

/*==================[typedef]================================================*/

typedef enum {S0, S1, S2, S3, S4, S5} fsmStatus;

typedef enum {STATUS_INITIALIZED, STATUS_INCOMPLETE, STATUS_NOT_MATCHES, STATUS_COMPLETE} ParserStatus;

typedef enum {
    AT_MSG_IPD = 0,
    AT_MSG_CONNECTION_OPEN,
    AT_MSG_CONNECTION_CLOSE,
    USER_DUTYCYCLE,
    PARSER_TYPES_COUNT
} ParserType;

typedef struct {
    ParserType  type;
    void*       data;
} Parser;

typedef struct {
    void            (*init)(void* parserData);
    ParserStatus    (*getStatus)(void* parserData);
    void            (*tryMatch)(void* parserData, uint8_t newChar);
    void*           (*getResults)(void* parserData);
    ParserType      type;
} ParserDefinition;

/*==================[inclusions]=============================================*/

#include "at_cmd/ipd.h"
#include "at_cmd/connection_open.h"
#include "at_cmd/connection_close.h"
#include "user_cmd/dutycycle.h"

/*==================[external data declaration]==============================*/

/*==================[external functions declaration]=========================*/

extern void parser_initModule(void);
extern int32_t parser_init(const Parser * parser);
extern ParserStatus parser_getStatus(const Parser * parser);
extern ParserStatus parser_tryMatch(const Parser * parser, uint8_t newChar);
extern void* parser_getResults(const Parser * parser);

extern void parser_add(const ParserDefinition * newParser);

#endif /* _PARSER_H_ */
