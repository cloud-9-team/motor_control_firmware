/*==================[inclusions]=============================================*/

#include "parser.h"

/*==================[macros and definitions]=================================*/

/*==================[internal data declaration]==============================*/

/*==================[internal functions declaration]=========================*/

/*==================[internal data definition]===============================*/

static const ParserDefinition * parserFunction[PARSER_TYPES_COUNT];

/*==================[external data definition]===============================*/

/*==================[internal functions definition]==========================*/

/*==================[external functions definition]==========================*/

extern void parser_initModule(void){
    parser_ipdModule_init();
    parser_connectionOpenModule_init();
    parser_connectionCloseModule_init();
    parser_dutyCycleModule_init();
}

extern int32_t parser_init(const Parser * parser){
    if (    parser != 0 && parser->data != 0 &&
            parser->type < PARSER_TYPES_COUNT &&
            parserFunction[parser->type] != 0){

        parserFunction[parser->type]->init(parser->data);

        return 1;
    }

    return -1;
}

extern ParserStatus parser_getStatus(const Parser * parser){
    if (    parser != 0 && parser->data != 0 &&
            parser->type < PARSER_TYPES_COUNT &&
            parserFunction[parser->type] != 0){

        return parserFunction[parser->type]->getStatus(parser->data);
    }

    return STATUS_NOT_MATCHES;
}

extern ParserStatus parser_tryMatch(const Parser * parser, uint8_t newChar){
    if (    parser != 0 && parser->data != 0 &&
            parser->type < PARSER_TYPES_COUNT &&
            parserFunction[parser->type] != 0){

        parserFunction[parser->type]->tryMatch(parser->data, newChar);

        return parserFunction[parser->type]->getStatus(parser->data);
    }

    return STATUS_NOT_MATCHES;
}

extern void* parser_getResults(const Parser * parser){
    if (    parser != 0 && parser->data != 0 &&
            parser->type < PARSER_TYPES_COUNT &&
            parserFunction[parser->type] != 0){

        return parserFunction[parser->type]->getResults(parser->data);
    }

    return 0;
}

extern void parser_add(const ParserDefinition * newParser){
    if (    newParser != 0 &&
            newParser->type < PARSER_TYPES_COUNT){
        parserFunction[newParser->type] = newParser;
    }
}
