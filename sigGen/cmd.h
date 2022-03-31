/*
 * cmd.h
 *
 *  Created on: Mar 25, 2022
 *      Author: Nicholas
 */

#ifndef CMD_H_
#define CMD_H_

#define MAX_INSTRUCTIONS 10
#define MAX_CHARS 80
#define MAX_FIELDS 5

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>

typedef struct _USER_DATA
{
char buffer[MAX_CHARS+1];
uint8_t fieldCount;
uint8_t fieldPosition[MAX_FIELDS];
char fieldType[MAX_FIELDS];
} USER_DATA;

typedef struct _instruction
{
uint8_t command;
uint8_t subcommand;
uint16_t argument;
} instruction;

void getsUart0(USER_DATA* data);
void parseFields(USER_DATA* data);

char* getFieldString(USER_DATA* data, uint8_t fieldNumber);
int32_t getFieldInteger(USER_DATA* data, uint8_t fieldNumber);
float getFieldFloat(USER_DATA *data, uint8_t fieldNumber);

bool strcomp(char * a, char * b);
bool isCommand(USER_DATA* data, char strCommand[], uint8_t minArguments);

void comm2str(instruction instruct, int index);

instruction comm2instruct(USER_DATA comm);
void instruct_insert(instruction * arr, instruction adding, uint8_t insert, int8_t index, bool max);
void instruct_delete(instruction * arr, uint8_t remove, int8_t index, bool max);

void data_flush(USER_DATA * clear);

#endif /* CMD_H_ */
