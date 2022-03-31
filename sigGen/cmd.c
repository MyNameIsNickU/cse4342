/*
 * cmd.c
 *
 *  Created on: Mar 25, 2022
 *      Author: Nicholas
 */


#include "cmd.h"
#include "uart0.h"


void getsUart0(USER_DATA* data)
{
    int count = 0;
    char c;

    while( 1 )
    {
        c = getcUart0();

        // If char c is a backspace (8 or 127), allows overriding of buffer
        if( c == 8 || c == 127 && count > 0)
            count--;
        // If the char c is readable (space, num, alpha), read to buffer
        else if( c >= 32 )
            data->buffer[count++] = c;

        // If return is hit or the MAX_CHARS limit reached, add the null and return
        if( c == 13 || count == MAX_CHARS)
        {
            data->buffer[count] = '\0';
            return;
        }
    }
}

// stores
void parseFields(USER_DATA* data)
{
    char c = '!';
    data->fieldCount = 0;
    bool isPrevDelim = true;
    uint8_t i;
    // Loop until we reach the end of the buffer
    for(i = 0; i < MAX_CHARS; i++)
    {
        c = data->buffer[i]; // Gets the char from the buffer
        if( c == '\0')
            return;

        if(isPrevDelim)
        {
            // if char c is alpha a-z LOWERCASE
            if( c >= 'a' && c <= 'z' )
            {
                data->fieldType[data->fieldCount] = 'a';
                data->fieldPosition[data->fieldCount] = i;
                data->fieldCount++;
                isPrevDelim = false;
            }

            // if char c is alpha A-Z UPPERCASE
            if ( c >= 'A' && c <= 'Z' )
            {
                data->fieldType[data->fieldCount] = 'A';
                data->fieldPosition[data->fieldCount] = i;
                data->fieldCount++;
                isPrevDelim = false;
            }

            // if char c is numeric
            if( c >= '0' && c <= '9')
            {
                data->fieldType[data->fieldCount] = 'n';
                data->fieldPosition[data->fieldCount] = i;
                data->fieldCount++;
                isPrevDelim = false;
            }

            // if c is float
            if( c == '.' )
            {
                data->fieldType[data->fieldCount] = 'f';
                data->fieldPosition[data->fieldCount] = i;
                data->fieldCount++;
                isPrevDelim = false;
            }
        }
        else
        {
            // only delimeters are SPACES and COMMAS
            if( c == ' ' || c == ',')
            {
                isPrevDelim = true;
                data->buffer[i] = '\0';
            }
            else if( data->fieldType[data->fieldCount-1] == 'n' && c == '.' )
                data->fieldType[data->fieldCount-1] = 'f';
        }

    }

}

// data holding command info...
// ...field# is which spot the command we want is
// 0 1 2 3...
char* getFieldString(USER_DATA* data, uint8_t fieldNumber)
{
    if(fieldNumber <= data->fieldCount)
        return &data->buffer[ data->fieldPosition[fieldNumber] ];
    else
        return '\0';
}

int32_t getFieldInteger(USER_DATA* data, uint8_t fieldNumber)
{
    char* strValue;
    int32_t returnVal = 0;
    uint8_t i;

    if(fieldNumber <= data->fieldCount && data->fieldType[fieldNumber] == 'n')
    {
        strValue = &data->buffer[ data->fieldPosition[fieldNumber] ];

        for(i = 0; strValue[i] != '\0'; ++i)
        {
            returnVal = returnVal * 10 + (strValue[i] - '0');
        }

        return returnVal;
    }

    else
        return -1;
}

float getFieldFloat(USER_DATA *data, uint8_t fieldNumber)
{
    char *strValue;
    float returnVal = -1;


    if(fieldNumber <= data->fieldCount && data->fieldType[fieldNumber] == 'f')
    {
        strValue = &data->buffer[ data->fieldPosition[fieldNumber] ];

        sscanf(strValue, "%f", &returnVal);
    }
    return returnVal;
}

bool strcomp(char * a, char * b)
{
    int8_t i = 0;
    char c1 = a[i];
    char c2 = b[i];

    do
    {
        if(c1 != c2)
            return false;
        else
        {
            i++;
            c1 = a[i];
            c2 = b[i];
        }
    } while(c1 != NULL || c2 != NULL);
    return true;
}

bool isCommand(USER_DATA* data, char strCommand[], uint8_t minArguments)
{
    char* strCompare = &data->buffer[ data->fieldPosition[0] ];

    if( minArguments >= data->fieldCount - 1 && strcomp(strCompare, strCommand) )
        return true;
    else
        return false;
}

void comm2str(instruction instruct, int index)
{
    char output[20];
    int16_t argument = instruct.argument;
    if(argument == 0xFFFF)
        argument = -1;
    switch(instruct.command)
    {
    case 0:
        if(instruct.argument == 0xFFFF)
            sprintf(output, "%d. forward", index+1);
        else
            sprintf(output, "%d. forward %d", index+1, instruct.argument );
        putsUart0(output);
        break;
        //putsUart0()
    case 1:
        if(instruct.argument == 0xFFFF)
            sprintf(output, "%d. reverse", index+1);
        else
            sprintf(output, "%d. reverse %d", index+1, instruct.argument);
        putsUart0(output);
        break;
    case 2:
        sprintf(output, "%d. cw %d", index+1, instruct.argument);
        putsUart0(output);
        break;
    case 3:
        sprintf(output, "%d. ccw %d", index+1, instruct.argument);
        putsUart0(output);
        break;
    case 4:
        if(instruct.argument == 0x1111)
            sprintf(output, "%d. wait pb", index+1);
        else if(instruct.argument == 0x2222)
            sprintf(output, "%d. wait distance %d", index+1, instruct.subcommand);
        putsUart0(output);
        break;
    case 5:
        sprintf(output, "%d. pause %d", index+1, instruct.argument);
        putsUart0(output);
        break;
    case 6:
        sprintf(output, "%d. stop", index+1);
        putsUart0(output);
        break;
    }
    putcUart0('\n');
    return;
}

instruction comm2instruct(USER_DATA comm)
{
    instruction returnStruct;
    if( isCommand(&comm, "forward", 2) )
    {
        returnStruct.command = 0;
        if( getFieldInteger(&comm, 1) == -1 )
            returnStruct.argument = 0xFFFF;
        else
            returnStruct.argument = getFieldInteger(&comm, 1);
    }

    if( isCommand(&comm, "reverse", 2) )
    {
        returnStruct.command = 1;
        if( getFieldInteger(&comm, 1) == -1 )
            returnStruct.argument = 0xFFFF;
        else
            returnStruct.argument = getFieldInteger(&comm, 1);
    }

    if( isCommand(&comm, "cw", 2) )
    {
        returnStruct.command = 2;
        if( getFieldInteger(&comm, 1) == -1 )
            returnStruct.argument = 0xFFFF;
        else
            returnStruct.argument = getFieldInteger(&comm, 1);
    }

    if( isCommand(&comm, "ccw", 2) )
    {
        returnStruct.command = 3;
        if( getFieldInteger(&comm, 1) == -1 )
            returnStruct.argument = 0xFFFF;
        else
            returnStruct.argument = getFieldInteger(&comm, 1);
    }

    if( isCommand(&comm, "wait", 2) )
    {
        returnStruct.command = 4;
        if( strcomp(getFieldString(&comm, 1), "pb") )
            returnStruct.argument = 0x1111;
        else
            returnStruct.argument = 0xFFFF;
    }

    if( isCommand(&comm, "pause", 2) )
    {
        returnStruct.command = 5;
        returnStruct.argument = getFieldInteger(&comm, 1);
    }

    if( isCommand(&comm, "stop", 1) )
    {
        returnStruct.command = 6;
        returnStruct.argument = getFieldInteger(&comm, 1);
    }

    return returnStruct;
}

void data_flush(USER_DATA * clear)
{
    int8_t i;
    for(i = 0; i < MAX_CHARS; i++)
        clear->buffer[i] = '\0';
    clear->fieldCount = 0;
    for(i = 0; i < MAX_FIELDS; i++)
    {
        clear->fieldPosition[i] = 0;
        clear->fieldType[i] = '\0';
    }

}

void instruct_insert(instruction * arr, instruction adding, uint8_t insert, int8_t index, bool max)
{
    uint8_t i;
    insert--;

    if(max)
    {
        for(i = MAX_INSTRUCTIONS-2; i >= insert; i--)
            arr[i+1] = arr[i];
        arr[insert] = adding;
    }
    else
    {
        if(insert >= index)
            return;

        for(i = index-1; i >= insert; i--)
            arr[i+1] = arr[i];
        arr[insert] = adding;
    }
    return;
}

void instruct_delete(instruction * arr, uint8_t remove, int8_t index, bool max)
{
    uint8_t i;
    remove--;

    if(max)
    {
        for(i = remove; i < MAX_INSTRUCTIONS-1; i++)
            arr[i] = arr[i+1];
    }
    else
    {
        if(remove >= index)
            return;

        for(i = remove; i < index-1; i++)
            arr[i] = arr[i+1];
    }
    return;
}
