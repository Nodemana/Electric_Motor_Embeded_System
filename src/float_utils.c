#include <stdint.h>
#include "utils/uartstdio.h"
#include "utils/ustdlib.h"


/// @brief Prints the given string and float value to the uart console.
/// @param message is the string to be printed, this function expects 
/// it to contain "%f" but will still work without it. It can only have 
/// one "%f".
/// @param length is the length of the string.
/// @param value is the float value to be interpolated into the given string.
void UartPrintFloat(char* message, uint32_t length, float value)
{
    // In the string, find the %f and replace with %d.%d%d.
    uint32_t size = length + 5;
    char new_string[size];
    char prev_char = ' ';
    uint32_t offset = 0;
    for(int i = 0; i < length; i++)
    {
        if(message[i] == 'f' && prev_char == '%'){
            // %f
            new_string[i] = 'd';
            new_string[i + 1] = '.';
            new_string[i + 2] = '%';
            new_string[i + 3] = 'd';
            new_string[i + 4] = '%';
            new_string[i + 5] = 'd';
            offset = 5;
        }
        else{
            new_string[i + offset] = message[i];
            prev_char = message[i];
        }
    }

    uint32_t msd = (uint32_t)value;
    uint32_t d1 = (uint32_t)(value * 10) - (msd * 10);
    uint32_t d2 = (uint32_t)(value * 100) - (msd * 100) - (d1 * 10);
    UARTprintf(new_string, msd, d1, d2);
}

void ftoa(char* message, char* output, uint32_t length, float value)
{
    // In the string, find the %f and replace with %d.%d%d.
    uint32_t size = length + 5;
    char new_string[size];
    char prev_char = ' ';
    uint32_t offset = 0;
    for(int i = 0; i < length; i++)
    {
        if(message[i] == 'f' && prev_char == '%'){
            // %f
            new_string[i] = 'd';
            new_string[i + 1] = '.';
            new_string[i + 2] = '%';
            new_string[i + 3] = 'd';
            new_string[i + 4] = '%';
            new_string[i + 5] = 'd';
            offset = 5;
        }
        else{
            new_string[i + offset] = message[i];
            prev_char = message[i];
        }
    }

    uint32_t msd = (uint32_t)value;
    uint32_t d1 = (uint32_t)(value * 10) - (msd * 10);
    uint32_t d2 = (uint32_t)(value * 100) - (msd * 100) - (d1 * 10);
    usprintf(output, new_string, msd, d1, d2);
}