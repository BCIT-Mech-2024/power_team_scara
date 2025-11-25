/********************************************************* 
* tic_generator.h
**********************************************************/
#ifndef TIC_GENERATOR_H
#define TIC_GENERATOR_H

#include "windows.h"
#include <stdio.h>
#include <process.h>    /* _beginthread, _endthread */

void tic_setup(void (*function_name)(void),long time_interval);

void tic_enable(int enable);

#endif