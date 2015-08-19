// stdafx.h : include file for standard system include files,
// or project specific include files that are used frequently, but
// are changed infrequently
//

#pragma once

#include <stdio.h>
#include <iostream>
#include <stdlib.h>
#include <windows.h>
#include <string.h>

#define StrSize 10

typedef struct {

	int value;
	char passedValue[StrSize];
	int packetSize;

} ControllerInput;

typedef struct {
	float value;
	char passedValue[StrSize];
	int packetSize;
} LeapInput;

//-----------------------------------------------------------------------------
// Function-prototypes
//-----------------------------------------------------------------------------
HRESULT UpdateControllerState();
int AnalogToDigital(int);
void FillByteSize();
void CheckDeadZone();
void CheckLeapDeadZone();
int SendData();