// stdafx.h : include file for standard system include files,
// or project specific include files that are used frequently, but
// are changed infrequently
//

#pragma once
#define WIN32_LEAN_AND_MEAN

#include <stdio.h>
#include <tchar.h>
#include <iostream>
#include <stdlib.h>
#include <windows.h>
#include <commdlg.h>
#include <basetsd.h>
#include <objbase.h>
#include <string.h>
#include <math.h>

#define StrSize 8

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
void SendData(HANDLE hserial);
void RecieveData(HANDLE hSerial);