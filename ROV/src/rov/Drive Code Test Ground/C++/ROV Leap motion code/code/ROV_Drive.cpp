/******************************************************************************\
* Copyright (C) 2012-2014 Leap Motion, Inc. All rights reserved.               *
* Leap Motion proprietary and confidential. Not for distribution.              *
* Use subject to the terms of the Leap Motion SDK Agreement available at       *
* https://developer.leapmotion.com/sdk_agreement, or another agreement         *
* between Leap Motion and you, your company or other organization.             *
\******************************************************************************/
#include "ROV_Drive.h"
#include "Leap.h"

#ifdef USE_DIRECTX_SDK
#include <C:\Program Files (x86)\Microsoft DirectX SDK (June 2010)\include\xinput.h>
#pragma comment(lib,"xinput.lib")
#elif (_WIN32_WINNT >= 0x0602 /*_WIN32_WINNT_WIN8*/)
#include <XInput.h>
#pragma comment(lib,"xinput.lib")
#else
#include <XInput.h>
#pragma comment(lib,"xinput9_1_0.lib")
#endif


//-----------------------------------------------------------------------------
// Defines, constants, and global variables
//-----------------------------------------------------------------------------
#define MAX_CONTROLLERS 4  // XInput handles up to 4 controllers 
#define CONTROLLER_DEADZONE  ( 0.15f * FLOAT(0x7FFF) )  // Default to 24% of the +/- 32767 range.   This is a reasonable default value but can be altered if needed.
#define CONTROLLER1 0
#define NUMBER_OF_BUTTONS 20

#define NUMBER_OF_LEAP_INPUTS 6
#define LEAP_DEADZONE 50

struct CONTROLLER_STATE
{
	XINPUT_STATE state;
	bool bConnected;
};

CONTROLLER_STATE g_Controllers[MAX_CONTROLLERS];
ControllerInput Controller[NUMBER_OF_BUTTONS + 1];
WCHAR g_szMessage[4][1024] = { 0 };
HWND    g_hWnd;
bool    g_bDeadZoneOn = true;

LeapInput Leapvalues[NUMBER_OF_LEAP_INPUTS];
bool leapConnected = false;

int main()
{
	Leap::Controller controller;
	Leap::Frame frame;
	Leap::HandList hands;
	Leap::Hand h1;
	Leap::FingerList fingers;
	Leap::Finger index;
	Leap::Finger thumb;
	Leap::PointableList pointables;
	float indexX = 0, indexY = 0, indexZ = 0, thumbX = 0, thumbY = 0, thumbZ = 0, sum = 0, supersample[20];

	int i = 0;

	// Setup serial port connection and needed variables.
	HANDLE hSerial = CreateFile(L"COM6", GENERIC_WRITE | GENERIC_READ, 0, 0, OPEN_EXISTING, FILE_ATTRIBUTE_NORMAL, 0);

	if (hSerial != INVALID_HANDLE_VALUE)
	{
		printf("Port opened! \n");

		DCB dcbSerialParams;
		GetCommState(hSerial, &dcbSerialParams);

		dcbSerialParams.BaudRate = CBR_14400;
		dcbSerialParams.ByteSize = 8;
		dcbSerialParams.Parity = NOPARITY;
		dcbSerialParams.StopBits = ONESTOPBIT;

		SetCommState(hSerial, &dcbSerialParams);

		Sleep(1000);
	}

	else
	{
		if (GetLastError() == ERROR_FILE_NOT_FOUND)
		{
			printf("Serial port doesn't exist! \n");
		}

		printf("Error while setting up serial port! \n");
	}

	Controller[20].value = 9;	//Verification Byte sent to make sure everything else ends up in the right location
	FillByteSize();

	while (true)
	{
		UpdateControllerState();	//Updates all values on the controller
		WORD wButtons = g_Controllers[CONTROLLER1].state.Gamepad.wButtons;

		//Stores all of the values from the controller into the controller structure
		Controller[0].value = g_Controllers[CONTROLLER1].state.Gamepad.sThumbRX;
		Controller[1].value = g_Controllers[CONTROLLER1].state.Gamepad.sThumbRY;
		Controller[2].value = g_Controllers[CONTROLLER1].state.Gamepad.sThumbLX;
		Controller[3].value = g_Controllers[CONTROLLER1].state.Gamepad.sThumbLY;
		Controller[4].value = (g_Controllers[CONTROLLER1].state.Gamepad.bRightTrigger);
		Controller[5].value = (g_Controllers[CONTROLLER1].state.Gamepad.bLeftTrigger);
		Controller[6].value = (wButtons & XINPUT_GAMEPAD_RIGHT_THUMB);
		Controller[7].value = (wButtons & XINPUT_GAMEPAD_LEFT_THUMB);
		Controller[8].value = (wButtons & XINPUT_GAMEPAD_RIGHT_SHOULDER);
		Controller[9].value = (wButtons & XINPUT_GAMEPAD_LEFT_SHOULDER);
		Controller[10].value = (wButtons & XINPUT_GAMEPAD_DPAD_UP);
		Controller[11].value = (wButtons & XINPUT_GAMEPAD_DPAD_DOWN);
		Controller[12].value = (wButtons & XINPUT_GAMEPAD_DPAD_LEFT);
		Controller[13].value = (wButtons & XINPUT_GAMEPAD_DPAD_RIGHT);
		Controller[14].value = (wButtons & XINPUT_GAMEPAD_A);
		Controller[15].value = (wButtons & XINPUT_GAMEPAD_B);
		Controller[16].value = (wButtons & XINPUT_GAMEPAD_Y);
		Controller[17].value = (wButtons & XINPUT_GAMEPAD_X);
		Controller[18].value = (wButtons & XINPUT_GAMEPAD_START);
		Controller[19].value = (wButtons & XINPUT_GAMEPAD_BACK);

		CheckDeadZone();

		if (controller.isConnected() == true)
		{
			sum = 0;
			frame = controller.frame();
			hands = frame.hands();
			h1 = hands[0];
			fingers = frame.fingers();
			thumb = fingers[0];
			index = fingers[1];
			pointables = frame.pointables();

			Leapvalues[0].value = h1.palmVelocity().x;
			Leapvalues[1].value = h1.palmVelocity().y;
			Leapvalues[2].value = h1.palmVelocity().z;
			Leapvalues[3].value = h1.direction().pitch()*Leap::RAD_TO_DEG;
			Leapvalues[4].value = h1.direction().yaw()*Leap::RAD_TO_DEG;

			indexX = index.tipPosition().x;
			indexY = index.tipPosition().y;
			indexZ = index.tipPosition().z;

			thumbX = thumb.tipPosition().x;
			thumbY = thumb.tipPosition().y;
			thumbZ = thumb.tipPosition().z;

			Leapvalues[5].value = sqrt(pow((indexX - thumbX), 2) + pow((indexY - thumbY), 2) + pow((indexZ - thumbZ), 2));

			leapConnected = true;
			CheckLeapDeadZone();
		}



		for (i = 6; i < NUMBER_OF_BUTTONS; i++)	//DO NOT SET TO <= NUMBER_OF_BUTTONS, NOT A MISTAKE. Verification bit should always keep its value
		{
			{
				Controller[i].value = AnalogToDigital(Controller[i].value);	//converts all of the button presses on the controller to a binary value
			}
		}

		//turns all of the numerical values into buffers that can be passed to the arduino
		for (i = 0; i <= NUMBER_OF_BUTTONS; i++)
		{
			_itoa_s(Controller[i].value, Controller[i].passedValue, 10);
		}

		for (i = 0; i < NUMBER_OF_LEAP_INPUTS; i++)
		{
			_itoa_s(Leapvalues[i].value, Leapvalues[i].passedValue, 10);
		}

		SendData(hSerial);

		std::cout << Controller[8].value << std::endl;
	}
	return 0;
}

//-----------------------------------------------------------------------------
HRESULT UpdateControllerState()
{
	DWORD dwResult;
	for (DWORD w = 0; w < MAX_CONTROLLERS; w++)
	{
		// Simply get the state of the controller from XInput.
		dwResult = XInputGetState(w, &g_Controllers[w].state);

		if (dwResult == ERROR_SUCCESS)
			g_Controllers[w].bConnected = true;
		else
			g_Controllers[w].bConnected = false;
	}

	return S_OK;
}

//Defaulty, each controller button has its own respective value for high when pressed.
//Analog to digital converts them all to a binary value so less data needs to be sent to the arduino
int AnalogToDigital(int ControllerVal)
{
	if (ControllerVal != 0)
	{
		return 1;
	}

	else
		return 0;
}

//Tells the structure what the packet size should be for each piece of data
//analog sticks need 7 bytes, triggers take 4, buttons take 2
void FillByteSize()
{
	int i = 0;
	for (i = 0; i <= NUMBER_OF_BUTTONS; i++)
	{
		if (i <= 3)
			Controller[i].packetSize = 7;

		if (i >= 4 && i <= 5)
			Controller[i].packetSize = 4;

		if (i >= 6 && i <= NUMBER_OF_BUTTONS)
		{
			Controller[i].packetSize = 2;
		}
	}

	for (i = 0; i <= NUMBER_OF_LEAP_INPUTS; i++)
	{
		Leapvalues[i].packetSize = 10;
	}
}


//Checks the value to the analog sticks and converts them to zero if they are under the threshold
void CheckDeadZone()
{
	int i = 0;
	for (i = 0; i <= 3; i++)
	{
		if (abs(Controller[i].value) < CONTROLLER_DEADZONE)
		{
			Controller[i].value = 0;
		}
	}
}

void CheckLeapDeadZone()
{
	int i = 0;
	for (i = 0; i < 3; i++)
	{
		if (abs(Leapvalues[i].value) < LEAP_DEADZONE)
			Leapvalues[i].value = 0;
	}
}

void SendData(HANDLE hSerial)
{
	int i = 0;
	int count = 0;
	char ioControlBuffer[3];
	char ioControl[3];
	WriteFile(hSerial, "hgl", 4, NULL, NULL);
	while (true)
	{
		ReadFile(hSerial, ioControlBuffer, 2, NULL, NULL);

		ioControl[0] = ioControlBuffer[0];
		printf("%c\t", ioControl[0]);

		if (ioControl[0] == 'r')
		{
			for (i = 0; i <= NUMBER_OF_BUTTONS; i++)
				WriteFile(hSerial, Controller[i].passedValue, Controller[i].packetSize, NULL, NULL);

			if (leapConnected == true)
			{
				for (i = 0; i < NUMBER_OF_LEAP_INPUTS; i++)
					WriteFile(hSerial, Leapvalues[i].passedValue, Leapvalues[i].packetSize, NULL, NULL);
			}

			else
			{
				for (i = 0; i < NUMBER_OF_LEAP_INPUTS; i++)
					WriteFile(hSerial, "0", Leapvalues[i].packetSize, NULL, NULL);
			}

			count++;
			if (count > 5)
			{
				_sleep(750);
				break;
			}
		}

		else if (ioControl[0] == 'q')
		{
			break;
		}

		else
		{
			printf("Error: Unknown codeword passed from arduino [%c], aborting send\n", ioControl[0]);
		}
	}
}
void RecieveData(HANDLE hSerial)
{
	const int bufferSize = 10;
	int i = 0;
	char AcX[bufferSize], AcY[bufferSize], AcZ[bufferSize], Tmp[bufferSize], GyX[bufferSize], GyY[bufferSize], GyZ[bufferSize];
	//WriteFile(hSerial, "0", 8, NULL, NULL);
	ReadFile(hSerial, AcX, 8, NULL, NULL);
	/*ReadFile(hSerial, AcY, 8, NULL, NULL);
	ReadFile(hSerial, AcZ, 8, NULL, NULL);
	ReadFile(hSerial, Tmp, 8, NULL, NULL);
	ReadFile(hSerial, GyX, 8, NULL, NULL);
	ReadFile(hSerial, GyY, 8, NULL, NULL);
	ReadFile(hSerial, GyZ, 8, NULL, NULL);

	printf("AcX = %3s\t", AcX);
	printf("AcY = %3s\t", AcY);
	printf("AcZ = %3s\t", AcZ);
	printf("Tmp = %3s\t", Tmp);
	printf("GyX = %3s\t", GyX);
	printf("GyY = %3s\n", GyY);
	printf("GyZ = %3s\n", GyZ);*/
	std::cout << "AcX = " << AcX << std::endl;
}