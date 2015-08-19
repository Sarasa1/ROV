/******************************************************************************\
* Copyright (C) 2012-2014 Leap Motion, Inc. All rights reserved.               *
* Leap Motion proprietary and confidential. Not for distribution.              *
* Use subject to the terms of the Leap Motion SDK Agreement available at       *
* https://developer.leapmotion.com/sdk_agreement, or another agreement         *
* between Leap Motion and you, your company or other organization.             *
\******************************************************************************/
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif

//#undef UNICODE

#include "ROV_Drive.h"
#include "Leap.h"

#include <winsock2.h>
#include <ws2tcpip.h>


#pragma comment(lib, "Ws2_32.lib")
#pragma comment(lib, "Mswsock.lib")
#pragma comment(lib, "AdvApi32.lib")

#include "SerialFix.cpp"

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

#define NUMBER_OF_LEAP_INPUTS 7
#define LEAP_DEADZONE 50

#define PORT_NUM 3
#define BAUD 115200

#define DEFAULT_BUFLEN 512
#define DEFAULT_PORT "9995"


CSerial SerialPort;

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

char received[8][8];

char* hostName = "127.0.0.1";

using namespace std;
int main()
{
	//Leap Motion Vairables
	Leap::Controller controller;
	Leap::Frame frame;
	Leap::HandList hands;
	Leap::Hand h1;
	Leap::FingerList fingers;
	Leap::Finger index;
	Leap::Finger thumb;
	Leap::PointableList pointables;
	float indexX = 0, indexY = 0, indexZ = 0, thumbX = 0, thumbY = 0, thumbZ = 0, sum = 0;
	unsigned long cycles = 0;

	// TCP Variables
	WSADATA wsaData;
	SOCKET connectSocket = INVALID_SOCKET;

	struct addrinfo* result = NULL;
	struct addrinfo* ptr = NULL;
	struct addrinfo  hints;

	char cSendBuf[512][512];
	char sSendBuf[512];

	int iResult;
	int recvBufLen = DEFAULT_BUFLEN;

	int i = 0;

	iResult = WSAStartup(MAKEWORD(2, 2), &wsaData);
	if (iResult != 0) {
		printf("WSAStartup failed with error: %d\n", iResult);
		return 1;
	}

	// Initialize all address info to 0 to start.
	SecureZeroMemory(&hints, sizeof(hints));
	hints.ai_family = AF_UNSPEC;     // Doesn't matter if we use IPV4 or IPV6
	hints.ai_socktype = SOCK_STREAM; // TCP Stream sockets
	hints.ai_protocol = IPPROTO_TCP;

	// Resolve the server address and port
	iResult = getaddrinfo("127.0.0.1", DEFAULT_PORT, &hints, &result);
	if (iResult != 0) {
		printf("getaddrinfo failed with error: %d\n", iResult);
		WSACleanup();
		return 1;
	}

	// Attempt to connect to an address until one succeeds
	for (ptr = result; ptr != NULL; ptr = ptr->ai_next) {

		// create a socket for connecting to the server
		connectSocket = socket(ptr->ai_family, ptr->ai_socktype, ptr->ai_protocol);

		if (connectSocket == INVALID_SOCKET) {
			printf("socket failed with error: %ld\n", WSAGetLastError());
			WSACleanup();
			return 1;
		}

		// Connect to the server
		iResult = connect(connectSocket, ptr->ai_addr, (int)ptr->ai_addrlen);
		if (iResult == SOCKET_ERROR) {
			closesocket(connectSocket);
			connectSocket = INVALID_SOCKET;
			continue;
		}

		break;
	}

	// Deallocate the address info
	freeaddrinfo(result);

	if (connectSocket == INVALID_SOCKET) {
		printf("Unable to connect to server!\n");
		WSACleanup();
		return 1;
	}

	// Setup serial port connection and needed variables.
	SerialPort.Open(PORT_NUM, BAUD);

	Controller[20].value = 9;	//Verification Byte sent to make sure everything else ends up in the right location
	FillByteSize();

	while (true)
	{
		cycles++;
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
			Leapvalues[5].value = h1.direction().roll()*Leap::RAD_TO_DEG;

			indexX = index.tipPosition().x;
			indexY = index.tipPosition().y;
			indexZ = index.tipPosition().z;

			thumbX = thumb.tipPosition().x;
			thumbY = thumb.tipPosition().y;
			thumbZ = thumb.tipPosition().z;

			Leapvalues[6].value = sqrt(pow((indexX - thumbX), 2) + pow((indexY - thumbY), 2) + pow((indexZ - thumbZ), 2));

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

		/*
		for (i = 0; i < NUMBER_OF_BUTTONS; i++) {
			_itoa_s(Controller[i].value, cSendBuf[i], 10);
			cSendBuf[i][strlen(cSendBuf[i])] = '\0';

			iResult = send(connectSocket, cSendBuf[0], (int)strlen(cSendBuf[0]), 0);

			printf("String sent: %s\n", cSendBuf[0]);

			// Check for errors
			if (iResult == SOCKET_ERROR) {
				printf("send failed with error: %d\n", WSAGetLastError());
				closesocket(connectSocket);
				WSACleanup();
				return 1;
			}
		}

		// Try to send the packet
		iResult = send(connectSocket, "\n", (int)strlen("\n"), 0);

		//printf("String sent: %s\n", sendBuf);

		// Check for errors
		if (iResult == SOCKET_ERROR) {
			printf("send failed with error: %d\n", WSAGetLastError());
			closesocket(connectSocket);
			WSACleanup();
			return 1;
		}*/

		if (leapConnected = true)
		{
			for (i = 0; i < NUMBER_OF_LEAP_INPUTS; i++)
			{
				_itoa_s(Leapvalues[i].value, Leapvalues[i].passedValue, 10);
			}
		}

		/*Values recieved in this order:
		0) YAW
		1) PITCH
		2) ROLL
		3) ACCELERATION ON X AXIS
		4) ACCELERATION ON Y AXIS
		5) ACCELERATION ON Z AXIS
		6) SONAR SENSOR DISTANCE (IN METERS)
		*/
		if (SendData() == 1)
		{
			for (i = 0; i < 7; i++)
			{
				while (SerialPort.ReadDataWaiting() < 3)
				{
				}

				if (i > 2 && i < 6)
					SerialPort.ReadData(received[i], 5);

				else
					SerialPort.ReadData(received[i], 4);

				std::cout << received[i] << ' ';

				// Added this 6.25.14
				strcpy(sSendBuf, received[i]);
				sSendBuf[strlen(sSendBuf)] = '\0';
				iResult = send(connectSocket, sSendBuf, (int)strlen(sSendBuf), 0);

				printf("String sent: %s\n", sSendBuf);

				// Check for errors
				if (iResult == SOCKET_ERROR) {
					printf("send failed with error: %d\n", WSAGetLastError());
					closesocket(connectSocket);
					WSACleanup();
					return 1;
				}

				// Try to send the packet
				iResult = send(connectSocket, "\n", (int)strlen("\n"), 0);

				//printf("String sent: %s\n", sendBuf);

				// Check for errors
				if (iResult == SOCKET_ERROR) {
					printf("send failed with error: %d\n", WSAGetLastError());
					closesocket(connectSocket);
					WSACleanup();
					return 1;
				}
			}
		}
			
		
		
		//std::cout << recieved[0];
		printf("\t%d", cycles);
		printf("\n");
		//Sleep(500);  <<'\t' << recieved[1]

		
	}

	closesocket(connectSocket);
	WSACleanup();
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

	for (i = 0; i < NUMBER_OF_LEAP_INPUTS; i++)
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

int SendData()
{
	int i = 0;
	int count = 0;
	char ioControl[3];
	int bytesRecieved = 0;

	while (true)
	{
		bytesRecieved = SerialPort.ReadData(ioControl, 1);

		printf("%c %d\t", ioControl[0], bytesRecieved);
		if (ioControl[0] == 'r')
		{
			for (i = 0; i <= NUMBER_OF_BUTTONS; i++)
				SerialPort.SendData(Controller[i].passedValue, sizeof(Controller[i].passedValue));

			if (leapConnected == true)
			{
				for (i = 0; i < NUMBER_OF_LEAP_INPUTS; i++)
					SerialPort.SendData(Leapvalues[i].passedValue, sizeof(Leapvalues[i].passedValue));
			}

			else
			{
				for (i = 0; i < NUMBER_OF_LEAP_INPUTS; i++)
					SerialPort.SendData("0", 1);
			}

			//Sleep(59);
			count++;

			if (count > 5)
			{
				SerialPort.Close();
				Sleep(1000);
				SerialPort.Open(PORT_NUM, BAUD);
				return 0;
			}
		}

		else if (ioControl[0] == 'q')
		{
			return 1;
		}

		else
		{
			/*printf("Error: Unknown codeword passed from arduino [%c], aborting send\n", ioControl[0]);
			Sleep(60);*/
		}
	}
}