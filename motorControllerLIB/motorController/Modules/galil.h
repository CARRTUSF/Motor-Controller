
#ifdef GALIL_EXPORTS
#define GALIL_API __declspec(dllexport) 
#else
#define GALIL_API __declspec(dllimport) 
#endif


#ifndef GALIL_H
#define GALIL_H

#include <iostream>
#include <stdio.h>
#include <cstring>
#include <exception>
#include <string>
#include <sstream>
#include <vector>
#include <ctype.h>
//#include "SockStream.h"

/*
// windows includes
#include <winsock2.h>
#include <ws2tcpip.h>
#include <stdio.h>
#pragma comment(lib, "Ws2_32.lib")
*/

using namespace std;

class client_tcpsocket;

namespace galil
{
	class galilController {
	public:
		galilController();
		static __declspec(dllexport) bool initialize(string IP);
		static __declspec(dllexport) bool isInitialized(); // return initialized
		static __declspec(dllexport) string command(string Command);
		static __declspec(dllexport) int sID;

	private:
		static bool initialized ;
		string ipAddr;
		static __declspec(dllexport) int commandGalil(char* Command, char* Response, int ResponseSize);
		string toString(int val);
		string toString(float val);
		string toString(double val);
		static client_tcpsocket sock;
	};
}

#endif