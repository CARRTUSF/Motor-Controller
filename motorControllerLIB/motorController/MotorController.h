
#ifndef MOTORCONTROLLER_H
#define MOTORCONTROLLER_H

// ArmControl.h

#include <stdio.h>
#include <cstring>
#include <exception>
#include <string>
#include <iostream>
#include <sstream>
#include <vector>
#include <ctype.h>

using namespace std;
using namespace galil;

namespace ArmController
{
	class MotorController
	{
	public:
		MotorController();
		static __declspec(dllexport) bool initialize();
		static __declspec(dllexport) bool isInitialized(); // return initialized
		static __declspec(dllexport) bool wmraSetup();
		static __declspec(dllexport) bool Stop(); //emergancy stop
		static __declspec(dllexport) bool Stop(int motorNum); // emergancy stop a single motor
		static __declspec(dllexport) float readPos(int motorNum); // returns the current motor angle in radians
		static __declspec(dllexport) float readPosErr(int motorNum); // returns the error in  
		static __declspec(dllexport) bool setMaxVelocity(int motorNum, float angularVelocity);
		static __declspec(dllexport) bool setAccel(int motorNum, float angularAccelaration);
		static __declspec(dllexport) bool setAccelAll(std::vector<int> acclVal);
		static __declspec(dllexport) bool setDecel(int motorNum, float angularDecelaration);
		static __declspec(dllexport) float encToAng(int motorNum, long enc);
		static __declspec(dllexport) long angToEnc(int motorNum, float angle);
		static __declspec(dllexport) bool positionControl(int motor,float angle);
		static __declspec(dllexport) bool MotorsOFF();
		static __declspec(dllexport) bool motorMode(int mode);
		static __declspec(dllexport) bool addLI(vector<long> angle);
		static __declspec(dllexport) bool beginLI();
		static galilController controller;

	private:
		static __declspec(dllexport) long rad2Enc[9];
		static __declspec(dllexport) bool initialized ;
		static __declspec(dllexport) string ipAddr;
		static __declspec(dllexport) double enc2Radian[9];
		//double rad2Enc[9];
		static __declspec(dllexport) double normalize[8]; // doesn't apply to gripper
		static __declspec(dllexport) string motorLookup[9];
		//bool readSettings(); // read settings 
		static __declspec(dllexport) bool definePosition(int motorNum, float angle);
		static __declspec(dllexport) bool setPID(int motorNum, int P, int I, int D);
		static __declspec(dllexport) bool isValidMotor(int motorNum);
		static __declspec(dllexport) string toString(int val);
		static __declspec(dllexport) string toString(float val);
		static __declspec(dllexport) string toString(double val);
	};
}

#endif;