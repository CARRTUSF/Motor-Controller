#pragma once

#include <vector>
#include <string>
#include "WMRA_structs.h"
#include "GalilController.h"

namespace WMRA {
	class MotorController
	{
	public:
		MotorController();
		~MotorController();
		bool isInitialized(); // return initialized
		bool motorsOn(); // Turns motors on
		bool motorsOff(); // Turns motors off
		bool Stop(); //emergancy stop
		bool Stop(int motorNum); // emergancy stop a single motor
		double readPos(int motorNum); // returns the current motor angle in radians
		std::vector<double> readPos(); // returns the current motor angle in radians
		double readPosErr(int motorNum); // returns the error in  
		bool definePosition(int motorNum, double angle); // Sets the selected motor to a certain encoder value
		bool positionControl(int motorNum,double angle);  // moves selected motor to absolute radian angle
		bool setMaxVelocity(int motorNum, double angularVelocity); // Sets the maximum angular velocity for selected motor
		bool setAccel(int motorNum, double angularAccelaration); // Sets the acceleration of the selected motor
		bool setDecel(int motorNum, double angularDecelaration); // Read setAccel()
		bool motorMode(int mode); // Changes the type of motor movement mode to selected value for all motors
		bool LIadd(std::vector<long> angle); // Linear Interpolation, Adds the input milestone to list of milestones
		bool LIbegin(); // Begins Linear Interpolation movement		
		void velocityMove(int motor, long int v_value, long int a_value, long int d_value); // Move a single motor a selected velocity, acceleration, and deceleration. CAUTION!: Use Stop() or velocityALLStop() to stop the motors
		bool setPID(int motorNum, int P, int I, int D); // Sets the PID values of a selected motor

	private:
		static std::string motorLookup[9]; // Lookup table that converts the user given motor number to a motor letter
		static bool initialized; // Flag that shows if motor controller has been properly initialized	
		static long rad2Enc[9]; // Conversion factors used to convert radians to encoder values
		static double enc2Rad[9]; // Conversion factors used to convert encoder values to radians

		static GalilController controller; // Galil controller class used to control WMRA-2
		//static ConfigReader reader; // reads defaults from settings.conf
		static bool isValidMotor(int motorNum); // Uses motorLookup t check if a selected motor is a valid motor. 1-8 [A-H]
		static bool MotorController::getDefaults(); // reads the default values from settings.conf
		double enc2rad(int motorNum, long enc);  // converts encoder number to radian value for selected joint number
		long rad2enc(int motorNum, double rad);  // converts radian value to encoder number for selected joint number
		static bool setMotorMode(int motorNum, int mode);
		static bool setBrushedMode(int motorNum, int mode);
		static bool setSmoothing(double value);
	};

}
