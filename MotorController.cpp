
#include <iostream>
#include <sstream>
#include "SockStream.h"
#include "ConfigReader.h"
#include "utility.h"
#include "MotorController.h"

using namespace std;

namespace WMRA{
	

	GalilController MotorController::controller;
	std::string MotorController::motorLookup[9] = {"Z","A","B","C","D","E","F","G","H"};
	bool MotorController::initialized;
	long MotorController::rad2Enc[9];
	double MotorController::enc2Rad[9];

	MotorController::MotorController()
	{
		cout << "Motor Controller initializing Galil" << endl;
		initialized = controller.initialize();
		if(initialized)
		{
			cout << "Motor Controller: setting defaults" << endl;
			initialized = getDefaults();
		}
		if(initialized)
		{
			cout << "Motor Controller: Initialize" << endl;
			initialized = motorsOn();
		}
		if(!initialized)
			cout << "Error: Failed to initialize Motor Controller" << endl;
		else
			cout << "Motor Controller: Initialized" << endl;
	}

	MotorController::~MotorController()
	{
		if(initialized)
			initialized = !motorsOff();
		controller.~galilController();
	}

	bool MotorController::isInitialized() // return initialized
	{
		return initialized;
	}

	bool MotorController::motorsOn()
	{
		controller.command("SH"); //turn on motors
		return true;
	}

	bool MotorController::motorsOff()
	{
		controller.command("MO"); //turn off motors
		return true;
	}

	bool MotorController::Stop() //emergancy stop
	{
		controller.command("ST ABCDEFGH");
		return true; // #debug does this return need to happen after the Arm has fully stopped?
	}

	bool MotorController::Stop(int motorNum) // emergancy stop a single motor
	{
		if(isValidMotor(motorNum)){
			std::string motor = motorLookup[motorNum];
			controller.command("ST " + motor);
			return true;
		}
		else{
			return false;
		}
	}

	double MotorController::readPos(int motorNum) // returns the current motor angle in radians
	{
		long encoderVal;	
		std::string result;
		std::string motor;
		if ( isValidMotor(motorNum)){
			motor = motorLookup[motorNum];
			result = controller.command( "TP" + motor);	
			std::istringstream stream(result);
			stream >> encoderVal;
			return enc2rad(motorNum, encoderVal);        
		}
		else{
			if(motorNum != 9 && motorNum != 10)
			{
				std::cerr << "The motor specified is not valid" << std::endl;
				throw std::out_of_range ("MotorNum out_of_range");
			}
		}
	}

	vector<double> MotorController::readPos() // returns the current motor angle in radians
	{
		vector<double> positions;
		for(int i=1; i<9; i++)
			positions.push_back(readPos(i));
		return positions;
	}

	double MotorController::readPosErr(int motorNum) // returns the error in  
	{

		long encoderVal;	
		std::string result;
		std::string motor;
		if ( isValidMotor(motorNum)){
			motor = motorLookup[motorNum];
			result = controller.command( "TE" + motor);	
			std::istringstream stream(result);
			stream >> encoderVal;
			return enc2rad(motorNum, encoderVal);        
		}
		else{
			if(motorNum != 9 && motorNum != 10)
			{
				std::cerr << "The motor specified is not valid" << std::endl;
				throw std::out_of_range ("MotorNum out_of_range");
			}
		}
	}

	bool MotorController::definePosition(int motorNum, double angle)
	{
		if(isValidMotor(motorNum)){
			long encVal = abs(rad2enc(motorNum,angle));
			std::string motor = motorLookup[motorNum];
			if ((encVal >= -2147483647) && (encVal <= 2147483648)){
				std::string command = "DP" + motor;
				controller.command(command + "=" + utility::toString(encVal));
				return true;            
			}
			else{
				std::cerr << "The Position is outside the range" << std::endl;
				throw std::out_of_range ("Position out_of_range");            
			}
		}
		else{
			if(motorNum != 9 && motorNum != 10)
			{
				std::cerr << "The motor specified is not valid" << std::endl;
				throw std::out_of_range ("MotorNum out_of_range");
			}
		}
	}

	bool MotorController::positionControl(int motorNum,double angle)
	{
		if(isValidMotor(motorNum)){
			long encVal = (rad2enc(motorNum,angle));
			std::cout << encVal << std::endl;
			std::string motor;
			motor = motorLookup[motorNum];
			long temp = abs(encVal);
			if (temp <= 2147483648){
				std::string command = "PA" + motor + "=" + utility::toString(encVal);
				try	{
					controller.command(command);
				}
				catch(std::string s){
					std::cout << s << std::endl;
					std::cout << std::endl;
				}
				/*
				string command2 = "BG " + motor;
				try	{
				controller.command(command2);
				}
				catch(string s){
				cout << s << endl;
				cout << endl;
				}
				*/
				return true;            
			}
			else{
				std::cerr << "The Position is outside the range" << std::endl;
				throw std::out_of_range ("Position out_of_range");     
				return false;
			}
		}
		else{
			if(motorNum != 9 && motorNum != 10)
			{
				std::cerr << "The motor specified is not valid" << std::endl;
				throw std::out_of_range ("MotorNum out_of_range");
			}
		}
	}

	bool MotorController::setMaxVelocity(int motorNum, double angularVelocity)
	{
		if(isValidMotor(motorNum)){
			long encVal = abs(rad2enc(motorNum,angularVelocity));
			std::string motor = motorLookup[motorNum];
			if ((encVal >= 0) && (encVal < 12000000)){
				std::string command = "SP" + motor;
				controller.command(command + "=" + utility::toString(encVal));
				return true;            
			}
			else{
				std::cerr << "The velocity is outside the range" << std::endl;
				throw std::out_of_range ("velocity out_of_range");            
			}
		}
		else{
			if(motorNum != 9 && motorNum != 10)
			{
				std::cerr << "The motor specified is not valid" << std::endl;
				throw std::out_of_range ("MotorNum out_of_range");
			}
		}
	}

	bool MotorController::setAccel(int motorNum, double angularAccelaration)
	{	
		if(isValidMotor(motorNum)){
			long encVal = abs(rad2enc(motorNum,angularAccelaration));
			std::string motor = motorLookup[motorNum];
			if ((encVal >= 1024) && (encVal <= 67107840)){
				std::string command = "AC" + motor;
				controller.command(command + "=" + utility::toString(encVal));
				return true;            
			}
			else{
				std::cerr << "The Accelaration is outside the range" << std::endl;
				throw std::out_of_range ("Accelaration out_of_range");            
			}
		}
		else{
			if(motorNum != 9 && motorNum != 10)
			{
				std::cerr << "The motor specified is not valid" << std::endl;
				throw std::out_of_range ("MotorNum out_of_range");
			}
		}

	}

	bool MotorController::setDecel(int motorNum, double angularDecelaration)
	{
		if(isValidMotor(motorNum)){
			long encVal = abs(rad2enc(motorNum,angularDecelaration));
			std::string motor = motorLookup[motorNum];
			if ((encVal >= 1024) && (encVal <= 67107840)){
				std::string command = "DC" + motor;
				controller.command(command + "=" + utility::toString(encVal));
				return true;            
			}
			else{
				std::cerr << "The Decelaration is outside the range" << std::endl;
				throw std::out_of_range ("Decelaration out_of_range");            
			}
		}
		else{
			if(motorNum != 9 && motorNum != 10)
			{
				std::cerr << "The motor specified is not valid" << std::endl;
				throw std::out_of_range ("MotorNum out_of_range");
			}
		}

	}

	bool MotorController::motorMode(int mode) // 0=Position Tracking, 1=Linear Interpolation
	{
		if(mode == 0)
		{
			//set all motors to position tracking mode
			controller.command("PTA=1");
			controller.command("PTB=1");
			controller.command("PTC=1");
			controller.command("PTD=1");
			controller.command("PTE=1");
			controller.command("PTF=1");
			controller.command("PTG=1");
			controller.command("PTH=1");
		}
		else if(mode == 1)
		{
			//set all motors to linear interpolation mode
			controller.command("LM ABCDEFGH"); // galil manual pg.88 (pdf pg.98) 
			controller.command("CAS"); // or ("CAT") - Specifying the Coordinate Plane
		}
		else
			return 0;
		return 1;
	}

	bool MotorController::LIadd(std::vector<long> angle)
	{	
		if(angle.size() == 8)
		{
			char temp[100];
			std::string Command = "LI ";
			_ltoa(angle[0],temp,10);
			Command = Command + temp;
			Command = Command + ",";
			_ltoa(angle[1],temp,10);
			Command = Command + temp;
			Command = Command + ",";
			_ltoa(angle[2],temp,10);  
			Command = Command + temp;
			Command = Command + ",";
			_ltoa(angle[3],temp,10); 
			Command = Command + temp;
			Command = Command + ",";
			_ltoa(angle[4],temp,10);
			Command = Command + temp;
			Command = Command + ",";
			_ltoa(angle[5],temp,10); 
			Command = Command + temp;
			Command = Command + ",";
			_ltoa(angle[6],temp,10);
			Command = Command + temp;
			Command = Command + ",";
			_ltoa(angle[7],temp,10);  
			Command = Command + temp;

			controller.command(Command);
		}
		else
		{
			std::cout << "Error: LI takes 8 joint angles" << std::endl;
			return 0;
		}
		return 1;
	}

	bool MotorController::LIbegin()
	{
		controller.command("LE"); // Linear End, for smooth stopping
		controller.command("BGS"); // Begin Sequence
		return 1;
	}

	void MotorController::velocityMove(int motorNum, long int v_value, long int a_value, long int d_value)
	{
		string command1 = "#";
		if(isValidMotor(motorNum)){
			std::string motor;
			motor = motorLookup[motorNum];
			command1 = command1 + motor;
			//controller.command(command1);  // #DEBUG: IF found its not needed

			if (v_value != 0)
			{
				command1 = "JG";
				command1 = command1 + motor;
				command1 = command1 + "=";

				ostringstream buffer1;
				buffer1 << v_value;
				string str1 = buffer1.str();
				command1 = command1 + str1;
				controller.command(command1);
			}

			if (a_value != 0)
			{
				command1 = "AC";
				command1 = command1 + motor;
				command1 = command1 + "=";
				ostringstream buffer1;
				buffer1 << a_value;
				string str1 = buffer1.str();
				command1 = command1 + str1;
				controller.command(command1);
			}

			if (d_value != 0)
			{
				command1 = "DC";
				command1 = command1 + motor;
				command1 = command1 + "=";
				ostringstream buffer1;
				buffer1 << d_value;
				string str1 = buffer1.str();
				command1 = command1 + str1;
				controller.command(command1);
			}

			command1 = "BG ";
			command1 = command1 + motor;
			controller.command(command1);
		}
	}

	bool MotorController::setPID(int motorNum, int P, int I, int D){ // #DEBUG set pid values
		return false;
	}

	bool MotorController::setMotorMode(int motorNum, int mode)
	{
		string command1 = "PT";
		if(isValidMotor(motorNum)){
			std::string motor;
			motor = motorLookup[motorNum];
			command1 = command1 + motor;
			command1 = command1 + "=";
			ostringstream buffer1;
			buffer1 << mode;
			motor = buffer1.str();
			command1 = command1 + motor;

			controller.command(command1);
		}
		else
			return 0;
		return 1;
	}

	bool MotorController::setBrushedMode(int motorNum, int mode)
	{
		string command1 = "BR";
		if(isValidMotor(motorNum)){
			std::string motor;
			motor = motorLookup[motorNum];
			command1 = command1 + motor;
			command1 = command1 + "=";
			ostringstream buffer1;
			buffer1 << mode;
			motor = buffer1.str();
			command1 = command1 + motor;

			controller.command(command1);
		}
		else
			return 0;
		return 1;
	}

	bool MotorController::setSmoothing(double value)
	{
		string command1 = "IT*=";
		std::string motor;
		ostringstream buffer1;
		buffer1 << value;
		motor = buffer1.str();
		command1 = command1 + motor;

		controller.command(command1);

		return 1;
	}


	//
	//
	// Private Functions
	//
	//
	inline bool MotorController::isValidMotor(int motorNum){
		if( motorNum >= 1 && motorNum <= 8) return true;
		else return false;
	}

	bool MotorController::getDefaults()
	{
		ConfigReader reader;
		reader.parseFile("settings.conf");
		reader.setSection("MOTOR_CONTROLLER_DEFAULTS");

		if(reader.keyPresent("encoderPerRevolution1")){
			enc2Rad[1] = 2*PI/reader.getInt("encoderPerRevolution1"); //calculate conversion values
			rad2Enc[1] = 1/enc2Rad[1];
		}
		else{
			cout << "'encoderPerRevolution1' default not found" << endl;			
			return 0;
		}

		if(reader.keyPresent("encoderPerRevolution2")){
			enc2Rad[2] = 2*PI/reader.getInt("encoderPerRevolution2"); //calculate conversion values
			rad2Enc[2] = 1/enc2Rad[2];
		}
		else{
			cout << "'encoderPerRevolution2' default not found" << endl;			
			return 0;
		}

		if(reader.keyPresent("encoderPerRevolution3"))
		{
			enc2Rad[3] = 2*PI/reader.getInt("encoderPerRevolution3"); //calculate conversion values
			rad2Enc[3] = 1/enc2Rad[3];
		}
		else
		{
			cout << "'encoderPerRevolution3' default not found" << endl;

			return 0;
		}

		if(reader.keyPresent("encoderPerRevolution4"))
		{
			enc2Rad[4] = 2*PI/reader.getInt("encoderPerRevolution4"); //calculate conversion values
			rad2Enc[4] = 1/enc2Rad[4];
		}
		else
		{
			cout << "'encoderPerRevolution4' default not found" << endl;

			return 0;
		}

		if(reader.keyPresent("encoderPerRevolution5"))
		{
			enc2Rad[5] = 2*PI/reader.getInt("encoderPerRevolution5"); //calculate conversion values
			rad2Enc[5] = 1/enc2Rad[5];
		}
		else
		{
			cout << "'encoderPerRevolution5' default not found" << endl;

			return 0;
		}

		if(reader.keyPresent("encoderPerRevolution6"))
		{
			enc2Rad[6] = 2*PI/reader.getInt("encoderPerRevolution6"); //calculate conversion values
			rad2Enc[6] = 1/enc2Rad[6];
		}
		else
		{
			cout << "'encoderPerRevolution6' default not found" << endl;

			return 0;
		}

		if(reader.keyPresent("encoderPerRevolution7"))
		{
			enc2Rad[7] = 2*PI/reader.getInt("encoderPerRevolution7"); //calculate conversion values
			rad2Enc[7] = 1/enc2Rad[7];
		}
		else
		{
			cout << "'encoderPerRevolution7' default not found" << endl;

			return 0;
		}

		if(reader.keyPresent("encoderPerRevolution8"))
		{
			enc2Rad[8] = 2*PI/reader.getInt("encoderPerRevolution8"); //calculate conversion values
			rad2Enc[8] = 1/enc2Rad[8];
		}
		else
		{
			cout << "'encoderPerRevolution8' default not found" << endl;

			return 0;
		}

		// set motor type to brushed motor
		if(reader.keyPresent("brushedMotor1"))
			setBrushedMode(1, reader.getInt("brushedMotor1"));
		else
		{
			cout << "'brushedMotor1' default not found" << endl;

			return 0;
		}

		if(reader.keyPresent("brushedMotor2"))
			setBrushedMode(2, reader.getInt("brushedMotor2")); 
		else
		{
			cout << "'brushedMotor2' default not found" << endl;

			return 0;
		}

		if(reader.keyPresent("brushedMotor3"))
			setBrushedMode(3, reader.getInt("brushedMotor3"));
		else
		{
			cout << "'brushedMotor3' default not found" << endl;

			return 0;
		}

		if(reader.keyPresent("brushedMotor4"))
			setBrushedMode(4, reader.getInt("brushedMotor4")); 
		else
		{
			cout << "'brushedMotor4' default not found" << endl;

			return 0;
		}

		//set all motors to position tracking mode
		if(reader.keyPresent("motorMode1"))
			setMotorMode(1, reader.getInt("motorMode1"));
		else
		{
			cout << "'motorMode1' default not found" << endl;

			return 0;
		}

		if(reader.keyPresent("motorMode2"))
			setMotorMode(2, reader.getInt("motorMode2"));
		else
		{
			cout << "'motorMode2' default not found" << endl;

			return 0;
		}

		if(reader.keyPresent("motorMode3"))
			setMotorMode(3, reader.getInt("motorMode3"));
		else
		{
			cout << "'motorMode3' default not found" << endl;

			return 0;
		}

		if(reader.keyPresent("motorMode4"))
			setMotorMode(4, reader.getInt("motorMode4"));
		else
		{
			cout << "'motorMode4' default not found" << endl;

			return 0;
		}

		if(reader.keyPresent("motorMode5"))
			setMotorMode(5, reader.getInt("motorMode5"));
		else
		{
			cout << "'motorMode5' default not found" << endl;

			return 0;
		}

		if(reader.keyPresent("motorMode6"))
			setMotorMode(6, reader.getInt("motorMode6"));
		else
		{
			cout << "'motorMode6' default not found" << endl;

			return 0;
		}

		if(reader.keyPresent("motorMode7"))
			setMotorMode(7, reader.getInt("motorMode7")); 
		else
		{
			cout << "'motorMode7' default not found" << endl;

			return 0;
		}

		if(reader.keyPresent("motorMode8"))
			setMotorMode(8, reader.getInt("motorMode8"));
		else
		{
			cout << "'motorMode8' default not found" << endl;

			return 0;
		}

		// Set Max Velocity
		if(reader.keyPresent("maxVelocity1"))
			setMaxVelocity(1, reader.getDouble("maxVelocity1")); 
		else
		{
			cout << "'maxVelocity1' default not found" << endl;

			return 0;
		}

		if(reader.keyPresent("maxVelocity2"))
			setMaxVelocity(2, reader.getDouble("maxVelocity2")); 
		else
		{
			cout << "'maxVelocity2' default not found" << endl;

			return 0;
		}

		if(reader.keyPresent("maxVelocity3"))
			setMaxVelocity(3, reader.getDouble("maxVelocity3")); 
		else
		{
			cout << "'maxVelocity3' default not found" << endl;

			return 0;
		}

		if(reader.keyPresent("maxVelocity4"))
			setMaxVelocity(4, reader.getDouble("maxVelocity4")); 
		else
		{
			cout << "'maxVelocity4' default not found" << endl;

			return 0;
		}

		if(reader.keyPresent("maxVelocity5"))
			setMaxVelocity(5, reader.getDouble("maxVelocity5")); 
		else		
		{
			cout << "'maxVelocity5' default not found" << endl;

			return 0;
		}

		if(reader.keyPresent("maxVelocity6"))
			setMaxVelocity(6, reader.getDouble("maxVelocity6")); 
		else
		{
			cout << "'maxVelocity6' default not found" << endl;

			return 0;
		}

		if(reader.keyPresent("maxVelocity7"))
			setMaxVelocity(7, reader.getDouble("maxVelocity7")); 
		else
		{
			cout << "'maxVelocity7' default not found" << endl;

			return 0;
		}

		if(reader.keyPresent("maxVelocity8"))
			setMaxVelocity(8, reader.getDouble("maxVelocity8")); 
		else
		{
			cout << "'maxVelocity8' default not found" << endl;

			return 0;
		}

		//Set Acceleration
		if(reader.keyPresent("acceleration1"))
			setAccel(1, reader.getDouble("acceleration1")); 
		else
		{
			cout << "'acceleration1' default not found" << endl;

			return 0;
		}

		if(reader.keyPresent("acceleration2"))
			setAccel(2, reader.getDouble("acceleration2")); 
		else
		{
			cout << "'acceleration2' default not found" << endl;

			return 0;
		}

		if(reader.keyPresent("acceleration3"))
			setAccel(3, reader.getDouble("acceleration3")); 
		else
		{
			cout << "'acceleration3' default not found" << endl;

			return 0;
		}

		if(reader.keyPresent("acceleration4"))
			setAccel(4, reader.getDouble("acceleration4")); 
		else
		{
			cout << "'acceleration4' default not found" << endl;

			return 0;
		}

		if(reader.keyPresent("acceleration5"))
			setAccel(5, reader.getDouble("acceleration5")); 
		else		
		{
			cout << "'acceleration5' default not found" << endl;

			return 0;
		}

		if(reader.keyPresent("acceleration6"))
			setAccel(6, reader.getDouble("acceleration6")); 
		else
		{
			cout << "'acceleration6' default not found" << endl;

			return 0;
		}

		if(reader.keyPresent("acceleration7"))
			setAccel(7, reader.getDouble("acceleration7")); 
		else
		{
			cout << "'acceleration7' default not found" << endl;

			return 0;
		}

		if(reader.keyPresent("acceleration8"))
			setAccel(8, reader.getDouble("acceleration8")); 
		else
		{
			cout << "'acceleration8' default not found" << endl;

			return 0;
		}


		//Set Decceleration
		if(reader.keyPresent("decceleration1"))
			setDecel(1, reader.getDouble("decceleration1")); 
		else
		{
			cout << "'decceleration1' default not found" << endl;

			return 0;
		}

		if(reader.keyPresent("decceleration2"))
			setDecel(2, reader.getDouble("decceleration2")); 
		else
		{
			cout << "'decceleration2' default not found" << endl;

			return 0;
		}

		if(reader.keyPresent("decceleration3"))
			setDecel(3, reader.getDouble("decceleration3")); 
		else
		{
			cout << "'decceleration3' default not found" << endl;

			return 0;
		}

		if(reader.keyPresent("decceleration4"))
			setDecel(4, reader.getDouble("decceleration4")); 
		else
		{
			cout << "'decceleration4' default not found" << endl;

			return 0;
		}

		if(reader.keyPresent("decceleration5"))
			setDecel(5, reader.getDouble("decceleration5")); 
		else		
		{
			cout << "'decceleration5' default not found" << endl;

			return 0;
		}

		if(reader.keyPresent("decceleration6"))
			setDecel(6, reader.getDouble("decceleration6")); 
		else
		{
			cout << "'decceleration6' default not found" << endl;

			return 0;
		}

		if(reader.keyPresent("decceleration7"))
			setDecel(7, reader.getDouble("decceleration7")); 
		else
		{
			cout << "'decceleration7' default not found" << endl;

			return 0;
		}

		if(reader.keyPresent("decceleration8"))
			setDecel(8, reader.getDouble("decceleration8")); 
		else
		{
			cout << "'decceleration8' default not found" << endl;

			return 0;
		}

		//set accelaration smoothing 
		if(reader.keyPresent("smoothing"))
			setSmoothing(reader.getDouble("smoothing"));
		else
		{
			cout << "'smoothing' default not found" << endl;

			return 0;
		}

		// Set Default Position
		if(reader.keyPresent("position1"))
			definePosition(1, reader.getDouble("position1")); 
		else
		{
			cout << "'position1' default not found" << endl;

			return 0;
		}

		if(reader.keyPresent("position2"))
			definePosition(2, reader.getDouble("position2")); 
		else
		{
			cout << "'position2' default not found" << endl;

			return 0;
		}

		if(reader.keyPresent("position3"))
			definePosition(3, reader.getDouble("position3")); 
		else
		{
			cout << "'position3' default not found" << endl;

			return 0;
		}

		if(reader.keyPresent("position4"))
			definePosition(4, reader.getDouble("position4")); 
		else
		{
			cout << "'position4' default not found" << endl;

			return 0;
		}

		if(reader.keyPresent("position5"))
			definePosition(5, reader.getDouble("position5")); 
		else
		{
			cout << "'position5' default not found" << endl;

			return 0;
		}

		if(reader.keyPresent("position6"))
			definePosition(6, reader.getDouble("position6")); 
		else
		{
			cout << "'position6' default not found" << endl;

			return 0;
		}

		if(reader.keyPresent("position7"))
			definePosition(7, reader.getDouble("position7")); 
		else
		{
			cout << "'position7' default not found" << endl;

			return 0;
		}

		if(reader.keyPresent("position8"))
			definePosition(8, reader.getDouble("position8")); 
		else
		{
			cout << "'position8' default not found" << endl;

			return 0;
		}

		//
		return 1;
	}

	double MotorController::enc2rad(int motorNum, long enc) // #debug needs to be finished, Also need to check initialized
	{
		if (isValidMotor(motorNum)){
			return enc * enc2Rad[motorNum]; 
		}
		else{
			std::cerr << "motor number outside range" << std::endl;
			throw std::out_of_range ("MotorNum out_of_range");
		}

	}

	long MotorController::rad2enc(int motorNum, double rad) // #debug needs to be finished, Also need to check initialized
	{
		if (isValidMotor(motorNum)){
			return rad * rad2Enc[motorNum]; 
		}
		else{
			std::cerr << "motor number outside range" << std::endl;
			throw std::out_of_range ("MotorNum out_of_range");
		}
	}

}
