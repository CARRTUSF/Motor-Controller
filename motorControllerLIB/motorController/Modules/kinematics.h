
#ifdef KINEMATICS_EXPORTS
#define KINEMATICS_API __declspec(dllexport) 
#else
#define KINEMATICS_API __declspec(dllimport) 
#endif

/* This function gives the Transformation Matrix of the new USF WMRA with 7 DOF, given the joint angles in Radians.

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%% COPY RIGHTS RESERVED %%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%%%%%%% Developed By: Redwan M. Alqasemi %%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% April 2007 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

Function Declaration:*/

#ifndef _kinematics_H // if not defined
#define _kinematics_H // define MyHeader


#include <vector>
#include <time.h>
#include "matrix.h" 

#define EPS 2.2204460492503131e-016 

using namespace std;
using namespace math;

#ifndef _NO_TEMPLATE
typedef matrix<double> Matrix;
#else
typedef matrix Matrix;
#endif

namespace kinematicsLIB
{
	class kinematics
	{
	public:
		kinematics();
		static __declspec(dllexport) Matrix WMRA_DH(vector<double> q);
		static __declspec(dllexport) Matrix kinematic(vector<double> q);
		static __declspec(dllexport) Matrix kinematic(vector<double> q, Matrix dq, Matrix Twc, Matrix& Ta, Matrix& Twco, Matrix& T1, Matrix& T2, Matrix& T3, Matrix& T4, Matrix& T5, Matrix& T6, Matrix& T7);
		static __declspec(dllexport) Matrix kinematic(vector<double> q, Matrix& Ta, Matrix& T1, Matrix& T2, Matrix& T3, Matrix& T4, Matrix& T5, Matrix& T6, Matrix& T7);
		static __declspec(dllexport) Matrix WMRA_rotx(float t);
		static __declspec(dllexport) Matrix WMRA_rotz(float t);
		static __declspec(dllexport) Matrix WMRA_transl(float x, float y, float z);
		static __declspec(dllexport) Matrix WMRA_roty(float t);
		static __declspec(dllexport) Matrix WMRA_w2T(int ind, Matrix Tp, Matrix q);
		static __declspec(dllexport) Matrix WMRA_WCD();
		static __declspec(dllexport) Matrix WMRA_p2T(double x, double y, double a);
		static __declspec(dllexport) Matrix rotationMatrix(double pitch, double roll, double yaw);
	};
}

#endif
