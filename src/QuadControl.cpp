#include "Common.h"
#include "QuadControl.h"

#include "Utility/SimpleConfig.h"

#include "Utility/StringUtils.h"
#include "Trajectory.h"
#include "BaseController.h"
#include "Math/Mat3x3F.h"

#ifdef __PX4_NUTTX
#include <systemlib/param/param.h>
#endif

// added to constrain yaw angle
#include "Math/Angles.h"

const float G = 9.81f;

void QuadControl::Init()
{
	BaseController::Init();

	// variables needed for integral control
	integratedAltitudeError = 0;

#ifndef __PX4_NUTTX
	// Load params from simulator parameter system
	ParamsHandle config = SimpleConfig::GetInstance();

	// Load parameters (default to 0)
	kpPosXY = config->Get(_config + ".kpPosXY", 0);
	kpPosZ = config->Get(_config + ".kpPosZ", 0);
	KiPosZ = config->Get(_config + ".KiPosZ", 0);

	kpVelXY = config->Get(_config + ".kpVelXY", 0);
	kpVelZ = config->Get(_config + ".kpVelZ", 0);

	kpBank = config->Get(_config + ".kpBank", 0);
	kpYaw = config->Get(_config + ".kpYaw", 0);

	kpPQR = config->Get(_config + ".kpPQR", V3F());

	maxDescentRate = config->Get(_config + ".maxDescentRate", 100);
	maxAscentRate = config->Get(_config + ".maxAscentRate", 100);
	maxSpeedXY = config->Get(_config + ".maxSpeedXY", 100);
	maxAccelXY = config->Get(_config + ".maxHorizAccel", 100);

	maxTiltAngle = config->Get(_config + ".maxTiltAngle", 100);

	minMotorThrust = config->Get(_config + ".minMotorThrust", 0);
	maxMotorThrust = config->Get(_config + ".maxMotorThrust", 100);
#else
	// load params from PX4 parameter system
	//TODO
	param_get(param_find("MC_PITCH_P"), &Kp_bank);
	param_get(param_find("MC_YAW_P"), &Kp_yaw);
#endif
}

VehicleCommand QuadControl::GenerateMotorCommands(float collThrustCmd, V3F momentCmd)
{
	// Convert a desired 3-axis moment and collective thrust command to 
	//   individual motor thrust commands
	// INPUTS: 
	//   collThrustCmd: desired collective thrust [N]
	//   momentCmd: desired rotation moment about each axis [N m]
	// OUTPUT:
	//   set class member variable cmd (class variable for graphing) where
	//   cmd.desiredThrustsN[0..3]: motor commands, in [N]

	// HINTS: 
	// - you can access parts of momentCmd via e.g. momentCmd.x
	// You'll need the arm length parameter L, and the drag/thrust ratio kappa

	////////////////////////////// BEGIN STUDENT CODE ///////////////////////////

	float f[4];

	// I solved this two ways:
	// 1. define USE_OMEGA to get the solution that uses propeller velocities from lesson
	// 2. undef  USE_OMEGA to get the solution that solves the linear equation of forces and torques
#define USE_OMEGA

#if defined(USE_OMEGA)
	/* USE PROP VELOCITIES FOR MOTOR COMMANDS
	references used for this function:
	   Feed-Forward Parameter Identification for Precise Periodic Quadrocopter Motions
	   code from exercises
	   hints from previous cohorts (slack)
	*/
	float omega[4];
	float len = L / sqrtf(2.0f);
	float k_f = 1.0f;
	float k_m = 1.0f;

	// the problem i had was that c_bar was just collThrustCOmmand, not -c * mass
	float c_bar = collThrustCmd;
	float p_bar = momentCmd.x / len;
	float q_bar = momentCmd.y / len;
	float r_bar = -momentCmd.z / kappa;

	/* solving the linear equations for omega[n]
	isolate omega[0] by adding all four equations
	c_bar = O[0] + O[1] + O[2] + O[3]
	p_bar = O[0] - O[1] - O[2] + O[3]
	q_bar = O[0] + O[1] - O[2] - O[3]
	r_bar = O[0] - O[1] + O[2] - O[3]
	-----------------------------------
					4*O[0] + 0*O[1] + 0*O[2] + 0*O[3]
	c_bar + p_bar + q_bar + r_bar = 4 * omega[0]
	c_bar = omega[0] / 4

	isolate omega[1] by adding c_bar and q_bar
	c_bar = O[0] + O[1] + O[2] + O[3]
	q_bar = O[0] + O[1] - O[2] - O[3]
	-----------------------------------
	c_bar + qbar = 2*O[0] + 2*O[1]
	omega[1] = (c_bar + q_bar - 2*omega[0]) / 2

	isolate omega[2] by adding c_bar and r_bar
	c_bar = O[0] + O[1] + O[2] + O[3]
	r_bar = O[0] - O[1] + O[2] - O[3]
	-----------------------------------
	c_bar + r_bar = 2 * O[0] + 0 + 2 * O[2] + 0
	omega[2] = (c_bar + r_bar - 2*omega[0]) / 2

	isolate omega[3] by adding c_bar and p_bar
	c_bar = O[0] + O[1] + O[2] + O[3]
	p_bar = O[0] - O[1] - O[2] + O[3]
	-----------------------------------
	c_bar + p_bar = 2 * O[0] + 0 + 2 * O[3] + 0
	omega[3] = (c_bar + p_bar + 2*omega[0]) / 2
	*/

	omega[0] = (c_bar + p_bar + q_bar + r_bar) / 4.0f;
	omega[1] = (c_bar + q_bar - 2 * omega[0]) / 2.0f;
	omega[2] = (c_bar + r_bar - 2 * omega[0]) / 2.0f;
	omega[3] = (c_bar + p_bar - 2 * omega[0]) / 2.0f;

	f[0] = k_f * omega[0];
	f[1] = k_f * omega[1];
	f[2] = k_f * omega[2];
	f[3] = k_f * omega[3];
#else
	/* SOLVE THE INDIVIDUAL FORCE EQUATIONS
	references used for this function:
		Thrust Mixing, Saturation, and Body-Rate Control for Accurate Aggressive Quadrotor Flight
		Faessler,Scaramuzza
		hints from previous cohorts (slack)
	*/
	float t[4];
	float len = L / sqrtf(2.0f);

	t[0] = momentCmd.x / len;      // tau_x
	t[1] = momentCmd.y / len;      // tau_y
	t[2] = -momentCmd.z / kappa;   // tau_z
	t[3] = collThrustCmd;          // Ft

	/*
	t[0] = f[0] - f[1] - f[2] + f[3]
	t[1] = f[0] + f[1] - f[2] - f[3]
	t[2] = f[0] - f[1] + f[2] - f[3]
	t[3] = f[0] + f[1] + f[2] + f[3] = collective thrust
	---------------------------------------------------
	isolate f0 by adding all 4 equations
	t0 + t1 + t2 + t3 = 4f[0]
	f0 = (t0 + t1 + t2 + t3) / 4

	isolate f1 by by adding t1 and t3 and subtracting t0 and t2
	-t0 + t1 - t2 + t3 =
	-(f[0] - f[1] - f[2] + f[3])
	+(f[0] + f[1] - f[2] - f[3])
	-(f[0] - f[1] + f[2] - f[3])
	+(f[0] + f[1] + f[2] + f[3])
	 0F0     4f1    0f2    0f3
	 f1 = (-t0 + t1 - t2 + t3) / 4

	 isolate f2 by subtracting t0 and t1 and adding t2 and t3
	 -t0 - t1 + t2 + t3
	-(f[0] - f[1] - f[2] + f[3])
	-(f[0] + f[1] - f[2] - f[3])
	+(f[0] - f[1] + f[2] - f[3])
	+(f[0] + f[1] + f[2] + f[3])
	 0f0   + 0f1  + 4f2  + 0f3
	 f2 = (-t0 - t1 + t2 + t3)

	 isolate f3 by subtracting t2 and t1, adding t2 and t3
	+(f[0] - f[1] - f[2] + f[3])
	-(f[0] + f[1] - f[2] - f[3])
	-(f[0] - f[1] + f[2] - f[3])
	+(f[0] + f[1] + f[2] + f[3])
	  -f0  + 0f1  + 0f2  + 4f3
	 f3 = (t0 - t1 - t2 + t3) / 4
	*/

	f[0] = (+t[0] + t[1] + t[2] + t[3]) / 4.0f;
	f[1] = (-t[0] + t[1] - t[2] + t[3]) / 4.0f;
	f[2] = (-t[0] - t[1] + t[2] + t[3]) / 4.0f;
	f[3] = (+t[0] - t[1] - t[2] + t[3]) / 4.0f;
#endif

	// be sure to get the outputs in the right order for the motor positions
	cmd.desiredThrustsN[0] = CONSTRAIN(f[0], minMotorThrust, maxMotorThrust);
	cmd.desiredThrustsN[1] = CONSTRAIN(f[1], minMotorThrust, maxMotorThrust);
	cmd.desiredThrustsN[2] = CONSTRAIN(f[3], minMotorThrust, maxMotorThrust);
	cmd.desiredThrustsN[3] = CONSTRAIN(f[2], minMotorThrust, maxMotorThrust);

	/////////////////////////////// END STUDENT CODE ////////////////////////////

	return cmd;
}

V3F QuadControl::BodyRateControl(V3F pqrCmd, V3F pqr)
{
	// Calculate a desired 3-axis moment given a desired and current body rate
	// INPUTS: 
	//   pqrCmd: desired body rates [rad/s]
	//   pqr: current or estimated body rates [rad/s]
	// OUTPUT:
	//   return a V3F containing the desired moments for each of the 3 axes

	// HINTS: 
	//  - you can use V3Fs just like scalars: V3F a(1,1,1), b(2,3,4), c; c=a-b;
	//  - you'll need parameters for moments of inertia Ixx, Iyy, Izz
	//  - you'll also need the gain parameter kpPQR (it's a V3F)

	V3F momentCmd;

	////////////////////////////// BEGIN STUDENT CODE ///////////////////////////
	// references:
	//    lesson on pqr controller
	V3F I(Ixx, Iyy, Izz);

	// compute moments from rates * moment of intertia for each axis
	momentCmd = kpPQR * (pqrCmd - pqr) * I;
	/////////////////////////////// END STUDENT CODE ////////////////////////////

	return momentCmd;
}

// returns a desired roll and pitch rate 
V3F QuadControl::RollPitchControl(V3F accelCmd, Quaternion<float> attitude, float collThrustCmd)
{
	// Calculate a desired pitch and roll angle rates based on a desired global
	//   lateral acceleration, the current attitude of the quad, and desired
	//   collective thrust command
	// INPUTS: 
	//   accelCmd: desired acceleration in global XY coordinates [m/s2]
	//   attitude: current or estimated attitude of the vehicle
	//   collThrustCmd: desired collective thrust of the quad [N]
	// OUTPUT:
	//   return a V3F containing the desired pitch and roll rates. The Z
	//     element of the V3F should be left at its default value (0)

	// HINTS: 
	//  - we already provide rotation matrix R: to get element R[1,2] (python) use R(1,2) (C++)
	//  - you'll need the roll/pitch gain kpBank
	//  - collThrustCmd is a force in Newtons! You'll likely want to convert it to acceleration first

	V3F pqrCmd;
	Mat3x3F R = attitude.RotationMatrix_IwrtB();

	////////////////////////////// BEGIN STUDENT CODE ///////////////////////////
	float c;

	// no negative thrust allowed
	if (collThrustCmd < 0.0f) {
		// constructor zeroes these out
		return pqrCmd;
	}

	// convert collective thrust to acceleration
	c = -collThrustCmd / mass;

	float b_x_c_target;
	float b_x_c_actual;
	float b_x_c_dot;

	float b_y_c_target;
	float b_y_c_actual;
	float b_y_c_dot;

	// from Feed-Forward Parameter Identification for Precise Periodic Quadrocopter Motions
	// section III and IV
	b_x_c_target = accelCmd.x / c;
	b_x_c_actual = R(0, 2);
	b_x_c_target = CONSTRAIN(b_x_c_target, -maxTiltAngle, maxTiltAngle);
	b_x_c_dot = kpBank * (b_x_c_target - b_x_c_actual);


	b_y_c_target = accelCmd.y / c;
	b_y_c_actual = R(1, 2);
	b_y_c_target = CONSTRAIN(b_y_c_target, -maxTiltAngle, maxTiltAngle);
	b_y_c_dot = kpBank * (b_y_c_target - b_y_c_actual);

	pqrCmd.x = (b_x_c_dot * R(1, 0) - b_y_c_dot * R(0, 0)) / R(2, 2);
	pqrCmd.y = (b_x_c_dot * R(1, 1) - b_y_c_dot * R(0, 1)) / R(2, 2);
	pqrCmd.z = 0;

	/////////////////////////////// END STUDENT CODE ////////////////////////////

	return pqrCmd;
}

float QuadControl::AltitudeControl(float posZCmd, float velZCmd, float posZ, float velZ, Quaternion<float> attitude, float accelZCmd, float dt)
{
	// Calculate desired quad thrust based on altitude setpoint, actual altitude,
	//   vertical velocity setpoint, actual vertical velocity, and a vertical 
	//   acceleration feed-forward command
	// INPUTS: 
	//   posZCmd, velZCmd: desired vertical position and velocity in NED [m]
	//   posZ, velZ: current vertical position and velocity in NED [m]
	//   accelZCmd: feed-forward vertical acceleration in NED [m/s2]
	//   dt: the time step of the measurements [seconds]
	// OUTPUT:
	//   return a collective thrust command in [N]

	// HINTS: 
	//  - we already provide rotation matrix R: to get element R[1,2] (python) use R(1,2) (C++)
	//  - you'll need the gain parameters kpPosZ and kpVelZ
	//  - maxAscentRate and maxDescentRate are maximum vertical speeds. Note they're both >=0!
	//  - make sure to return a force, not an acceleration
	//  - remember that for an upright quad in NED, thrust should be HIGHER if the desired Z acceleration is LOWER

	Mat3x3F R = attitude.RotationMatrix_IwrtB();
	float thrust;

	////////////////////////////// BEGIN STUDENT CODE ///////////////////////////
	float z_err;
	float z_err_dot;
	float z_dot_limited;
	float u_1_bar;
	float c;

	/*
	   references:
		  PID Control Exercise
	*/

	// compute the desired z_dot_limited
	z_err = posZCmd - posZ;
	z_dot_limited = (z_err * kpPosZ) + velZCmd;
	z_dot_limited = CONSTRAIN(z_dot_limited, -maxAscentRate, maxDescentRate);

	// position term
	z_err_dot = velZCmd - velZ;

	// integrator
	integratedAltitudeError += z_err * dt;

	// u_1_bar term
	u_1_bar = (kpPosZ * z_err) + (kpVelZ * z_err_dot) + (KiPosZ * integratedAltitudeError) + accelZCmd;

	// collective acceleration
	c = (u_1_bar - G) / R(2, 2);

	// convert to force (F=MA) with UP = +
	thrust = -1.0f * (mass * c);

	/////////////////////////////// END STUDENT CODE ////////////////////////////
	return thrust;
}

// returns a desired acceleration in global frame
V3F QuadControl::LateralPositionControl(V3F posCmd, V3F velCmd, V3F pos, V3F vel, V3F accelCmdFF)
{
	// Calculate a desired horizontal acceleration based on 
	//  desired lateral position/velocity/acceleration and current pose
	// INPUTS: 
	//   posCmd: desired position, in NED [m]
	//   velCmd: desired velocity, in NED [m/s]
	//   pos: current position, NED [m]
	//   vel: current velocity, NED [m/s]
	//   accelCmdFF: feed-forward acceleration, NED [m/s2]
	// OUTPUT:
	//   return a V3F with desired horizontal accelerations. 
	//     the Z component should be 0
	// HINTS: 
	//  - use the gain parameters kpPosXY and kpVelXY
	//  - make sure you limit the maximum horizontal velocity and acceleration
	//    to maxSpeedXY and maxAccelXY

	// make sure we don't have any incoming z-component
	accelCmdFF.z = 0;
	velCmd.z = 0;
	posCmd.z = pos.z;

	// we initialize the returned desired acceleration to the feed-forward value.
	// Make sure to _add_, not simply replace, the result of your controller
	// to this variable
	V3F accelCmd = accelCmdFF;

	////////////////////////////// BEGIN STUDENT CODE ///////////////////////////
	/*
	   references:
		  3d control lesson
		  hints on how to limit velocity by magxy from slack channel p3-control
	*/

	float x_err;
	float x_err_dot;
	float y_err;
	float y_err_dot;
	float norm;

	// limit xy velocity command
	norm = velCmd.magXY();
	if (norm > maxSpeedXY) {
		velCmd *= maxSpeedXY / norm;
	}


	// compute x term
	x_err = posCmd.x - pos.x;
	x_err_dot = velCmd.x - vel.x;
	accelCmd.x = (kpPosXY * x_err) + (kpVelXY * x_err_dot) + accelCmdFF.x;

	// compute y term
	y_err = posCmd.y - pos.y;
	y_err_dot = velCmd.y - vel.y;
	accelCmd.y = (kpPosXY * y_err) + (kpVelXY * y_err_dot) + accelCmdFF.y;

	// limiting the accelaration command made the trajectory following worse
	// so it is removed
	// limit xy velocity command
	// norm = accelCmd.magXY();
	// if (norm > maxAccelXY) {
	// 	accelCmd *= maxAccelXY / norm;
	// }

	// no z component
	accelCmd.z = 0.0f;

	/////////////////////////////// END STUDENT CODE ////////////////////////////

	return accelCmd;
}

// returns desired yaw rate
float QuadControl::YawControl(float yawCmd, float yaw)
{
	// Calculate a desired yaw rate to control yaw to yawCmd
	// INPUTS: 
	//   yawCmd: commanded yaw [rad]
	//   yaw: current yaw [rad]
	// OUTPUT:
	//   return a desired yaw rate [rad/s]
	// HINTS: 
	//  - use fmodf(foo,b) to unwrap a radian angle measure float foo to range [0,b]. 
	//  - use the yaw control gain parameter kpYaw

	float yawRateCmd = 0;



	////////////////////////////// BEGIN STUDENT CODE ///////////////////////////
	// THIS IS MY FIX FOR THE ESTIMATION LESSON
	// It properly limits the yaw error to += PI
	float yaw_error;
	yaw_error = AngleNormF(yawCmd - yaw);
	yawRateCmd = kpYaw * yaw_error;

	// THIS IS The VERSION FROM THE CONTROL LESSON
	// THIS VERSION DID NOT WORK WITH THE ESTIMATION. It passed the tests
	// in the Control lesson but it was obvious when I ran the  Estimation
	// Scenario_11 that there was a yaw rate problem. During debug
	// I saw the that yawRateCmd was exceeding +=PI. 
	// I was not constraining the yaw error properly
	// limit yaw to 2PI
	//yawCmd = fmod(yawCmd, 2.0f * F_PI);
	//yawCmd = AngleNormF(yawCmd);
	//yaw = AngleNormF(yaw);
	//yawRateCmd = kpYaw * (yawCmd - yaw);
	/////////////////////////////// END STUDENT CODE ////////////////////////////

	//////////////////////////////// END SOLUTION ///////////////////////////////
	return yawRateCmd;

}

VehicleCommand QuadControl::RunControl(float dt, float simTime)
{
	curTrajPoint = GetNextTrajectoryPoint(simTime);

	float collThrustCmd = AltitudeControl(curTrajPoint.position.z, curTrajPoint.velocity.z, estPos.z, estVel.z, estAtt, curTrajPoint.accel.z, dt);

	// reserve some thrust margin for angle control
	float thrustMargin = .1f * (maxMotorThrust - minMotorThrust);
	collThrustCmd = CONSTRAIN(collThrustCmd, (minMotorThrust + thrustMargin) * 4.f, (maxMotorThrust - thrustMargin) * 4.f);

	V3F desAcc = LateralPositionControl(curTrajPoint.position, curTrajPoint.velocity, estPos, estVel, curTrajPoint.accel);

	V3F desOmega = RollPitchControl(desAcc, estAtt, collThrustCmd);
	desOmega.z = YawControl(curTrajPoint.attitude.Yaw(), estAtt.Yaw());

	V3F desMoment = BodyRateControl(desOmega, estOmega);

	return GenerateMotorCommands(collThrustCmd, desMoment);
}
