#include "Common.h"
#include "QuadEstimatorEKF.h"
#include "Utility/SimpleConfig.h"
#include "Utility/StringUtils.h"
#include "Math/Quaternion.h"
#include <cassert>
using namespace SLR;

// EKF state vector indices
const int EKF_STATE_X       = 0;
const int EKF_STATE_Y       = 1;
const int EKF_STATE_Z       = 2;
const int EKF_STATE_X_DOT   = 3;
const int EKF_STATE_Y_DOT   = 4;
const int EKF_STATE_Z_DOT   = 5;
const int EKF_STATE_YAW     = 6;


const int QuadEstimatorEKF::QUAD_EKF_NUM_STATES;

QuadEstimatorEKF::QuadEstimatorEKF(string config, string name)
	: BaseQuadEstimator(config),
	Q(QUAD_EKF_NUM_STATES, QUAD_EKF_NUM_STATES),
	R_GPS(6, 6),
	R_Mag(1, 1),
	ekfState(QUAD_EKF_NUM_STATES),
	ekfCov(QUAD_EKF_NUM_STATES, QUAD_EKF_NUM_STATES),
	trueError(QUAD_EKF_NUM_STATES)
{
	_name = name;
	Init();
}

QuadEstimatorEKF::~QuadEstimatorEKF()
{

}

void QuadEstimatorEKF::Init()
{
	ParamsHandle paramSys = SimpleConfig::GetInstance();

	paramSys->GetFloatVector(_config + ".InitState", ekfState);

	VectorXf initStdDevs(QUAD_EKF_NUM_STATES);
	paramSys->GetFloatVector(_config + ".InitStdDevs", initStdDevs);
	ekfCov.setIdentity();
	for (int i = 0; i < QUAD_EKF_NUM_STATES; i++)
	{
		ekfCov(i, i) = initStdDevs(i) * initStdDevs(i);
	}

	// complementary filter params
	attitudeTau = paramSys->Get(_config + ".AttitudeTau", .1f);
	dtIMU = paramSys->Get(_config + ".dtIMU", .002f);

	pitchEst = 0;
	rollEst = 0;

	// GPS measurement model covariance
	R_GPS.setZero();
	R_GPS(0, 0) = R_GPS(1, 1) = powf(paramSys->Get(_config + ".GPSPosXYStd", 0), 2);
	R_GPS(2, 2) = powf(paramSys->Get(_config + ".GPSPosZStd", 0), 2);
	R_GPS(3, 3) = R_GPS(4, 4) = powf(paramSys->Get(_config + ".GPSVelXYStd", 0), 2);
	R_GPS(5, 5) = powf(paramSys->Get(_config + ".GPSVelZStd", 0), 2);

	// magnetometer measurement model covariance
	R_Mag.setZero();
	R_Mag(0, 0) = powf(paramSys->Get(_config + ".MagYawStd", 0), 2);

	// load the transition model covariance
	Q.setZero();
	Q(0, 0) = Q(1, 1) = powf(paramSys->Get(_config + ".QPosXYStd", 0), 2);
	Q(2, 2) = powf(paramSys->Get(_config + ".QPosZStd", 0), 2);
	Q(3, 3) = Q(4, 4) = powf(paramSys->Get(_config + ".QVelXYStd", 0), 2);
	Q(5, 5) = powf(paramSys->Get(_config + ".QVelZStd", 0), 2);
	Q(6, 6) = powf(paramSys->Get(_config + ".QYawStd", 0), 2);
	Q *= dtIMU;

	rollErr = pitchErr = maxEuler = 0;
	posErrorMag = velErrorMag = 0;
}

void QuadEstimatorEKF::UpdateFromIMU(V3F accel, V3F gyro)
{
	// Improve a complementary filter-type attitude filter
	// 
	// Currently a small-angle approximation integration method is implemented
	// The integrated (predicted) value is then updated in a complementary filter style with attitude information from accelerometers
	// 
	// Implement a better integration method that uses the current attitude estimate (rollEst, pitchEst and ekfState(6))
	// to integrate the body rates into new Euler angles.
	//
	// HINTS:
	//  - there are several ways to go about this, including:
	//    1) create a rotation matrix based on your current Euler angles, integrate that, convert back to Euler angles
	//    OR 
	//    2) use the Quaternion<float> class, which has a handy FromEuler123_RPY function for creating a quaternion from Euler Roll/PitchYaw
	//       (Quaternion<float> also has a IntegrateBodyRate function, though this uses quaternions, not Euler angles)

	////////////////////////////// BEGIN STUDENT CODE ///////////////////////////
	// SMALL ANGLE GYRO INTEGRATION:


	// forwarding variables to match existing code
	float predictedPitch;
	float predictedRoll;
	float yaw = ekfState(6);

	// create quaternion from current Euler angle estimates
	Quaternion<float> q_t = Quaternion<float>::FromEuler123_RPY(rollEst, pitchEst, yaw);

	// integrate with the body rates
	Quaternion<float> q_bar = q_t.IntegrateBodyRate(gyro, dtIMU);

	// convert back to euler
	V3D rpy = q_bar.ToEulerRPY();
	
	// forward the values
	predictedRoll  = static_cast<float>(rpy[0]);
	predictedPitch = static_cast<float>(rpy[1]);
	ekfState(6)    = static_cast<float>(rpy[2]);

	//// normalize yaw to -pi .. pi
	if (ekfState(6) >  F_PI) ekfState(6) -= 2.f*F_PI;
	if (ekfState(6) < -F_PI) ekfState(6) += 2.f*F_PI;

	/////////////////////////////// END STUDENT CODE ////////////////////////////

	// CALCULATE UPDATE
	accelRoll = atan2f(accel.y, accel.z);
	accelPitch = atan2f(-accel.x, 9.81f);

	// FUSE INTEGRATION AND UPDATE
	rollEst = attitudeTau / (attitudeTau + dtIMU) * (predictedRoll)+dtIMU / (attitudeTau + dtIMU) * accelRoll;
	pitchEst = attitudeTau / (attitudeTau + dtIMU) * (predictedPitch)+dtIMU / (attitudeTau + dtIMU) * accelPitch;

	lastGyro = gyro;
}

void QuadEstimatorEKF::UpdateTrueError(V3F truePos, V3F trueVel, Quaternion<float> trueAtt)
{
	VectorXf trueState(QUAD_EKF_NUM_STATES);
	trueState(0) = truePos.x;
	trueState(1) = truePos.y;
	trueState(2) = truePos.z;
	trueState(3) = trueVel.x;
	trueState(4) = trueVel.y;
	trueState(5) = trueVel.z;
	trueState(6) = trueAtt.Yaw();

	trueError = ekfState - trueState;
	if (trueError(6) > F_PI) trueError(6) -= 2.f * F_PI;
	if (trueError(6) < -F_PI) trueError(6) += 2.f * F_PI;

	pitchErr = pitchEst - trueAtt.Pitch();
	rollErr = rollEst - trueAtt.Roll();
	maxEuler = MAX(fabs(pitchErr), MAX(fabs(rollErr), fabs(trueError(6))));

	posErrorMag = truePos.dist(V3F(ekfState(0), ekfState(1), ekfState(2)));
	velErrorMag = trueVel.dist(V3F(ekfState(3), ekfState(4), ekfState(5)));
}

VectorXf QuadEstimatorEKF::PredictState(VectorXf curState, float dt, V3F accel, V3F gyro)
{
	assert(curState.size() == QUAD_EKF_NUM_STATES);
	VectorXf predictedState = curState;
	// Predict the current state forward by time dt using current accelerations and body rates as input
	// INPUTS: 
	//   curState: starting state
	//   dt: time step to predict forward by [s]
	//   accel: acceleration of the vehicle, in body frame, *not including gravity* [m/s2]
	//   gyro: body rates of the vehicle, in body frame [rad/s]
	//   
	// OUTPUT:
	//   return the predicted state as a vector

	// HINTS 
	// - dt is the time duration for which you should predict. It will be very short (on the order of 1ms)
	//   so simplistic integration methods are fine here
	// - we've created an Attitude Quaternion for you from the current state. Use 
	//   attitude.Rotate_BtoI(<V3F>) to rotate a vector from body frame to inertial frame
	// - the yaw integral is already done in the IMU update. Be sure not to integrate it again here

	Quaternion<float> attitude = Quaternion<float>::FromEuler123_RPY(rollEst, pitchEst, curState(6));

	////////////////////////////// BEGIN STUDENT CODE ///////////////////////////

	V3F v = V3F(curState[3],curState[4],curState[5]);
	V3F p = V3F(curState[0],curState[1],curState[2]);
	V3F g = V3F(0.0f, 0.0f, 9.81f);
	V3F euler_dot;
	V3F linear_acc;
	
	// get euler derivatives (not used)
	euler_dot = attitude.Rotate_BtoI(gyro);

	// get linear acceleration in inertial frame
	// and remove the gravity vector
	linear_acc = attitude.Rotate_BtoI(accel) - g;

	// integrate to get velocities
	v += linear_acc * dt;

	// integrate to get position
	p += v * dt;
	
	predictedState[EKF_STATE_X]       = p[0];
	predictedState[EKF_STATE_Y]       = p[1]; 
	predictedState[EKF_STATE_Z]       = p[2]; 
	predictedState[EKF_STATE_X_DOT]   = v[0]; 
	predictedState[EKF_STATE_Y_DOT]   = v[1]; 
	predictedState[EKF_STATE_Z_DOT]   = v[2]; 
	predictedState[6] = curState[EKF_STATE_YAW]; 
	
	/////////////////////////////// END STUDENT CODE ////////////////////////////

	return predictedState;
}

MatrixXf QuadEstimatorEKF::GetRbgPrime(float roll, float pitch, float yaw)
{
	// first, figure out the Rbg_prime
	MatrixXf RbgPrime(3, 3);
	RbgPrime.setZero();

	// Return the partial derivative of the Rbg rotation matrix with respect to yaw. We call this RbgPrime.
	// INPUTS: 
	//   roll, pitch, yaw: Euler angles at which to calculate RbgPrime
	//   
	// OUTPUT:
	//   return the 3x3 matrix representing the partial derivative at the given point

	// HINTS
	// - this is just a matter of putting the right sin() and cos() functions in the right place.
	//   make sure you write clear code and triple-check your math
	// - You can also do some numerical partial derivatives in a unit test scheme to check 
	//   that your calculations are reasonable

	////////////////////////////// BEGIN STUDENT CODE ///////////////////////////
	float c_phi = cosf(roll);
	float s_phi = sinf(roll);
	float c_tht = cosf(pitch);
	float s_tht = sinf(pitch);
	float c_psi = cosf(yaw);
	float s_psi = sinf(yaw);

	// from section 7.2 Transition Model of Estimation for Quadrotors
	RbgPrime(0, 0) =         -c_tht * s_psi;
	RbgPrime(0, 1) = -s_phi * s_tht * s_psi - c_phi * c_psi;
	RbgPrime(0, 2) = -c_phi * s_tht * s_psi + s_phi * c_psi;

	RbgPrime(1, 0) =         c_tht * c_psi;
	RbgPrime(1, 1) = s_phi * s_tht * c_psi - c_phi * s_psi;
	RbgPrime(1, 2) = c_phi * s_tht * c_psi + s_phi * s_psi;

	RbgPrime(2, 0) = 0;
	RbgPrime(2, 1) = 0;
	RbgPrime(2, 2) = 0;

	/////////////////////////////// END STUDENT CODE ////////////////////////////

	return RbgPrime;
}

void QuadEstimatorEKF::Predict(float dt, V3F accel, V3F gyro)
{
	// predict the state forward
	VectorXf newState = PredictState(ekfState, dt, accel, gyro);

	// Predict the current covariance forward by dt using the current accelerations and body rates as input.
	// INPUTS: 
	//   dt: time step to predict forward by [s]
	//   accel: acceleration of the vehicle, in body frame, *not including gravity* [m/s2]
	//   gyro: body rates of the vehicle, in body frame [rad/s]
	//   state (member variable): current state (state at the beginning of this prediction)
	//   
	// OUTPUT:
	//   update the member variable cov to the predicted covariance

	// HINTS
	// - update the covariance matrix cov according to the EKF equation.
	// 
	// - you may find the current estimated attitude in variables rollEst, pitchEst, state(6).
	//
	// - use the class MatrixXf for matrices. To create a 3x5 matrix A, use MatrixXf A(3,5).
	//
	// - the transition model covariance, Q, is loaded up from a parameter file in member variable Q
	// 
	// - This is unfortunately a messy step. Try to split this up into clear, manageable steps:
	//   1) Calculate the necessary helper matrices, building up the transition jacobian
	//   2) Once all the matrices are there, write the equation to update cov.
	//
	// - if you want to transpose a matrix in-place, use A.transposeInPlace(), not A = A.transpose()
	// 

	// we'll want the partial derivative of the Rbg matrix
	MatrixXf RbgPrime = GetRbgPrime(rollEst, pitchEst, ekfState(6));

	// we've created an empty Jacobian for you, currently simply set to identity
	MatrixXf gPrime(QUAD_EKF_NUM_STATES, QUAD_EKF_NUM_STATES);
	gPrime.setIdentity();

	////////////////////////////// BEGIN STUDENT CODE ///////////////////////////
	// control inputs are the measured accelerations in body frame
	// ref: Lesson 4.5 : EKF Tradeoffs 2 - Control
	// ref: Estimation_For_Quadrotors
	// use VectorXf so Eigen matrix muliply works
	assert(RbgPrime.size() == 9);
	assert(gPrime.size() == 49);

	
	// create matrix from Rbg' and inputs for update to g'

	// u is control inputs : body accelerations
	Eigen::Vector3f u(3);
	u[0] = accel.x;
	u[1] = accel.y;
	u[2] = accel.z - 9.81f;

	// compute derivatives RgbPrime * (u * dt)
	Eigen::MatrixXf r;
	r = RbgPrime * (u * dt);
	assert(r.size() == 3);
	// initialize gPrime Jacobian

	// these cells are delta T
	gPrime(0, 3) = dt;
	gPrime(1, 4) = dt;
	gPrime(2, 5) = dt;

	// these cells are the derivaives from 'r'
	gPrime(3, 6) = r(0);
	gPrime(4, 6) = r(1);
	gPrime(5, 6) = r(2);

	// compute new covariance 
	ekfCov = (gPrime * (ekfCov * gPrime.transpose())) + Q;
	assert(ekfCov.size() == 49);

	/////////////////////////////// END STUDENT CODE ////////////////////////////

	ekfState = newState;
}

void QuadEstimatorEKF::UpdateFromGPS(V3F pos, V3F vel)
{
	VectorXf z(6), zFromX(6);
	z(0) = pos.x;
	z(1) = pos.y;
	z(2) = pos.z;
	z(3) = vel.x;
	z(4) = vel.y;
	z(5) = vel.z;

	MatrixXf hPrime(6, QUAD_EKF_NUM_STATES);
	hPrime.setZero();

	// GPS UPDATE
	// Hints: 
	//  - The GPS measurement covariance is available in member variable R_GPS
	//  - this is a very simple update
	////////////////////////////// BEGIN STUDENT CODE ///////////////////////////
	// set zFromX to first 6 elements of ekfState
	for (auto i = 0; i < zFromX.size(); ++i) {
		zFromX(i) = ekfState(i);
	}

	// set diagonal from 0 to rows to 1.0f
	// hPrime has 6 rows, 7 columns. leave seventh column == 0
	for (auto i = 0; i < hPrime.rows(); ++i) {
		hPrime(i,i) = 1.0f;
	}

	/////////////////////////////// END STUDENT CODE ////////////////////////////

	Update(z, hPrime, R_GPS, zFromX);
}

void QuadEstimatorEKF::UpdateFromMag(float magYaw)
{
	VectorXf z(1), zFromX(1);
	z(0) = magYaw;

	MatrixXf hPrime(1, QUAD_EKF_NUM_STATES);
	hPrime.setZero();

	// MAGNETOMETER UPDATE
	// Hints: 
	//  - Your current estimated yaw can be found in the state vector: ekfState(6)
	//  - Make sure to normalize the difference between your measured and estimated yaw
	//    (you don't want to update your yaw the long way around the circle)
	//  - The magnetomer measurement covariance is available in member variable R_Mag
	////////////////////////////// BEGIN STUDENT CODE ///////////////////////////
	// update jacobian
	hPrime(6) = 1;

	// current value
	zFromX(0) = ekfState(6);

	// difference
	float dz = z(0) - zFromX(0);

	// normalize z(0)
	if (dz > F_PI)  z(0) -= 2.f * F_PI;
	if (dz < -F_PI) z(0) += 2.f * F_PI;

	/////////////////////////////// END STUDENT CODE ////////////////////////////

	Update(z, hPrime, R_Mag, zFromX);
}

// Execute an EKF update step
// z: measurement
// H: Jacobian of observation function evaluated at the current estimated state
// R: observation error model covariance 
// zFromX: measurement prediction based on current state
void QuadEstimatorEKF::Update(VectorXf & z, MatrixXf & H, MatrixXf & R, VectorXf & zFromX)
{
	assert(z.size() == H.rows());
	assert(QUAD_EKF_NUM_STATES == H.cols());
	assert(z.size() == R.rows());
	assert(z.size() == R.cols());
	assert(z.size() == zFromX.size());

	MatrixXf toInvert(z.size(), z.size());
	toInvert = H * ekfCov * H.transpose() + R;
	MatrixXf K = ekfCov * H.transpose() * toInvert.inverse();

	ekfState = ekfState + K * (z - zFromX);

	MatrixXf eye(QUAD_EKF_NUM_STATES, QUAD_EKF_NUM_STATES);
	eye.setIdentity();

	ekfCov = (eye - K * H) * ekfCov;
}

// Calculate the condition number of the EKF ovariance matrix (useful for numerical diagnostics)
// The condition number provides a measure of how similar the magnitudes of the error metric beliefs 
// about the different states are. If the magnitudes are very far apart, numerical issues will start to come up.
float QuadEstimatorEKF::CovConditionNumber() const
{
	MatrixXf m(7, 7);
	for (int i = 0; i < 7; i++)
	{
		for (int j = 0; j < 7; j++)
		{
			m(i, j) = ekfCov(i, j);
		}
	}

	Eigen::JacobiSVD<MatrixXf> svd(m);
	float cond = svd.singularValues()(0)
		/ svd.singularValues()(svd.singularValues().size() - 1);
	return cond;
}

// Access functions for graphing variables
bool QuadEstimatorEKF::GetData(const string & name, float& ret) const
{
	if (name.find_first_of(".") == string::npos) return false;
	string leftPart = LeftOf(name, '.');
	string rightPart = RightOf(name, '.');

	if (ToUpper(leftPart) == ToUpper(_name))
	{
#define GETTER_HELPER(A,B) if (SLR::ToUpper(rightPart) == SLR::ToUpper(A)){ ret=(B); return true; }
		GETTER_HELPER("Est.roll", rollEst);
		GETTER_HELPER("Est.pitch", pitchEst);

		GETTER_HELPER("Est.x", ekfState(0));
		GETTER_HELPER("Est.y", ekfState(1));
		GETTER_HELPER("Est.z", ekfState(2));
		GETTER_HELPER("Est.vx", ekfState(3));
		GETTER_HELPER("Est.vy", ekfState(4));
		GETTER_HELPER("Est.vz", ekfState(5));
		GETTER_HELPER("Est.yaw", ekfState(6));

		GETTER_HELPER("Est.S.x", sqrtf(ekfCov(0, 0)));
		GETTER_HELPER("Est.S.y", sqrtf(ekfCov(1, 1)));
		GETTER_HELPER("Est.S.z", sqrtf(ekfCov(2, 2)));
		GETTER_HELPER("Est.S.vx", sqrtf(ekfCov(3, 3)));
		GETTER_HELPER("Est.S.vy", sqrtf(ekfCov(4, 4)));
		GETTER_HELPER("Est.S.vz", sqrtf(ekfCov(5, 5)));
		GETTER_HELPER("Est.S.yaw", sqrtf(ekfCov(6, 6)));

		// diagnostic variables
		GETTER_HELPER("Est.D.AccelPitch", accelPitch);
		GETTER_HELPER("Est.D.AccelRoll", accelRoll);

		GETTER_HELPER("Est.D.ax_g", accelG[0]);
		GETTER_HELPER("Est.D.ay_g", accelG[1]);
		GETTER_HELPER("Est.D.az_g", accelG[2]);

		GETTER_HELPER("Est.E.x", trueError(0));
		GETTER_HELPER("Est.E.y", trueError(1));
		GETTER_HELPER("Est.E.z", trueError(2));
		GETTER_HELPER("Est.E.vx", trueError(3));
		GETTER_HELPER("Est.E.vy", trueError(4));
		GETTER_HELPER("Est.E.vz", trueError(5));
		GETTER_HELPER("Est.E.yaw", trueError(6));
		GETTER_HELPER("Est.E.pitch", pitchErr);
		GETTER_HELPER("Est.E.roll", rollErr);
		GETTER_HELPER("Est.E.MaxEuler", maxEuler);

		GETTER_HELPER("Est.E.pos", posErrorMag);
		GETTER_HELPER("Est.E.vel", velErrorMag);

		GETTER_HELPER("Est.D.covCond", CovConditionNumber());
#undef GETTER_HELPER
	}
	return false;
};

vector<string> QuadEstimatorEKF::GetFields() const
{
	vector<string> ret = BaseQuadEstimator::GetFields();
	ret.push_back(_name + ".Est.roll");
	ret.push_back(_name + ".Est.pitch");

	ret.push_back(_name + ".Est.x");
	ret.push_back(_name + ".Est.y");
	ret.push_back(_name + ".Est.z");
	ret.push_back(_name + ".Est.vx");
	ret.push_back(_name + ".Est.vy");
	ret.push_back(_name + ".Est.vz");
	ret.push_back(_name + ".Est.yaw");

	ret.push_back(_name + ".Est.S.x");
	ret.push_back(_name + ".Est.S.y");
	ret.push_back(_name + ".Est.S.z");
	ret.push_back(_name + ".Est.S.vx");
	ret.push_back(_name + ".Est.S.vy");
	ret.push_back(_name + ".Est.S.vz");
	ret.push_back(_name + ".Est.S.yaw");

	ret.push_back(_name + ".Est.E.x");
	ret.push_back(_name + ".Est.E.y");
	ret.push_back(_name + ".Est.E.z");
	ret.push_back(_name + ".Est.E.vx");
	ret.push_back(_name + ".Est.E.vy");
	ret.push_back(_name + ".Est.E.vz");
	ret.push_back(_name + ".Est.E.yaw");
	ret.push_back(_name + ".Est.E.pitch");
	ret.push_back(_name + ".Est.E.roll");

	ret.push_back(_name + ".Est.E.pos");
	ret.push_back(_name + ".Est.E.vel");

	ret.push_back(_name + ".Est.E.maxEuler");

	ret.push_back(_name + ".Est.D.covCond");

	// diagnostic variables
	ret.push_back(_name + ".Est.D.AccelPitch");
	ret.push_back(_name + ".Est.D.AccelRoll");
	ret.push_back(_name + ".Est.D.ax_g");
	ret.push_back(_name + ".Est.D.ay_g");
	ret.push_back(_name + ".Est.D.az_g");
	return ret;
};
