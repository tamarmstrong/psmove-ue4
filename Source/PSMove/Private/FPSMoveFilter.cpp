//
//  FPSMoveFilter.cpp
//  Kalman filter used to smooth out noise in the PSMoves positional tracking
//
//  Created by Brendan Walker on 2015-06-16.
//
#include "PSMovePrivatePCH.h"
#include "FPSMoveCalibration.h"
#include "FPSMoveFilter.h"
#include "FPSMoveMatrix.h"
#include "FPSMove.h"
#include <math.h>
#include <assert.h>

//-- prototypes ----
static void MakeStateTransitionMatrix(const float dT, FStackMatrixMxN<6, 6> &m);
static void MakeControlModelMatrix(const float dT, FStackMatrixMxN<6,3> &m);
static void MakeProcessNoiseMatrix(const float varA, const float dT, FStackMatrixMxN<6, 6> &m);
static void MakeControlInputVector(const FVector &v, FStackMatrixMxN<3,1> &m);
static void ComputeErrorVector(const FStackMatrixMxN<6, 1> &PredictStateVector, const FVector &MeasuredPosition, FStackMatrixMxN<3, 1> &ErrorVector);
static void ComputHPHtMatrix(const FStackMatrixMxN<6, 6> &P, FStackMatrixMxN<3, 3> &HPH_t);
static void ComputPHtMatrix(const FStackMatrixMxN<6, 6> &P, FStackMatrixMxN<6, 3> &PH_t);
static void ComputIMinusKHMatrix(const FStackMatrixMxN<6, 3> &K, FStackMatrixMxN<6, 6> &IMinusKH);
static void MatrixInvert(FStackMatrixMxN<3, 3> &m);

static void MakeStateTransitionMatrix(const float dT, FStackMatrixMxN<2, 2> &m);
static void MakeControlModelMatrix(const float dT, FStackMatrixMxN<2, 1> &m);
static void MakeProcessNoiseMatrix(const float varA, const float dT, FStackMatrixMxN<2, 2> &m);
static void MakeControlInputVector(const FVector &v, FStackMatrixMxN<1, 1> &m);
static void ComputeErrorVector(const FStackMatrixMxN<2, 1> &PredictStateVector, const float MeasuredPosition, FStackMatrixMxN<1, 1> &ErrorVector);
static void ComputHPHtMatrix(const FStackMatrixMxN<2, 2> &P, FStackMatrixMxN<1, 1> &HPH_t);
static void ComputPHtMatrix(const FStackMatrixMxN<2, 2> &P, FStackMatrixMxN<2, 1> &PH_t);
static void ComputIMinusKHMatrix(const FStackMatrixMxN<2, 1> &K, FStackMatrixMxN<2, 2> &IMinusKH);
static void MatrixInvert(FStackMatrixMxN<1, 1> &m);

//-- FPSMove1DKalmanFilter ----
FPSMove1DKalmanFilter::FPSMove1DKalmanFilter() :
	StateVector(),
	StateCovarianceMatrix(),
	AccelerationVariance(0.f),
	MeasurementNoiseMatrix()
{
	Clear();
}

float FPSMove1DKalmanFilter::GetFilteredPosition() const
{
	return StateVector[0][0];
}

float FPSMove1DKalmanFilter::GetFilteredVelocity() const
{
	return StateVector[1][0];
}

float FPSMove1DKalmanFilter::GetFilteredPositionVariance() const
{
	return StateCovarianceMatrix[0][0];
}

float FPSMove1DKalmanFilter::GetFilteredVelocityVariance() const
{
	return StateCovarianceMatrix[1][1];
}

void FPSMove1DKalmanFilter::Clear()
{
	IsInitialized = false;

	AccelerationVariance = 0.f;
	MeasurementNoiseMatrix[0][0] = 0.f;
	StateCovarianceMatrix.SetZero();
	StateVector.SetZero();
}

void FPSMove1DKalmanFilter::Initialize(
	const float accelerationVariance,
	const float measurementNoiseVariance,
	const float position)
{
	const float k_large_variance = 10.f;

	// Set the calibrated noise parameters
	AccelerationVariance = accelerationVariance;
	MeasurementNoiseMatrix[0][0] = measurementNoiseVariance;

	// Initialize the state covariance matrix to have large initial uncertainty about position/velocity
	StateCovarianceMatrix.SetIdentity();
	StateCovarianceMatrix[0][0] = k_large_variance;
	StateCovarianceMatrix[1][1] = k_large_variance;

	// Initialize the state vector with the given starting location and assume zero velocity
	StateVector[0][0] = position;
	StateVector[1][0] = 0.f;

	IsInitialized = true;
}

void FPSMove1DKalmanFilter::Update(
	const float MeasuredPosition,	// The position measured by the sensors.
	const float AccelerationControl,	// The world space acceleration measured on the controller.
	const float TimeDelta)				// The time delta in seconds
{
	// 'x_(k|k-1)' - Predicted state given the previous state
	// In this case a 6x1 vector containing position and velocity
	FStackMatrixMxN<2, 1> PredictStateVector;

	// 'P_(k|k-1)' - Predicted covariance matrix of the state 'x_(k|k-1)'
	// Captures the uncertainty of the state vector for the predicted state vector
	// In this case a 6x6 matrix
	FStackMatrixMxN<2, 2> PredictStateCovarianceMatrix;

	// -- Prediction Phase --
	{
		// 'F' - State transition matrix. Model of how the physics evolves for the current state.
		// In this case a 6x6 matrix 
		FStackMatrixMxN<2, 2> StateTransitionMatrix;
		FStackMatrixMxN<2, 2> StateTransitionMatrixTranspose;

		// 'B' - Control model matrix. Model of how control inputs maps the input vector to the state.
		// In this case a 2x1 matrix 
		FStackMatrixMxN<2, 1> ControlModelMatrix;

		// 'Q' - Process noise covariance matrix.
		// The process noise sigma_p, which estimates the error in our system model. 
		// It is dependent on the model, i.e. how exactly it estimates future 
		// values from the current state of the Kalman filter.
		// In this case a 2x2 matrix: Q = B*B_transpose*acceleration_variance
		FStackMatrixMxN<2, 2> ProcessNoiseMatrix;

		// Build the matrices needed for prediction
		MakeStateTransitionMatrix(TimeDelta, StateTransitionMatrix); // 2x2 Matrix F
		MakeControlModelMatrix(TimeDelta, ControlModelMatrix); // 2x1 Matrix B
		MakeProcessNoiseMatrix(AccelerationVariance, TimeDelta, ProcessNoiseMatrix); // 2x2 Matrix Q

		// x_(k|k-1) = F*x_(k-1|k-1)
		MatrixMultiplyAccumulate(StateTransitionMatrix, StateVector, PredictStateVector);
		// + B*u
		PredictStateVector[0][0] += ControlModelMatrix[0][0] * AccelerationControl;
		PredictStateVector[1][0] += ControlModelMatrix[1][0] * AccelerationControl;

		// P_(k|k-1) = F*P_(k-1|k-1)*transpose(F) + Q
		{
			FStackMatrixMxN<2, 2> F_times_P_k_minus_1;

			MatrixTranspose(StateTransitionMatrix, StateTransitionMatrixTranspose);
			MatrixMultiply(StateTransitionMatrix, StateCovarianceMatrix, F_times_P_k_minus_1);
			MatrixMultiply(F_times_P_k_minus_1, StateTransitionMatrixTranspose, PredictStateCovarianceMatrix);
			MatrixAdd(PredictStateCovarianceMatrix, ProcessNoiseMatrix, PredictStateCovarianceMatrix);
		}
	}

	// -- Correction Phase--
	{
		// 'x_(k|k)' - Corrected state given the predicted state
		// In this case a 6x1 vector containing position and velocity
		FStackMatrixMxN<2, 1> CorrectedStateVector;

		// 'P_(k|k)' - The covariance matrix of the state 'x_(k)'
		// Captures the uncertainty of the corrected state vector
		FStackMatrixMxN<2, 2> CorrectedStateCovarianceMatrix;

		// 'y' - The error between the measurement and what we expected
		FStackMatrixMxN<1, 1> ErrorVector;

		// 'S^-1' - The inverse of covariance of the move
		FStackMatrixMxN<1, 1> MoveCovarianceInverseMatrix;

		// 'K' - The Kalman gain, which is used to compute the corrected 'x_(k)' and 'P_(k)'
		FStackMatrixMxN<2, 1> KalmanGain;

		// y = z - H*x_(k|k-1)
		// In this case 'y' is a 3x1 vector where:
		//  z = the sensor measurement vector of the system (in this case the measure position)
		//  H = the observation matrix that maps the the state vector into sensor vector space
		ComputeErrorVector(PredictStateVector, MeasuredPosition, ErrorVector);

		// S^-1 = (H * P_(k|k-1) * transpose(H) + R)^-1
		// In this case S is a 1x1 matrix where:
		//  H = the observation matrix that maps the the state vector into sensor vector space
		//  P_(k|k-1) = The predicted state covariance matrix computed in the update phase
		//  R = The covariance matrix of the measurement vector 'z'
		{
			FStackMatrixMxN<1, 1> HPH_t;

			ComputHPHtMatrix(PredictStateCovarianceMatrix, HPH_t);
			MatrixAdd(HPH_t, MeasurementNoiseMatrix, MoveCovarianceInverseMatrix);
			MatrixInvert(MoveCovarianceInverseMatrix);
		}

		// K = P_(k|k-1)*transpose(H)*S^-1
		{
			FStackMatrixMxN<2, 1> PH_t;

			ComputPHtMatrix(PredictStateCovarianceMatrix, PH_t);
			MatrixMultiply(PH_t, MoveCovarianceInverseMatrix, KalmanGain);
		}

		// x_(k|k) = x_(k|k-1) + K*y
		{
			FStackMatrixMxN<2, 1> Ky;

			MatrixMultiply(KalmanGain, ErrorVector, Ky);
			MatrixAdd(PredictStateVector, Ky, CorrectedStateVector);
		}

		// P_(k|k) = (I - K*H)*P_(k|k-1)
		{
			FStackMatrixMxN<2, 2> IMinusKH;

			ComputIMinusKHMatrix(KalmanGain, IMinusKH);
			MatrixMultiply(IMinusKH, PredictStateCovarianceMatrix, CorrectedStateCovarianceMatrix);
		}

		// x_(k-1|k-1)= x_(k|k)
		StateVector.CopyFromMatrix(CorrectedStateVector);

		// P_(k-1|k-1) = P_(k|k)
		StateCovarianceMatrix.CopyFromMatrix(CorrectedStateCovarianceMatrix);
	}
}

//-- FPSMovePositionKalmanFilter ----
FPSMovePositionKalmanFilter::FPSMovePositionKalmanFilter() :
	StateVector(),
	StateCovarianceMatrix(),
	AccelerationVariance(0.f),
	MeasurementNoiseMatrix()
{
	Clear();
}

FVector FPSMovePositionKalmanFilter::GetFilteredPosition() const
{
	return FVector(StateVector[0][0], StateVector[1][0], StateVector[2][0]);
}

FVector FPSMovePositionKalmanFilter::GetFilteredVelocity() const
{
	return FVector(StateVector[3][0], StateVector[4][0], StateVector[5][0]);
}

FVector FPSMovePositionKalmanFilter::GetFilteredPositionVariance() const
{
	return FVector(StateCovarianceMatrix[0][0], StateCovarianceMatrix[1][1], StateCovarianceMatrix[2][2]);
}

FVector FPSMovePositionKalmanFilter::GetFilteredVelocityVariance() const
{
	return FVector(StateCovarianceMatrix[3][3], StateCovarianceMatrix[4][4], StateCovarianceMatrix[5][5]);
}

void FPSMovePositionKalmanFilter::Clear()
{
	IsInitialized = false;

	AccelerationVariance = 0.f;
	MeasurementNoiseMatrix.SetZero();
	StateCovarianceMatrix.SetZero();
	StateVector.SetZero();
}

void FPSMovePositionKalmanFilter::Initialize(
	float accelerationVariance,
	const FStackMatrixMxN<3, 3> &measurementCovarianceMatrix,
	const FVector &position)
{
	const float k_large_variance = 10.f;

	// Set the calibrated noise parameters
	AccelerationVariance = accelerationVariance;
	MeasurementNoiseMatrix.CopyFromMatrix(measurementCovarianceMatrix);

	// Initialize the state covariance matrix to have large initial uncertainty about position/velocity
	StateCovarianceMatrix.SetIdentity();
	StateCovarianceMatrix[0][0] = k_large_variance;
	StateCovarianceMatrix[1][1] = k_large_variance;
	StateCovarianceMatrix[2][2] = k_large_variance;
	StateCovarianceMatrix[3][3] = k_large_variance;
	StateCovarianceMatrix[4][4] = k_large_variance;
	StateCovarianceMatrix[5][5] = k_large_variance;

	// Initialize the state vector with the given starting location and assume zero velocity
	StateVector[0][0] = position.X;
	StateVector[1][0] = position.Y;
	StateVector[2][0] = position.Z;
	StateVector[3][0] = 0.f;
	StateVector[4][0] = 0.f;
	StateVector[5][0] = 0.f;

	IsInitialized = true;
}

void FPSMovePositionKalmanFilter::Update(
	const FVector &MeasuredPosition,	// The position measured by the sensors.
	const FVector &AccelerationControl,	// The world space acceleration measured on the controller.
	const float TimeDelta)				// The time delta in seconds
{
	// 'x_(k|k-1)' - Predicted state given the previous state
	// In this case a 6x1 vector containing position and velocity
	FStackMatrixMxN<6,1> PredictStateVector;

	// 'P_(k|k-1)' - Predicted covariance matrix of the state 'x_(k|k-1)'
	// Captures the uncertainty of the state vector for the predicted state vector
	// In this case a 6x6 matrix
	FStackMatrixMxN<6,6> PredictStateCovarianceMatrix;

	// -- Prediction Phase --
	{
		// 'F' - State transition matrix. Model of how the physics evolves for the current state.
		// In this case a 6x6 matrix 
		FStackMatrixMxN<6,6> StateTransitionMatrix;
		FStackMatrixMxN<6,6> StateTransitionMatrixTranspose;

		// 'B' - Control model matrix. Model of how control inputs maps the input vector to the state.
		// In this case a 6x3 matrix 
		FStackMatrixMxN<6,3> ControlModelMatrix;

		// 'u' - Control input vector. The acceleration control applied to the system.
		// In this case a 3x1 matrix
		FStackMatrixMxN<3,1> ControlInputVector;

		// 'Q' - Process noise covariance matrix.
		// The process noise sigma_p, which estimates the error in our system model. 
		// It is dependent on the model, i.e. how exactly it estimates future 
		// values from the current state of the Kalman filter.
		// In this case a 6x6 matrix: Q = B*B_transpose*acceleration_variance
		FStackMatrixMxN<6,6> ProcessNoiseMatrix;

		// Build the matrices needed for prediction
		MakeStateTransitionMatrix(TimeDelta, StateTransitionMatrix); // 6x6 Matrix F
		MakeControlModelMatrix(TimeDelta, ControlModelMatrix); // 6x3 Matrix B
		MakeControlInputVector(AccelerationControl, ControlInputVector); // 3x1 Vector u
		MakeProcessNoiseMatrix(AccelerationVariance, TimeDelta, ProcessNoiseMatrix); // 6x6 Matrix Q

		// x_(k|k-1) = F*x_(k-1|k-1) + B*u
		MatrixMultiplyAccumulate(StateTransitionMatrix, StateVector, PredictStateVector);
		MatrixMultiplyAccumulate(ControlModelMatrix, ControlInputVector, PredictStateVector);

		// P_(k|k-1) = F*P_(k-1|k-1)*transpose(F) + Q
		{
			FStackMatrixMxN<6, 6> F_times_P_k_minus_1;

			MatrixTranspose(StateTransitionMatrix, StateTransitionMatrixTranspose);
			MatrixMultiply(StateTransitionMatrix, StateCovarianceMatrix, F_times_P_k_minus_1);
			MatrixMultiply(F_times_P_k_minus_1, StateTransitionMatrixTranspose, PredictStateCovarianceMatrix);
			MatrixAdd(PredictStateCovarianceMatrix, ProcessNoiseMatrix, PredictStateCovarianceMatrix);
		}
	}

	// -- Correction Phase--
	{
		// 'x_(k|k)' - Corrected state given the predicted state
		// In this case a 6x1 vector containing position and velocity
		FStackMatrixMxN<6, 1> CorrectedStateVector;

		// 'P_(k|k)' - The covariance matrix of the state 'x_(k)'
		// Captures the uncertainty of the corrected state vector
		FStackMatrixMxN<6, 6> CorrectedStateCovarianceMatrix;

		// 'y' - The error between the measurement and what we expected
		FStackMatrixMxN<3, 1> ErrorVector;

		// 'S^-1' - The inverse of covariance of the move
		FStackMatrixMxN<3, 3> MoveCovarianceInverseMatrix;

		// 'K' - The Kalman gain, which is used to compute the corrected 'x_(k)' and 'P_(k)'
		FStackMatrixMxN<6, 3> KalmanGain;

		// y = z - H*x_(k|k-1)
		// In this case 'y' is a 3x1 vector where:
		//  z = the sensor measurement vector of the system (in this case the measure position)
		//  H = the observation matrix that maps the the state vector into sensor vector space
		ComputeErrorVector(PredictStateVector, MeasuredPosition, ErrorVector);

		// S^-1 = (H * P_(k|k-1) * transpose(H) + R)^-1
		// In this case S is a 3x3 matrix where:
		//  H = the observation matrix that maps the the state vector into sensor vector space
		//  P_(k|k-1) = The predicted state covariance matrix computed in the update phase
		//  R = The covariance matrix of the measurement vector 'z'
		{
			FStackMatrixMxN<3, 3> HPH_t;

			ComputHPHtMatrix(PredictStateCovarianceMatrix, HPH_t);
			MatrixAdd(HPH_t, MeasurementNoiseMatrix, MoveCovarianceInverseMatrix);
			MatrixInvert(MoveCovarianceInverseMatrix);
		}

		// K = P_(k|k-1)*transpose(H)*S^-1
		{
			FStackMatrixMxN<6, 3> PH_t;

			ComputPHtMatrix(PredictStateCovarianceMatrix, PH_t);
			MatrixMultiply(PH_t, MoveCovarianceInverseMatrix, KalmanGain);
		}

		// x_(k|k) = x_(k|k-1) + K*y
		{
			FStackMatrixMxN<6, 1> Ky;

			MatrixMultiply(KalmanGain, ErrorVector, Ky);
			MatrixAdd(PredictStateVector, Ky, CorrectedStateVector);
		}

		// P_(k|k) = (I - K*H)*P_(k|k-1)
		{
			FStackMatrixMxN<6, 6> IMinusKH;

			ComputIMinusKHMatrix(KalmanGain, IMinusKH);
			MatrixMultiply(IMinusKH, PredictStateCovarianceMatrix, CorrectedStateCovarianceMatrix);
		}

		// x_(k-1|k-1)= x_(k|k)
		StateVector.CopyFromMatrix(CorrectedStateVector);

		// P_(k-1|k-1) = P_(k|k)
		StateCovarianceMatrix.CopyFromMatrix(CorrectedStateCovarianceMatrix);
	}
}

//-- private methods ----
static void MakeStateTransitionMatrix(
	const float dT,
	FStackMatrixMxN<6,6> &m)
{
	m.SetIdentity();
	m[0][3] = dT;
	m[1][4] = dT;
	m[2][5] = dT;
}

static void MakeStateTransitionMatrix(
	const float dT,
	FStackMatrixMxN<2, 2> &m)
{
	m.SetIdentity();
	m[0][1] = dT;
}

static void MakeControlModelMatrix(
	const float dT,
	FStackMatrixMxN<6,3> &m)
{
	const float one_half_delta_t_sq = 0.5f*dT*dT;

	// Control Model B used to apply acceleration input onto state
	m[0][0] = one_half_delta_t_sq; m[0][1] = 0.f;                 m[0][2] = 0.f;
	m[1][0] = 0.f;                 m[1][1] = one_half_delta_t_sq; m[1][2] = 0.f;
	m[2][0] = 0.f;                 m[2][1] = 0.f;                 m[2][2] = one_half_delta_t_sq;
	m[3][0] = dT;                  m[3][1] = 0.f;                 m[3][2] = 0.f;
	m[4][0] = 0.f;                 m[4][1] = dT;                  m[4][2] = 0.f;
	m[5][0] = 0.f;                 m[5][1] = 0.f;                 m[5][2] = dT;
}

static void MakeControlModelMatrix(
	const float dT,
	FStackMatrixMxN<2, 1> &m)
{
	// Control Model B used to apply acceleration input onto state
	m[0][0] = 0.5f*dT*dT;
	m[1][0] = dT;
}

static void MakeProcessNoiseMatrix(
	const float varA, // variance of acceleration due to noise
	const float dT,
	FStackMatrixMxN<6,6> &m)
{
	const float dT_squared = dT*dT;
	const float q2 = varA * dT_squared;
	const float q3 = varA * 0.5f*dT_squared*dT;
	const float q4 = varA * 0.25f*dT_squared*dT_squared;

	// Q = B * Transpose(B) * var(acceleration)
	m[0][0] =  q4; m[0][1] = 0.f; m[0][2] = 0.f;   m[0][3] =  q3; m[0][4] = 0.f; m[0][5] = 0.f;
	m[1][0] = 0.f; m[1][1] =  q4; m[1][2] = 0.f;   m[1][3] = 0.f; m[1][4] =  q3; m[1][5] = 0.f;
	m[2][0] = 0.f; m[2][1] = 0.f; m[2][2] =  q4;   m[2][3] = 0.f; m[2][4] = 0.f; m[2][5] =  q3;

	m[3][0] =  q3; m[3][1] = 0.f; m[3][2] = 0.f;   m[3][3] =  q2; m[3][4] = 0.f; m[3][5] = 0.f;
	m[4][0] = 0.f; m[4][1] =  q3; m[4][2] = 0.f;   m[4][3] = 0.f; m[4][4] =  q2; m[4][5] = 0.f;
	m[5][0] = 0.f; m[5][1] = 0.f; m[5][2] =  q3;   m[5][3] = 0.f; m[5][4] = 0.f; m[5][5] =  q2;
}

static void MakeProcessNoiseMatrix(
	const float varA, // variance of acceleration due to noise
	const float dT,
	FStackMatrixMxN<2, 2> &m)
{
	const float dT_squared = dT*dT;
	const float q2 = varA * dT_squared;
	const float q3 = varA * 0.5f*dT_squared*dT;
	const float q4 = varA * 0.25f*dT_squared*dT_squared;

	// Q = B * Transpose(B) * var(acceleration)
	m[0][0] = q4; m[0][1] = q3;
	m[1][0] = q3; m[1][1] = q2;
}

static void MakeControlInputVector(
	const FVector &v,
	FStackMatrixMxN<3,1> &m)
{
	m[0][0] = v.X;
	m[1][0] = v.Y;
	m[2][0] = v.Z;
}

static void ComputeErrorVector(
	const FStackMatrixMxN<6,1> &PredictStateVector,
	const FVector &MeasuredPosition,
	FStackMatrixMxN<3, 1> &ErrorVector)
{
	ErrorVector[0][0] = MeasuredPosition.X - PredictStateVector[0][0];
	ErrorVector[1][0] = MeasuredPosition.Y - PredictStateVector[1][0];
	ErrorVector[2][0] = MeasuredPosition.Z - PredictStateVector[2][0];
}

static void ComputeErrorVector(
	const FStackMatrixMxN<2, 1> &PredictStateVector,
	const float MeasuredPosition,
	FStackMatrixMxN<1, 1> &ErrorVector)
{
	ErrorVector[0][0] = MeasuredPosition - PredictStateVector[0][0];
}

static void ComputHPHtMatrix(
	const FStackMatrixMxN<6,6> &P,
	FStackMatrixMxN<3, 3> &HPH_t)
{
	// Compute H * P_(k|k-1) * transpose(H)
	// Where H (a.k.a the Extraction matrix) equals:
	//
	// |1 0 0 0 0 0|
	// |0 1 0 0 0 0|
	// |0 0 1 0 0 0|
	//
	// This has the effect of extracting the upper-left 3x3 portion of P
	HPH_t[0][0] = P[0][0]; HPH_t[0][1] = P[0][1]; HPH_t[0][2] = P[0][2];
	HPH_t[1][0] = P[1][0]; HPH_t[1][1] = P[1][1]; HPH_t[1][2] = P[1][2];
	HPH_t[2][0] = P[2][0]; HPH_t[2][1] = P[2][1]; HPH_t[2][2] = P[2][2];
}

static void ComputHPHtMatrix(
	const FStackMatrixMxN<2, 2> &P,
	FStackMatrixMxN<1, 1> &HPH_t)
{
	// Compute H * P_(k|k-1) * transpose(H)
	// Where H (a.k.a the Extraction matrix) equals:
	//
	// |1 0|
	//
	// This has the effect of extracting the upper-left 1x1 portion of P
	HPH_t[0][0] = P[0][0];
}

static void ComputPHtMatrix(
	const FStackMatrixMxN<6, 6> &P,
	FStackMatrixMxN<6, 3> &PH_t)
{
	// Compute P_(k|k-1) * transpose(H)
	// Where transpose(H) (a.k.a the transpose of the extraction matrix) equals:
	//
	// |1 0 0|
	// |0 1 0|
	// |0 0 1|
	// |0 0 0|
	// |0 0 0|
	// |0 0 0|
	//
	// This has the effect of extracting the left 6x3 portion of P
	PH_t[0][0] = P[0][0]; PH_t[0][1] = P[0][1]; PH_t[0][2] = P[0][2];
	PH_t[1][0] = P[1][0]; PH_t[1][1] = P[1][1]; PH_t[1][2] = P[1][2];
	PH_t[2][0] = P[2][0]; PH_t[2][1] = P[2][1]; PH_t[2][2] = P[2][2];
	PH_t[3][0] = P[3][0]; PH_t[3][1] = P[3][1]; PH_t[3][2] = P[3][2];
	PH_t[4][0] = P[4][0]; PH_t[4][1] = P[4][1]; PH_t[4][2] = P[4][2];
	PH_t[5][0] = P[5][0]; PH_t[5][1] = P[5][1]; PH_t[5][2] = P[5][2];
}

static void ComputPHtMatrix(
	const FStackMatrixMxN<2, 2> &P,
	FStackMatrixMxN<2, 1> &PH_t)
{
	// Compute P_(k|k-1) * transpose(H)
	// Where transpose(H) (a.k.a the transpose of the extraction matrix) equals:
	//
	// |1|
	// |0|
	//
	// This has the effect of extracting the left 2x1 portion of P
	PH_t[0][0] = P[0][0];
	PH_t[1][0] = P[1][0];
}

static void ComputIMinusKHMatrix(
	const FStackMatrixMxN<6, 3> &K,
	FStackMatrixMxN<6, 6> &IMinusKH)
{
	// Compute I - K*H:
	// Where I is a 6x6 identity matrix
	// K (the the 6x3 Kalman gain) equals:
	// |k00 k01 k02|
	// |k10 k11 k12|
	// |k20 k21 k22|
	// |k30 k31 k32|
	// |k40 k41 k42|
	// |k50 k51 k52|
	// And H (a.k.a the Extraction matrix) equals:
	// |1 0 0 0 0 0|
	// |0 1 0 0 0 0|
	// |0 0 1 0 0 0|

	// This has the effect of extracting the left 6x3 portion of P
	// from which the identity matrix subtracts the upper-left 3x3 portion.
	IMinusKH[0][0] = 1.f-K[0][0]; IMinusKH[0][1] =    -K[0][1]; IMinusKH[0][2] =    -K[0][2]; IMinusKH[0][3] = 0.f; IMinusKH[0][4] = 0.f; IMinusKH[0][5] = 0.f;
	IMinusKH[1][0] =    -K[1][0]; IMinusKH[1][1] = 1.f-K[1][1]; IMinusKH[1][2] =    -K[1][2]; IMinusKH[1][3] = 0.f; IMinusKH[1][4] = 0.f; IMinusKH[1][5] = 0.f;
	IMinusKH[2][0] =    -K[2][0]; IMinusKH[2][1] =    -K[2][1]; IMinusKH[2][2] = 1.f-K[2][2]; IMinusKH[2][3] = 0.f; IMinusKH[2][4] = 0.f; IMinusKH[2][5] = 0.f;
	IMinusKH[3][0] =    -K[3][0]; IMinusKH[3][1] =    -K[3][1]; IMinusKH[3][2] =    -K[3][2]; IMinusKH[3][3] = 1.f; IMinusKH[3][4] = 0.f; IMinusKH[3][5] = 0.f;
	IMinusKH[4][0] =    -K[4][0]; IMinusKH[4][1] =    -K[4][1]; IMinusKH[4][2] =    -K[4][2]; IMinusKH[4][3] = 0.f; IMinusKH[4][4] = 1.f; IMinusKH[4][5] = 0.f;
	IMinusKH[5][0] =    -K[5][0]; IMinusKH[5][1] =    -K[5][1]; IMinusKH[5][2] =    -K[5][2]; IMinusKH[5][3] = 0.f; IMinusKH[5][4] = 0.f; IMinusKH[5][5] = 1.f;
}

static void ComputIMinusKHMatrix(
	const FStackMatrixMxN<2, 1> &K,
	FStackMatrixMxN<2, 2> &IMinusKH)
{
	// Compute I - K*H:
	// Where I is a 6x6 identity matrix
	// K (the the 2x1 Kalman gain) equals:
	// |k00|
	// |k10|
	// And H (a.k.a the Extraction matrix) equals:
	// |1 0|

	// This has the effect of extracting the left 6x3 portion of P
	// from which the identity matrix subtracts the upper-left 2x2 portion.
	IMinusKH[0][0] = 1.f - K[0][0]; IMinusKH[0][1] = 0.f;
	IMinusKH[1][0] =      -K[1][0]; IMinusKH[1][1] = 1.f;
}

static void MatrixInvert(
	FStackMatrixMxN<3,3> &m)
{
	FMatrix temp(ForceInitToZero);
	FMatrix inverseTemp;
	
	// For 3x3 matrices it's faster to convert this over to an unreal matrix,
	// invert that, and convert back again.
	m.CopyToUnreal4x4Matrix(temp);
	temp.M[3][3] = 1.f;

	inverseTemp= temp.Inverse();
	m.CopyFromUnreal4x4Matrix(inverseTemp);
}

static void MatrixInvert(
	FStackMatrixMxN<1, 1> &m)
{
	if (!FMath::IsNearlyZero(m[0][0]))
	{
		m[0][0] = 1.f / m[0][0];
	}
	else
	{
		m[0][0] = 1.f;
	}
}

//-- Unit Testing ---
void FPSMove1DKalmanFilter::Test()
{
	// Recorded sample data
	float positionSamples[] = {
		-1146.9f,
		-1164.9f,
		-1146.9f,
		-1164.9f,
		-1183.0f,
		-1164.9f,
		-1164.9f,
		-1146.9f,
		-1183.1f,
		-1164.9f,
		-1164.9f,
		-1164.9f,
		-1164.8f,
		-1183.0f,
		-1164.9f,
		-1146.9f,
		-1164.9f,
		-1164.8f,
		-1146.9f,
		-1147.0f,
		-1164.9f,
		-1164.8f,
		-1146.8f,
		-1164.9f,
		-1183.1f,
		-1164.9f,
		-1183.1f,
		-1164.8f,
		-1183.1f,
		-1164.8f,
		-1164.8f,
		-1146.9f,
		-1183.1f,
		-1164.8f,
		-1146.9f,
		-1146.8f,
		-1164.9f,
		-1146.9f,
		-1164.9f,
		-1129.4f,
		-1164.8f,
		-1146.9f,
		-1183.1f,
		-1183.0f,
		-1164.8f,
		-1146.9f,
		-1183.0f,
		-1183.0f,
		-1164.8f,
		-1164.8f,
		-1164.8f,
		-1164.9f,
		-1164.8f,
		-1164.9f,
		-1183.1f,
		-1164.9f,
		-1183.1f,
		-1164.9f,
		-1164.9f,
		-1164.8f,
		-1164.9f,
		-1164.9f,
		-1164.8f,
		-1183.0f,
		-1146.9f,
		-1164.9f,
		-1183.1f,
		-1164.9f,
		-1164.9f,
		-1183.1f,
		-1183.0f,
		-1164.9f,
		-1164.9f,
		-1164.8f,
		-1146.9f,
		-1164.8f,
		-1164.9f,
		-1164.8f,
		-1164.9f,
		-1183.1f,
		-1164.9f,
		-1146.9f,
		-1164.9f,
		-1164.9f,
		-1164.9f,
		-1146.9f,
		-1164.8f,
		-1146.9f,
		-1164.8f,
		-1164.8f,
		-1164.9f,
		-1164.9f,
		-1164.8f,
		-1164.9f,
		-1164.9f,
		-1146.9f,
		-1164.8f,
		-1164.9f,
		-1164.9f,
		-1164.8f,
		-1164.9f,
		-1164.9f,
		-1164.9f,
		-1146.9f,
		-1164.8f,
		-1164.8f,
		-1146.9f,
		-1183.1f,
		-1183.1f,
		-1146.9f,
		-1164.8f,
		-1164.9f,
		-1164.9f,
		-1146.9f,
		-1164.9f,
		-1183.0f,
		-1164.9f,
		-1164.9f,
		-1183.1f,
		-1164.9f,
		-1164.8f,
		-1164.8f,
		-1164.7f,
		-1146.9f,
		-1146.9f,
		-1183.0f,
		-1164.8f,
		-1146.9f,
		-1164.9f,
		-1164.9f,
		-1164.9f,
		-1164.9f,
		-1183.1f,
		-1183.1f,
		-1164.8f,
		-1164.8f,
		-1183.1f,
		-1146.8f,
		-1146.9f,
		-1164.9f,
		-1146.9f,
		-1164.9f,
		-1164.8f,
		-1164.9f,
		-1146.9f,
		-1164.9f,
		-1164.8f,
		-1164.9f,
		-1164.9f,
		-1164.8f,
		-1183.1f,
		-1164.9f,
		-1164.9f,
		-1164.9f,
		-1146.9f,
		-1164.8f,
		-1164.9f,
		-1164.9f,
		-1164.9f,
		-1146.9f,
		-1183.1f,
		-1183.1f,
		-1183.0f,
		-1164.9f,
		-1164.9f,
		-1164.9f,
		-1146.9f,
		-1183.1f,
	};
	
	bool success;
	FPSMove1DKalmanFilter PositionFilter;

	//const float accelerationVariance = 1754.46228f;
	const float accelerationVariance = 0.f;
	const float measurementCovarianceMatrix = 132.710083f;
	//const float measurementCovarianceMatrix = 14.8531485f;
	//const float measurementCovarianceMatrix = 28.6374550f;

	PositionFilter.Initialize(accelerationVariance, measurementCovarianceMatrix, positionSamples[0]);

	// Simulate a run of the filter for 5 seconds
	{
		float SimTime = 0;
		UE_LOG(LogPSMove, Log, TEXT("    Time\t    RawX\t       X\t      VX\t    VarX\t   VarVX"));

		for (int32 sampleIndex = 0; sampleIndex < ARRAY_COUNT(positionSamples); ++sampleIndex)
		{
			const float TimeDelta = 0.03333f; // 30 FPS
			float WorldRelativeRawPosition = positionSamples[sampleIndex];
			float WorldRelativeRawAcceleration = 0.f;

			PositionFilter.Update(WorldRelativeRawPosition, WorldRelativeRawAcceleration, TimeDelta);

			float WorldRelativeFilteredPosition = PositionFilter.GetFilteredPosition();
			float WorldRelativeFilteredVelocity = PositionFilter.GetFilteredVelocity();
			float FilteredPositionVariance = PositionFilter.GetFilteredPositionVariance();
			float FilteredVelocityVariance = PositionFilter.GetFilteredVelocityVariance();

			UE_LOG(LogPSMove, Log,
				TEXT("%8.2f\t%8.1f\t%8.1f\t%8.1f\t%8.1f\t%8.1f"),
				SimTime,
				WorldRelativeRawPosition,
				WorldRelativeFilteredPosition,
				WorldRelativeFilteredVelocity,
				FilteredPositionVariance,
				FilteredVelocityVariance);

			if (FMath::Abs(WorldRelativeFilteredPosition - WorldRelativeRawPosition) > 100.f)
			{
				// Any deviation by more than 100mm means something has gone horribly wrong
				success = false;
			}

			if (WorldRelativeFilteredVelocity > 100.f)
			{
				// Any deviation by more than 100mm/sec means something has gone horribly wrong
				success = false;
			}

			SimTime += TimeDelta;
		}
	}

	assert(success);
}

void FPSMovePositionKalmanFilter::Test()
{
	// Recorded sample data
	FVector positionSamples[] = {
		FVector(-1146.9f, -4807.3f, 628.1f),
		FVector(-1164.9f, -4813.2f, 619.6f),
		FVector(-1146.9f, -4807.2f, 628.1f),
		FVector(-1164.9f, -4813.2f, 619.6f),
		FVector(-1183.0f, -4819.3f, 611.5f),
		FVector(-1164.9f, -4813.2f, 619.6f),
		FVector(-1164.9f, -4813.2f, 619.6f),
		FVector(-1146.9f, -4807.3f, 628.1f),
		FVector(-1183.1f, -4819.3f, 611.4f),
		FVector(-1164.9f, -4813.3f, 619.5f),
		FVector(-1164.9f, -4813.2f, 619.6f),
		FVector(-1164.9f, -4813.2f, 619.6f),
		FVector(-1164.8f, -4813.2f, 619.7f),
		FVector(-1183.0f, -4819.3f, 611.5f),
		FVector(-1164.9f, -4813.2f, 619.6f),
		FVector(-1146.9f, -4807.2f, 628.1f),
		FVector(-1164.9f, -4813.2f, 619.6f),
		FVector(-1164.8f, -4813.2f, 619.7f),
		FVector(-1146.9f, -4807.2f, 628.1f),
		FVector(-1147.0f, -4807.1f, 627.9f),
		FVector(-1164.9f, -4813.2f, 619.6f),
		FVector(-1164.8f, -4813.2f, 619.7f),
		FVector(-1146.8f, -4807.2f, 628.4f),
		FVector(-1164.9f, -4813.2f, 619.6f),
		FVector(-1183.1f, -4819.3f, 611.3f),
		FVector(-1164.9f, -4813.2f, 619.6f),
		FVector(-1183.1f, -4819.3f, 611.3f),
		FVector(-1164.8f, -4813.1f, 619.8f),
		FVector(-1183.1f, -4819.3f, 611.3f),
		FVector(-1164.8f, -4813.2f, 619.7f),
		FVector(-1164.8f, -4813.2f, 619.7f),
		FVector(-1146.9f, -4807.2f, 628.3f),
		FVector(-1183.1f, -4819.3f, 611.3f),
		FVector(-1164.8f, -4813.2f, 619.7f),
		FVector(-1146.9f, -4807.2f, 628.1f),
		FVector(-1146.8f, -4807.2f, 628.4f),
		FVector(-1164.9f, -4813.2f, 619.6f),
		FVector(-1146.9f, -4807.2f, 628.1f),
		FVector(-1164.9f, -4813.2f, 619.6f),
		FVector(-1129.4f, -4801.2f, 636.3f),
		FVector(-1164.8f, -4813.2f, 619.7f),
		FVector(-1146.9f, -4807.2f, 628.1f),
		FVector(-1183.1f, -4819.3f, 611.3f),
		FVector(-1183.0f, -4819.3f, 611.5f),
		FVector(-1164.8f, -4813.2f, 619.7f),
		FVector(-1146.9f, -4807.2f, 628.1f),
		FVector(-1183.0f, -4819.3f, 611.5f),
		FVector(-1183.0f, -4819.3f, 611.5f),
		FVector(-1164.8f, -4813.2f, 619.8f),
		FVector(-1164.8f, -4813.2f, 619.8f),
		FVector(-1164.8f, -4813.2f, 619.7f),
		FVector(-1164.9f, -4813.1f, 619.7f),
		FVector(-1164.8f, -4813.2f, 619.7f),
		FVector(-1164.9f, -4813.2f, 619.6f),
		FVector(-1183.1f, -4819.3f, 611.3f),
		FVector(-1164.9f, -4813.2f, 619.6f),
		FVector(-1183.1f, -4819.3f, 611.3f),
		FVector(-1164.9f, -4813.2f, 619.6f),
		FVector(-1164.9f, -4813.2f, 619.6f),
		FVector(-1164.8f, -4813.2f, 619.7f),
		FVector(-1164.9f, -4813.2f, 619.6f),
		FVector(-1164.9f, -4813.2f, 619.6f),
		FVector(-1164.8f, -4813.2f, 619.7f),
		FVector(-1183.0f, -4819.3f, 611.5f),
		FVector(-1146.9f, -4807.2f, 628.3f),
		FVector(-1164.9f, -4813.2f, 619.6f),
		FVector(-1183.1f, -4819.3f, 611.3f),
		FVector(-1164.9f, -4813.2f, 619.6f),
		FVector(-1164.9f, -4813.2f, 619.6f),
		FVector(-1183.1f, -4819.3f, 611.3f),
		FVector(-1183.0f, -4819.3f, 611.5f),
		FVector(-1164.9f, -4813.2f, 619.6f),
		FVector(-1164.9f, -4813.2f, 619.6f),
		FVector(-1164.8f, -4813.2f, 619.7f),
		FVector(-1146.9f, -4807.2f, 628.1f),
		FVector(-1164.8f, -4813.2f, 619.7f),
		FVector(-1164.9f, -4813.2f, 619.6f),
		FVector(-1164.8f, -4813.2f, 619.7f),
		FVector(-1164.9f, -4813.2f, 619.6f),
		FVector(-1183.1f, -4819.3f, 611.3f),
		FVector(-1164.9f, -4813.3f, 619.5f),
		FVector(-1146.9f, -4807.2f, 628.1f),
		FVector(-1164.9f, -4813.2f, 619.6f),
		FVector(-1164.9f, -4813.2f, 619.6f),
		FVector(-1164.9f, -4813.2f, 619.6f),
		FVector(-1146.9f, -4807.2f, 628.2f),
		FVector(-1164.8f, -4813.2f, 619.7f),
		FVector(-1146.9f, -4807.2f, 628.1f),
		FVector(-1164.8f, -4813.2f, 619.7f),
		FVector(-1164.8f, -4813.2f, 619.7f),
		FVector(-1164.9f, -4813.2f, 619.6f),
		FVector(-1164.9f, -4813.2f, 619.6f),
		FVector(-1164.8f, -4813.2f, 619.7f),
		FVector(-1164.9f, -4813.2f, 619.6f),
		FVector(-1164.9f, -4813.2f, 619.6f),
		FVector(-1146.9f, -4807.2f, 628.1f),
		FVector(-1164.8f, -4813.2f, 619.7f),
		FVector(-1164.9f, -4813.2f, 619.6f),
		FVector(-1164.9f, -4813.2f, 619.6f),
		FVector(-1164.8f, -4813.2f, 619.7f),
		FVector(-1164.9f, -4813.2f, 619.6f),
		FVector(-1164.9f, -4813.2f, 619.6f),
		FVector(-1164.9f, -4813.2f, 619.6f),
		FVector(-1146.9f, -4807.2f, 628.1f),
		FVector(-1164.8f, -4813.2f, 619.7f),
		FVector(-1164.8f, -4813.2f, 619.7f),
		FVector(-1146.9f, -4807.2f, 628.3f),
		FVector(-1183.1f, -4819.3f, 611.3f),
		FVector(-1183.1f, -4819.3f, 611.3f),
		FVector(-1146.9f, -4807.2f, 628.1f),
		FVector(-1164.8f, -4813.2f, 619.7f),
		FVector(-1164.9f, -4813.2f, 619.6f),
		FVector(-1164.9f, -4813.2f, 619.6f),
		FVector(-1146.9f, -4807.2f, 628.1f),
		FVector(-1164.9f, -4813.2f, 619.6f),
		FVector(-1183.0f, -4819.3f, 611.5f),
		FVector(-1164.9f, -4813.2f, 619.6f),
		FVector(-1164.9f, -4813.2f, 619.6f),
		FVector(-1183.1f, -4819.3f, 611.3f),
		FVector(-1164.9f, -4813.2f, 619.6f),
		FVector(-1164.8f, -4813.1f, 619.8f),
		FVector(-1164.8f, -4813.2f, 619.7f),
		FVector(-1164.7f, -4813.1f, 620.3f),
		FVector(-1146.9f, -4807.2f, 628.1f),
		FVector(-1146.9f, -4807.2f, 628.3f),
		FVector(-1183.0f, -4819.3f, 611.5f),
		FVector(-1164.8f, -4813.2f, 619.7f),
		FVector(-1146.9f, -4807.2f, 628.1f),
		FVector(-1164.9f, -4813.2f, 619.6f),
		FVector(-1164.9f, -4813.2f, 619.6f),
		FVector(-1164.9f, -4813.2f, 619.6f),
		FVector(-1164.9f, -4813.2f, 619.6f),
		FVector(-1183.1f, -4819.3f, 611.3f),
		FVector(-1183.1f, -4819.3f, 611.3f),
		FVector(-1164.8f, -4813.2f, 619.7f),
		FVector(-1164.8f, -4813.2f, 619.7f),
		FVector(-1183.1f, -4819.3f, 611.3f),
		FVector(-1146.8f, -4807.2f, 628.4f),
		FVector(-1146.9f, -4807.2f, 628.1f),
		FVector(-1164.9f, -4813.2f, 619.6f),
		FVector(-1146.9f, -4807.3f, 628.1f),
		FVector(-1164.9f, -4813.2f, 619.6f),
		FVector(-1164.8f, -4813.2f, 619.7f),
		FVector(-1164.9f, -4813.2f, 619.6f),
		FVector(-1146.9f, -4807.2f, 628.1f),
		FVector(-1164.9f, -4813.2f, 619.6f),
		FVector(-1164.8f, -4813.2f, 619.7f),
		FVector(-1164.9f, -4813.2f, 619.6f),
		FVector(-1164.9f, -4813.2f, 619.6f),
		FVector(-1164.8f, -4813.2f, 619.7f),
		FVector(-1183.1f, -4819.3f, 611.3f),
		FVector(-1164.9f, -4813.2f, 619.6f),
		FVector(-1164.9f, -4813.2f, 619.6f),
		FVector(-1164.9f, -4813.2f, 619.6f),
		FVector(-1146.9f, -4807.2f, 628.1f),
		FVector(-1164.8f, -4813.2f, 619.7f),
		FVector(-1164.9f, -4813.2f, 619.6f),
		FVector(-1164.9f, -4813.2f, 619.6f),
		FVector(-1164.9f, -4813.2f, 619.6f),
		FVector(-1146.9f, -4807.2f, 628.3f),
		FVector(-1183.1f, -4819.3f, 611.3f),
		FVector(-1183.1f, -4819.3f, 611.3f),
		FVector(-1183.0f, -4819.3f, 611.5f),
		FVector(-1164.9f, -4813.2f, 619.6f),
		FVector(-1164.9f, -4813.2f, 619.6f),
		FVector(-1164.9f, -4813.2f, 619.6f),
		FVector(-1146.9f, -4807.2f, 628.1f),
		FVector(-1183.1f, -4819.3f, 611.3f)
	};

	bool success;
	FPSMovePositionKalmanFilter PositionFilter;

	//const float accelerationVariance = 1754.46228f;
	const float accelerationVariance = 0.f;
	FStackMatrixMxN<3, 3> measurementCovarianceMatrix;

	measurementCovarianceMatrix[0][0] = 132.710083f; measurementCovarianceMatrix[0][1] = 0.f; measurementCovarianceMatrix[0][2] = 0.f;
	measurementCovarianceMatrix[1][0] = 0.f; measurementCovarianceMatrix[1][1] = 14.8531485f; measurementCovarianceMatrix[1][2] = 0.f;
	measurementCovarianceMatrix[2][0] = 0.f; measurementCovarianceMatrix[2][1] = 0.f; measurementCovarianceMatrix[2][2] = 28.6374550f;

	PositionFilter.Initialize(accelerationVariance, measurementCovarianceMatrix, positionSamples[0]);

	// Simulate a run of the filter for 5 seconds
	{
		float SimTime = 0;
		UE_LOG(LogPSMove, Log, TEXT("    Time\t    RawX\t    RawY\t    RawZ\t       X\t       Y\t       Z\t      VX\t      VY\t      VZ\t    VarX\t    VarY\t    VarZ\t   VarVX\t   VarVY\t   VarVZ"));

		for (int32 sampleIndex = 0; sampleIndex < ARRAY_COUNT(positionSamples); ++sampleIndex)
		{
			const float TimeDelta = 0.03333f; // 30 FPS
			FVector WorldRelativeRawPosition = positionSamples[sampleIndex];
			FVector WorldRelativeRawAcceleration = FVector(0.f, 0.f, 0.f);

			PositionFilter.Update(WorldRelativeRawPosition, WorldRelativeRawAcceleration, TimeDelta);

			FVector WorldRelativeFilteredPosition = PositionFilter.GetFilteredPosition();
			FVector WorldRelativeFilteredVelocity = PositionFilter.GetFilteredVelocity();
			FVector FilteredPositionVariance = PositionFilter.GetFilteredPositionVariance();
			FVector FilteredVelocityVariance = PositionFilter.GetFilteredVelocityVariance();

			UE_LOG(LogPSMove, Log,
				TEXT("%8.2f\t%8.1f\t%8.1f\t%8.1f\t%8.1f\t%8.1f\t%8.1f\t%8.1f\t%8.1f\t%8.1f\t%8.1f\t%8.1f\t%8.1f\t%8.1f\t%8.1f\t%8.1f"),
				SimTime,
				WorldRelativeRawPosition.X,
				WorldRelativeRawPosition.Y,
				WorldRelativeRawPosition.Z,
				WorldRelativeFilteredPosition.X,
				WorldRelativeFilteredPosition.Y,
				WorldRelativeFilteredPosition.Z,
				WorldRelativeFilteredVelocity.X,
				WorldRelativeFilteredVelocity.Y,
				WorldRelativeFilteredVelocity.Z,
				FilteredPositionVariance.X,
				FilteredPositionVariance.Y,
				FilteredPositionVariance.Z,
				FilteredVelocityVariance.X,
				FilteredVelocityVariance.Y,
				FilteredVelocityVariance.Z);

			if (FVector::Dist(WorldRelativeFilteredPosition, WorldRelativeRawPosition) > 100.f)
			{
				// Any deviation by more than 100mm means something has gone horribly wrong
				success = false;
			}

			if (WorldRelativeFilteredVelocity.Size() > 100.f)
			{
				// Any deviation by more than 100mm/sec means something has gone horribly wrong
				success = false;
			}

			SimTime += TimeDelta;
		}
	}

	assert(success);
}