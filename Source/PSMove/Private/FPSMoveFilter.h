//
//  FPSMoveFilter.h
//  Kalman filter used to smooth out noise in the PSMoves positional tracking
//
//  Created by Brendan Walker on 2015-06-23.
//
#pragma once

#include "FPSMoveMatrix.h"

class FPSMove1DKalmanFilter
{
public:
	FPSMove1DKalmanFilter();

	FORCEINLINE bool GetIsInitialized() const { return IsInitialized; }
	float GetFilteredPosition() const;
	float GetFilteredVelocity() const;
	float GetFilteredPositionVariance() const;
	float GetFilteredVelocityVariance() const;

	void Clear();
	void Initialize(
		const float accelerationVariance,
		const float positionVariance,
		const float initialValue);
	void Update(
		const float measuredPosition,		// The world space value measured by the sensors.
		const float accelerationControl,	// The world space acceleration measured on the controller.
		const float timeDelta);				// The time delta in seconds

	static void Test();

private:
	bool IsInitialized;

	// 'x_(k-1|k-1)' - State vector for the previous frame
	// In this case a 2x1 vector containing position and velocity
	FStackMatrixMxN<2, 1> StateVector;

	// 'P_(k-1|k-1)' - The covariance matrix of the state 'x_(k-1)'
	// Captures the uncertainty of the state vector for the state vector
	// In this case a 6x6 matrix
	FStackMatrixMxN<2, 2> StateCovarianceMatrix;

	// The variance in the acceleration noise
	// Used to compute the Process Noise Matrix 'Q'
	float AccelerationVariance;

	// 'R' - The covariance matrix of the measurement vector 'z'
	// The measurement noise 'R', which is the inaccuracy introduced by the sensors 
	// It models the difference between sensor's measurement value and real position..
	// In this case 'R' is a single variance value
	FStackMatrixMxN<1, 1> MeasurementNoiseMatrix;
};

class FPSMovePositionKalmanFilter
{
public:
	FPSMovePositionKalmanFilter();

	FORCEINLINE bool GetIsInitialized() const { return IsInitialized;  }
	FVector GetFilteredPosition() const;
	FVector GetFilteredVelocity() const;
	FVector GetFilteredPositionVariance() const;
	FVector GetFilteredVelocityVariance() const;

	void Clear();
	void Initialize(
		const float accelerationVariance,
		const FStackMatrixMxN<3, 3> &measurementCovarianceMatrix,
		const FVector &position);
	void Update(
		const FVector &measuredPosition,		// The position measured by the sensors.
		const FVector &accelerationControl,	// The world space acceleration measured on the controller.
		const float timeDelta);				// The time delta in seconds

	static void Test();

private:
	bool IsInitialized;

	// 'x_(k-1|k-1)' - State vector for the previous frame
	// In this case a 6x1 vector containing position and velocity
	FStackMatrixMxN<6,1> StateVector;

	// 'P_(k-1|k-1)' - The covariance matrix of the state 'x_(k-1)'
	// Captures the uncertainty of the state vector for the state vector
	// In this case a 6x6 matrix
	FStackMatrixMxN<6, 6> StateCovarianceMatrix;

	// The variance in the acceleration noise
	// Used to compute the Process Noise Matrix 'Q'
	float AccelerationVariance;

	// 'R' - The covariance matrix of the measurement vector 'z'
	// The measurement noise 'R', which is the inaccuracy introduced by the sensors 
	// It models the difference between sensor's measurement value and real position..
	// In this case 'R' is a 3x3 matrix computed at calibration time
	FStackMatrixMxN<3, 3> MeasurementNoiseMatrix;
};