//
//  FPSMoveCalibration.h
//  Utility classes used to build calibration data needed by the controllers at runtime
//
//  Created by Brendan Walker on 2015-06-23.
//
#pragma once

// -- includes ----
#include "FPSMoveMatrix.h"
#include "FPSMoveClock.h"

// -- pre-declarations ----
struct FPSMoveRawSharedData_Base;
struct FPSMoveRawControllerData_Base;

// -- interface ----
class FPSMoveCalibration
{
public:
	FPSMoveCalibration(FPSMoveRawSharedData_TLS *SharedData);

	// Interface
	void Initialize();
	void SharedUpdate();
	void ControllerUpdate(const int32 controllerIndex, PSMove* psmove, const FPSMoveRawControllerData_Base *controllerData);
	void OnControllerCountChanged();

	// Accessors
	FORCEINLINE const FPSMoveControllerClock &GetClock() const { return Clock; }
	FORCEINLINE const FMatrix &GetPSMoveSpaceToHMDSpace() const { return PSMoveSpaceToHMDSpace; }
	FORCEINLINE float GetAccelerationVariance() const { return AccelerationVariance; }
	FORCEINLINE FVector GetPSMoveSpaceAccelerationSkew() const { return PSMoveSpaceAccelerationSkew; }
	FORCEINLINE const FStackMatrixMxN<3, 3> &GetMeasurementNoiseMatrix() const { return MeasurementNoiseMatrix; }
	FORCEINLINE float GetGravityUnits() const {
		// 'g' in terms of mm is 9806.65mm/s^2
		return 9806.65f;
	}

private:
	void SetCalibrationStatus(EPSMoveCalibrationStatusEnum::Type state);
	void ResetNoiseSampleData();
	void ResetAlignmentSampleData();
	void ResetCalibrationSampleData();
	void RecordNoiseSample(const FPSMoveRawControllerData_Base *controllerData);
	void RecordAlignmentSample(const FPSMoveRawControllerData_Base *controllerData);
	void LoadCalibrationFile();
	void SaveCalibrationFile();
	bool IsNoiseUncalibrated() const;

	FORCEINLINE TArray<FVector> &GetPSMoveSamplePoints() { return ScratchPoints1; }
	FORCEINLINE TArray<FVector> &GetHMDSamplePoints() { return ScratchPoints2; }

	FORCEINLINE TArray<FVector> &GetPositionNoiseSamplePoints() { return ScratchPoints1; }
	FORCEINLINE TArray<FVector> &GetAccelerationNoiseSampleVectors() { return ScratchPoints2; }

private:
	// Clock used for timer
	FPSMoveControllerClock Clock;
	float StateTimer;

	// The index of the controller we're pulling calibration data from
	int32 CalibrationControllerIndex;

	// Scratch pad buffers for temp calibration data stored for multiple frames
	TArray<FVector> ScratchPoints1;
	TArray<FVector> ScratchPoints2;

	// Converts a position from PSMove space to Unreal world space
	FMatrix PSMoveSpaceToHMDSpace;

	// The variance of the noise in the PS Move acceleration data
	float AccelerationVariance;

	// Some axes don't quite read the 1g when they are supposed to.
	// This corrects for that.
	FVector PSMoveSpaceAccelerationSkew;

	// The covariance matrix of the noise in the PS Move position data
	FStackMatrixMxN<3, 3> MeasurementNoiseMatrix;

	// Calibration state shared with the PSMoveComponent
	FPSMoveRawSharedData_TLS *SharedData;

	// Used to detect when edge crossing
	bool PreviousFrameCalibrationActionRequest;
};

class AffineTransformBuilder
{
public:
	AffineTransformBuilder();

	FORCEINLINE const FMatrix &GetAffineTransform() const { return AffineTransform; }

	bool ComputeAffineLeastSquareFit(const TArray<FVector> &from_pts, const TArray<FVector> &to_pts);
	void LogTransform();
	static void Test();

private:
	FMatrix AffineTransform; // the matrix constructed by ComputeAffineLeastSquareFit
};

class AverageQuaternionBuilder
{
public:
	AverageQuaternionBuilder();

	FORCEINLINE const FQuat &GetAvageQuaternion() const { return AverageQuaternion; }

	bool ComputeAverageQuaternionFast(const TArray<FQuat> &samples);
	bool ComputeAverageQuaternionAccurate(const TArray<FQuat> &samples);
	void LogQuaternion();
	static void Test();

private:
	FQuat AverageQuaternion; // The average of all of the quaternion samples
};

class PointCloudCovarianceMatrixBuilder
{
public:
	PointCloudCovarianceMatrixBuilder();

	FORCEINLINE const FVector GetMean() const { return Mean; }
	FORCEINLINE const FMatrix &GetCovarianvceMatrix() const { return CovarianceMatrix; }

	bool ComputeCovarianceMatrix(const TArray<FVector> &samples);
	bool ComputeCovarianceMatrix(const FVector *samples, const int32 sampleCount);
	void LogMean();
	void LogCovarianveMatrix();
	static void Test();

private:
	FVector Mean;
	FMatrix CovarianceMatrix;
};