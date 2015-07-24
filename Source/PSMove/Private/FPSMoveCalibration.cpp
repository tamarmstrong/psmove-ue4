//
//  FPSMoveCalibration.cpp
//  Helper methods used to align PSMove controller transforms to the space of the VR headset 
//
//  Created by Brendan Walker on 2015-06-15.
//
#include "PSMovePrivatePCH.h"
#include "FPSMoveCalibration.h"
#include "FPSMoveMatrix.h"
#include "FPSMove.h"
#include <math.h>
#include <assert.h>
#include <random>

// -- Constants -----
static const int32 DIMENSION_COUNT = 3;
static const int32 NOISE_SAMPLES_NEEDED = 100;
static const int32 POINTS_NEEDED_FOR_CALIBRATION = 5;
static const float HITCH_DETECTION_THRESHOLD = 0.0666f; // 15FPS

static const TCHAR *CONFIG_FILE = TEXT("PSMoveSettings.ini");
static const TCHAR *PSMOVESPACETOHMDSPACE_SECTION = TEXT("PSMoveSpaceToHMDSpace");
static const TCHAR *PSMOVEQUATTOHMDQUAT_SECTION = TEXT("PSMoveQuatToHMDQuat");
static const TCHAR *NOISE_SECTION = TEXT("Noise");

static const int32 CALIBRATION_BUTTON_BITMASK = PSMove_Button::Btn_START;

static const float WAIT_FOR_STABLE_DURATION = 2.f; // seconds

// -- PSMove Calibration -----
FPSMoveCalibration::FPSMoveCalibration(
	FPSMoveRawSharedData_TLS *sharedData) :
	Clock(),
	StateTimer(0.f),
	CalibrationControllerIndex(-1),
	ScratchPoints1(),
	ScratchPoints2(),
	PSMoveSpaceToHMDSpace(ForceInitToZero),
	AccelerationVariance(0.f),
	PSMoveSpaceAccelerationSkew(0.f, 0.09f, 0.f),
	MeasurementNoiseMatrix(),
	SharedData(sharedData),
	PreviousFrameCalibrationActionRequest(false)
{

}

void FPSMoveCalibration::Initialize()
{
	PSMoveSpaceToHMDSpace.SetIdentity();
	AccelerationVariance = 0.f;
	MeasurementNoiseMatrix.SetZero();
	PreviousFrameCalibrationActionRequest = false;

	ResetCalibrationSampleData();
	LoadCalibrationFile();

	Clock.Initialize();
}

void FPSMoveCalibration::ResetNoiseSampleData()
{
	GetPositionNoiseSamplePoints().Empty();
	GetAccelerationNoiseSampleVectors().Empty();
}

void FPSMoveCalibration::ResetAlignmentSampleData()
{
	GetPSMoveSamplePoints().Empty();
	GetHMDSamplePoints().Empty();
}

void FPSMoveCalibration::ResetCalibrationSampleData()
{
	CalibrationControllerIndex = -1;
	ResetNoiseSampleData();
	ResetAlignmentSampleData();
}

void FPSMoveCalibration::SharedUpdate()
{
	// Compute the time delta since the last update
	Clock.Update();

	// Spew warnings if the worker thread is taking too long on updates
	if (Clock.TimeDeltaInSeconds > HITCH_DETECTION_THRESHOLD)
	{
		UE_LOG(LogPSMove, Warning, TEXT("FPSMoveCalibration: SharedUpdate: HITCH DETECTED! Worker thread time delta=%0.3fs"), Clock.TimeDeltaInSeconds);
	}

	// Keep track of how long we're in the current calibration state for
	StateTimer += Clock.TimeDeltaInSeconds;

	// Always wait for the PSMove component to acknowledge the previous state
	// change before attempting to update the state again (otherwise it misses state changes).
	if (!SharedData->ComponentHasNotReadWorkerData)
	{
		// Clear out stale calibration point results (regardless of calibration status)
		if (SharedData->CalibrationPointResultEnum != EPSMoveCalibrationStepResultEnum::Waiting)
		{
			SharedData->CalibrationPointResultEnum = EPSMoveCalibrationStepResultEnum::Waiting;
			SharedData->MarkDirty();
		}
	}
}

inline bool IsControllerStableAndVisible(
	const FPSMoveRawControllerData_Base *controllerData)
{
	const float kCosine10Degrees = 0.984808f;
	bool isOk = false;

	if (controllerData->CanSee)
	{
		FVector kGravityVector(0.f, 1.f, 0.f);
		FVector accelerationDirection = controllerData->PSMoveRelativeRawAcceleration;
		float accelerationMagnitude = accelerationDirection.Size();

		accelerationDirection =
			(accelerationMagnitude > SMALL_NUMBER) ?
			(accelerationDirection / accelerationMagnitude) :
			FVector(0, 0, 0);

		isOk =
			FMath::IsNearlyEqual(1.f, accelerationMagnitude, 0.1f) &&
			FVector::DotProduct(kGravityVector, accelerationDirection) >= kCosine10Degrees;
	}

	return isOk;
}

void FPSMoveCalibration::ControllerUpdate(
	const int32 controllerIndex,
	PSMove *psmove,
	const FPSMoveRawControllerData_Base *controllerData)
{
	// - Always wait for the PSMove component to acknowledge the previous state
	//   change before attempting to update the state again (otherwise it misses state changes).
	// - Ignore input from any controller not driving the calibration process.
	if (!SharedData->ComponentHasNotReadWorkerData && 
		(CalibrationControllerIndex == -1 || CalibrationControllerIndex == controllerIndex))
	{
		bool calibrationActionRequested =
			controllerData->CalibrationActionRequest && !this->PreviousFrameCalibrationActionRequest;

		switch (SharedData->CalibrationStatusEnum)
		{
			case EPSMoveCalibrationStatusEnum::Inactive:
			{
				// If the component has requested to start calibration, 
				// then start the calibration process with this controller
				if (calibrationActionRequested)
				{
					UE_LOG(LogPSMove, Log, TEXT("FPSMoveCalibration: ControllerUpdate: Calibration button pressed. Starting calibration."));

					// Remember which controller we're calibrating with
					CalibrationControllerIndex = controllerIndex;

					// Wait for the controller to become stable
					if (IsNoiseUncalibrated())
					{
						SetCalibrationStatus(EPSMoveCalibrationStatusEnum::WaitForStationary);
					}
					else
					{
						// Move on to sampling the HMD+Controller to get alignment readings
						SetCalibrationStatus(EPSMoveCalibrationStatusEnum::HMDSampling);
					}
				}
			} break;

			case EPSMoveCalibrationStatusEnum::WaitForStationary:
			{
				if (IsControllerStableAndVisible(controllerData))
				{
					if (StateTimer > WAIT_FOR_STABLE_DURATION)
					{
						UE_LOG(LogPSMove, Log, TEXT("FPSMoveCalibration: ControllerUpdate: Controller is stable. Starting noise sampling."));

						// We know the controller should be point upright and stable at this point
						psmove_reset_orientation(psmove);

						// Start Sampling
						SetCalibrationStatus(EPSMoveCalibrationStatusEnum::StationarySampling);
					}
				}
				else
				{
					UE_LOG(LogPSMove, Log, TEXT("FPSMoveCalibration: ControllerUpdate: Controller moved. Restarting stationary wait timer."));

					// Doh! We got moved/occluded. Reset the state timer.
					StateTimer = 0.f;
				}
			}
			break;

			case EPSMoveCalibrationStatusEnum::StationarySampling:
			{
				if (IsControllerStableAndVisible(controllerData))
				{
					// Sample the controllers position and acceleration over many ticks
					// to compute the noise variances of those states.
					RecordNoiseSample(controllerData);

					// Stop once we got all of the samples we needed
					if (GetAccelerationNoiseSampleVectors().Num() >= NOISE_SAMPLES_NEEDED && 
						GetPositionNoiseSamplePoints().Num() >= NOISE_SAMPLES_NEEDED)
					{
						PointCloudCovarianceMatrixBuilder accelerationCovarianceBuilder;
						PointCloudCovarianceMatrixBuilder positionCovarianceBuilder;

						UE_LOG(LogPSMove, Log, TEXT("FPSMoveCalibration: ControllerUpdate: Noise sampling complete."));

						// Compute the 3x3 covariance matrix for the position and acceleration noise.
						// This assumes that the controller was stationary during sampling and thus any
						// variation in readings is due to noise in measurement and not some other source.
						accelerationCovarianceBuilder.ComputeCovarianceMatrix(GetAccelerationNoiseSampleVectors());
						positionCovarianceBuilder.ComputeCovarianceMatrix(GetPositionNoiseSamplePoints());

						// We know that the controller should be sitting level upright
						// so it should be reading on average <0g, +1g, 0g>, 
						// But for some reason the Y-axis accelerometer tends to read 
						// around 0.91g instead and if you turn the controller upside-down 
						// you read around -1.09g, thus the Y axis values are skewed by about -0.09g
						// We compute a correction skew here.
						// The X and Z axis tend to be pretty close to spot on.
						{
							const FVector AverageWorldSpaceAcceleration = accelerationCovarianceBuilder.GetMean();

							// NOTE: The accelerationCovarianceBuilder has acceleration samples in world space
							// (along the z-axis) measured in mm/s^2. 
							// We convert this back to units of 'g'.
							const float averageGravity = AverageWorldSpaceAcceleration.Z / GetGravityUnits();

							// Store the skew in PSMove Space
							this->PSMoveSpaceAccelerationSkew.X = 0.0;
							this->PSMoveSpaceAccelerationSkew.Y = 1.f - averageGravity;
							this->PSMoveSpaceAccelerationSkew.Z = 0.0;
						}

						// We only want a single variance value for the acceleration since the 
						// accelerometer /should/ have the same amount of noise across all 3 axes.
						{
							FMatrix accCovMatrix = accelerationCovarianceBuilder.GetCovarianvceMatrix();

							this->AccelerationVariance =
								FMath::Max3(accCovMatrix.M[0][0], accCovMatrix.M[1][1], accCovMatrix.M[2][2]);
						}

						// The positional noise covariance is used in the correction phase of the Kalman filter.
						// We keep this as a matrix because the position variance is different on each axis.
						// the X-axis (toward the camera) since that is based off the size of the tracking sphere
						// while the Y ans Z axes (perpendicular to the camera) are based off tracking sphere centroid.
						{
							FMatrix posCovMatrix= positionCovarianceBuilder.GetCovarianvceMatrix();

							this->MeasurementNoiseMatrix.SetIdentity();
							this->MeasurementNoiseMatrix[0][0] = posCovMatrix.M[0][0];
							this->MeasurementNoiseMatrix[1][1] = posCovMatrix.M[1][1];
							this->MeasurementNoiseMatrix[2][2] = posCovMatrix.M[2][2];
						}

						// Notify the user component that Stabilization succeeded
						SharedData->CalibrationPointResultEnum = EPSMoveCalibrationStepResultEnum::Valid;

						// Move on to sampling the HMD+Controller to get alignment readings
						SetCalibrationStatus(EPSMoveCalibrationStatusEnum::HMDSampling);
					}
				}
				else
				{
					UE_LOG(LogPSMove, Log, TEXT("FPSMoveCalibration: ControllerUpdate: Controller moved. Canceling sampling."));

					// Notify the user component that sampling was interrupted
					SharedData->CalibrationPointResultEnum = EPSMoveCalibrationStepResultEnum::Invalid;

					// Doh! We got moved/occluded. Wait for us to stabilize again.
					SetCalibrationStatus(EPSMoveCalibrationStatusEnum::WaitForStationary);
				}
			}
			break;

			case EPSMoveCalibrationStatusEnum::HMDSampling:
			{
				if (calibrationActionRequested && StateTimer > 0.5f)
				{
					UE_LOG(LogPSMove, Log, TEXT("FPSMoveCalibration: ControllerUpdate: Calibration button pressed. Recording alignment point."));

					RecordAlignmentSample(controllerData);

					// Reset the state timer so that we wait 1/2 second before accepting the next input
					StateTimer = 0.f;

					if (GetPSMoveSamplePoints().Num() >= POINTS_NEEDED_FOR_CALIBRATION)
					{
						AffineTransformBuilder transformBuilder;

						UE_LOG(LogPSMove, Log, TEXT("FPSMoveCalibration: ControllerUpdate: Alignment sampling complete."));

						// - Compute the best fit affine transform that turns sample points in PSMove space to points in HMD space
						// - Compute the average of the quaternion corrections
						if (transformBuilder.ComputeAffineLeastSquareFit(GetPSMoveSamplePoints(), GetHMDSamplePoints()))
						{
							PSMoveSpaceToHMDSpace = transformBuilder.GetAffineTransform();

							UE_LOG(LogPSMove, Log, TEXT("FPSMoveCalibration: ControllerUpdate: Alignment transform successfully computed."));

							SetCalibrationStatus(EPSMoveCalibrationStatusEnum::Succeeded);

							// Save the calibration file back to disk
							SaveCalibrationFile();
						}
						else
						{
							UE_LOG(LogPSMove, Log, TEXT("FPSMoveCalibration: ControllerUpdate: Failed to compute alignment transform. Bad sample points?"));
							SetCalibrationStatus(EPSMoveCalibrationStatusEnum::Failed);
						}
					}
				}
			}
			break;

			case EPSMoveCalibrationStatusEnum::Succeeded:
			case EPSMoveCalibrationStatusEnum::Failed:
			{
				UE_LOG(LogPSMove, Log, TEXT("FPSMoveCalibration: ControllerUpdate: Calibration done. Returning to inactive."));

				// Return to the inactive state
				SetCalibrationStatus(EPSMoveCalibrationStatusEnum::Inactive);
			} break;
		}

		// Remember this frames calibration action request
		this->PreviousFrameCalibrationActionRequest = controllerData->CalibrationActionRequest;
	}
}

void FPSMoveCalibration::SetCalibrationStatus(
	EPSMoveCalibrationStatusEnum::Type newStatus)
{
	if (SharedData->CalibrationStatusEnum != newStatus)
	{
		// On Status Entry
		switch (newStatus)
		{
			case EPSMoveCalibrationStatusEnum::Inactive:
			{
				UE_LOG(LogPSMove, Log, TEXT("FPSMoveCalibration: SetCalibrationStatus: -> Inactive"));
			} break;

			case EPSMoveCalibrationStatusEnum::WaitForStationary:
			{
				UE_LOG(LogPSMove, Log, TEXT("FPSMoveCalibration: SetCalibrationStatus: -> WaitForStationary"));

				// (Re)Start the noise sampling process
				ResetNoiseSampleData();
			} break;

			case EPSMoveCalibrationStatusEnum::StationarySampling:
			{
				UE_LOG(LogPSMove, Log, TEXT("FPSMoveCalibration: SetCalibrationStatus: -> StationarySampling"));

				// (Re)Start the noise sampling process
				ResetNoiseSampleData();
			}
			break;

			case EPSMoveCalibrationStatusEnum::HMDSampling:
			{
				UE_LOG(LogPSMove, Log, TEXT("FPSMoveCalibration: SetCalibrationStatus: -> HMDSampling"));

				ResetAlignmentSampleData();
			}
			break;

			case EPSMoveCalibrationStatusEnum::Succeeded:
			{
				UE_LOG(LogPSMove, Log, TEXT("FPSMoveCalibration: SetCalibrationStatus: -> Succeeded"));

				// Forget the controller we were calibrating with
				CalibrationControllerIndex = -1;
			} break;

			case EPSMoveCalibrationStatusEnum::Failed:
			{
				UE_LOG(LogPSMove, Log, TEXT("FPSMoveCalibration: SetCalibrationStatus: -> Failed"));

				// Forget the controller we were calibrating with
				CalibrationControllerIndex = -1;
			} break;
		}

		// Reset the state timer
		StateTimer = 0.f;

		// Update the calibration status
		SharedData->CalibrationStatusEnum = newStatus;
		SharedData->MarkDirty();
	}
}


void FPSMoveCalibration::OnControllerCountChanged()
{
	if (SharedData->CalibrationStatusEnum != EPSMoveCalibrationStatusEnum::Inactive ||
		SharedData->CalibrationStatusEnum != EPSMoveCalibrationStatusEnum::Succeeded)
	{
		// Forget everything up to this point
		ResetCalibrationSampleData();

		// Notify the user component that calibration was interrupted
		SharedData->CalibrationPointResultEnum = EPSMoveCalibrationStepResultEnum::Invalid;
		
		// Back to square one
		SetCalibrationStatus(EPSMoveCalibrationStatusEnum::WaitForStationary);
	}
}

void FPSMoveCalibration::RecordNoiseSample(
	const FPSMoveRawControllerData_Base *controllerData)
{
	const FVector &RawPSMoveAcceleration = controllerData->PSMoveRelativeRawAcceleration;
	const FVector &RawWorldPosition = controllerData->WorldRelativeRawPosition;
	const FQuat &orientation = controllerData->WorldRelativeFilteredOrientation;

	// Convert the raw acceleration from PSMove Space into Unreal World Space
	FVector WorldRelativeRawAcceleration =
		orientation.GetAxisX() * RawPSMoveAcceleration.Y + // +x in Unreal is +x in PSMoveLib  
		orientation.GetAxisY() * RawPSMoveAcceleration.X + // +y in Unreal is +y in PSMoveLib 
		orientation.GetAxisZ() * RawPSMoveAcceleration.Z;  // +z in Unreal is +z in PSMoveLib

	// The acceleration is currently in units of 'g'.
	// World space units are millimeters.
	WorldRelativeRawAcceleration*= GetGravityUnits();

	GetAccelerationNoiseSampleVectors().Add(WorldRelativeRawAcceleration);
	GetPositionNoiseSamplePoints().Add(RawWorldPosition);
}

void FPSMoveCalibration::RecordAlignmentSample(
	const FPSMoveRawControllerData_Base *controllerData)
{
	const bool canSeePSMove = controllerData->CanSee;

	if (SharedData->HMDIsTracked && canSeePSMove)
	{
		const FVector PSMovePosition = controllerData->PSMoveRelativeRawPosition;
		const FVector HMDPosition = SharedData->HMDTrackingSpacePosition;

		GetPSMoveSamplePoints().Add(PSMovePosition);
		GetHMDSamplePoints().Add(HMDPosition);

		SharedData->CalibrationPointResultEnum = EPSMoveCalibrationStepResultEnum::Valid;
	}
	else
	{
		SharedData->CalibrationPointResultEnum = EPSMoveCalibrationStepResultEnum::Invalid;
	}

	SharedData->MarkDirty();
}

inline void ConfigFileSetFloat(
	const TCHAR* Section,
	const TCHAR* Key,
	const float Value,
	FConfigFile *ConfigFile)
{
	TCHAR Text[MAX_SPRINTF] = TEXT("");
	FCString::Sprintf(Text, TEXT("%f"), Value);

	ConfigFile->SetString(Section, Key, Text);
}

inline void ConfigFileGetFloat(
	const FConfigFile *ConfigFile,
	const TCHAR* Section,
	const TCHAR* Key,
	const float DefaultValue,
	float *OutValue)
{
	FString StringValue;

	if (ConfigFile->GetString(Section, Key, StringValue))
	{
		*OutValue = FCString::Atof(*StringValue);
	}
	else
	{
		*OutValue = DefaultValue;
	}
}

void FPSMoveCalibration::LoadCalibrationFile()
{
	FConfigFile ConfigFile;
	FString ConfigPath = FString(FPaths::GameConfigDir()) / FString(CONFIG_FILE);

	UE_LOG(LogPSMove, Log, TEXT("FPSMoveCalibration: LoadCalibrationFile: Reading Calibration file"));

	ConfigFile.Read(ConfigPath);

	if (ConfigFile.Contains(PSMOVESPACETOHMDSPACE_SECTION))
	{
		ConfigFileGetFloat(&ConfigFile, PSMOVESPACETOHMDSPACE_SECTION, TEXT("m00"), 1.0f, &PSMoveSpaceToHMDSpace.M[0][0]);
		ConfigFileGetFloat(&ConfigFile, PSMOVESPACETOHMDSPACE_SECTION, TEXT("m01"), 0.0f, &PSMoveSpaceToHMDSpace.M[0][1]);
		ConfigFileGetFloat(&ConfigFile, PSMOVESPACETOHMDSPACE_SECTION, TEXT("m02"), 0.0f, &PSMoveSpaceToHMDSpace.M[0][2]);

		ConfigFileGetFloat(&ConfigFile, PSMOVESPACETOHMDSPACE_SECTION, TEXT("m10"), 0.0f, &PSMoveSpaceToHMDSpace.M[1][0]);
		ConfigFileGetFloat(&ConfigFile, PSMOVESPACETOHMDSPACE_SECTION, TEXT("m11"), 1.0f, &PSMoveSpaceToHMDSpace.M[1][1]);
		ConfigFileGetFloat(&ConfigFile, PSMOVESPACETOHMDSPACE_SECTION, TEXT("m12"), 0.0f, &PSMoveSpaceToHMDSpace.M[1][2]);

		ConfigFileGetFloat(&ConfigFile, PSMOVESPACETOHMDSPACE_SECTION, TEXT("m20"), 0.0f, &PSMoveSpaceToHMDSpace.M[2][0]);
		ConfigFileGetFloat(&ConfigFile, PSMOVESPACETOHMDSPACE_SECTION, TEXT("m21"), 0.0f, &PSMoveSpaceToHMDSpace.M[2][1]);
		ConfigFileGetFloat(&ConfigFile, PSMOVESPACETOHMDSPACE_SECTION, TEXT("m22"), 1.0f, &PSMoveSpaceToHMDSpace.M[2][2]);

		ConfigFileGetFloat(&ConfigFile, PSMOVESPACETOHMDSPACE_SECTION, TEXT("m30"), 0.0f, &PSMoveSpaceToHMDSpace.M[3][0]);
		ConfigFileGetFloat(&ConfigFile, PSMOVESPACETOHMDSPACE_SECTION, TEXT("m31"), 0.0f, &PSMoveSpaceToHMDSpace.M[3][1]);
		ConfigFileGetFloat(&ConfigFile, PSMOVESPACETOHMDSPACE_SECTION, TEXT("m32"), 0.0f, &PSMoveSpaceToHMDSpace.M[3][2]);
	}
	else
	{
		UE_LOG(LogPSMove, Log, TEXT("FPSMoveCalibration: LoadCalibrationFile: No PSMoveSpaceToHMDSpace section. Using defaults."));
	}

	if (ConfigFile.Contains(NOISE_SECTION))
	{
		ConfigFileGetFloat(&ConfigFile, NOISE_SECTION, TEXT("accelerationVariance"), 0.0f, &AccelerationVariance);

		ConfigFileGetFloat(&ConfigFile, NOISE_SECTION, TEXT("accelerationXSkew"), 0.0f, &PSMoveSpaceAccelerationSkew.X);
		ConfigFileGetFloat(&ConfigFile, NOISE_SECTION, TEXT("accelerationYSkew"), 0.09f, &PSMoveSpaceAccelerationSkew.Y);
		ConfigFileGetFloat(&ConfigFile, NOISE_SECTION, TEXT("accelerationZSkew"), 0.0f, &PSMoveSpaceAccelerationSkew.Z);		

		ConfigFileGetFloat(&ConfigFile, NOISE_SECTION, TEXT("variance_P00"), 0.0f, &MeasurementNoiseMatrix[0][0]);
		ConfigFileGetFloat(&ConfigFile, NOISE_SECTION, TEXT("variance_P11"), 0.0f, &MeasurementNoiseMatrix[1][1]);
		ConfigFileGetFloat(&ConfigFile, NOISE_SECTION, TEXT("variance_P22"), 0.0f, &MeasurementNoiseMatrix[2][2]);
	}
	else
	{
		UE_LOG(LogPSMove, Log, TEXT("FPSMoveCalibration: LoadCalibrationFile: No Noise section. Using defaults."));
	}
}

void FPSMoveCalibration::SaveCalibrationFile()
{
	FConfigFile ConfigFile;
	FString ConfigPath = FString(FPaths::GameConfigDir()) / FString(CONFIG_FILE);

	UE_LOG(LogPSMove, Log, TEXT("FPSMoveCalibration: SaveCalibrationFile: Saving Calibration file: %s"), *ConfigPath);

	ConfigFile.Read(ConfigPath);

	// Save the psmove space transform
	ConfigFileSetFloat(PSMOVESPACETOHMDSPACE_SECTION, TEXT("m00"), PSMoveSpaceToHMDSpace.M[0][0], &ConfigFile);
	ConfigFileSetFloat(PSMOVESPACETOHMDSPACE_SECTION, TEXT("m01"), PSMoveSpaceToHMDSpace.M[0][1], &ConfigFile);
	ConfigFileSetFloat(PSMOVESPACETOHMDSPACE_SECTION, TEXT("m02"), PSMoveSpaceToHMDSpace.M[0][2], &ConfigFile);

	ConfigFileSetFloat(PSMOVESPACETOHMDSPACE_SECTION, TEXT("m10"), PSMoveSpaceToHMDSpace.M[1][0], &ConfigFile);
	ConfigFileSetFloat(PSMOVESPACETOHMDSPACE_SECTION, TEXT("m11"), PSMoveSpaceToHMDSpace.M[1][1], &ConfigFile);
	ConfigFileSetFloat(PSMOVESPACETOHMDSPACE_SECTION, TEXT("m12"), PSMoveSpaceToHMDSpace.M[1][2], &ConfigFile);

	ConfigFileSetFloat(PSMOVESPACETOHMDSPACE_SECTION, TEXT("m20"), PSMoveSpaceToHMDSpace.M[2][0], &ConfigFile);
	ConfigFileSetFloat(PSMOVESPACETOHMDSPACE_SECTION, TEXT("m21"), PSMoveSpaceToHMDSpace.M[2][1], &ConfigFile);
	ConfigFileSetFloat(PSMOVESPACETOHMDSPACE_SECTION, TEXT("m22"), PSMoveSpaceToHMDSpace.M[2][2], &ConfigFile);

	ConfigFileSetFloat(PSMOVESPACETOHMDSPACE_SECTION, TEXT("m30"), PSMoveSpaceToHMDSpace.M[3][0], &ConfigFile);
	ConfigFileSetFloat(PSMOVESPACETOHMDSPACE_SECTION, TEXT("m31"), PSMoveSpaceToHMDSpace.M[3][1], &ConfigFile);
	ConfigFileSetFloat(PSMOVESPACETOHMDSPACE_SECTION, TEXT("m32"), PSMoveSpaceToHMDSpace.M[3][2], &ConfigFile);

	// Save the noise variance info
	ConfigFileSetFloat(NOISE_SECTION, TEXT("accelerationVariance"), AccelerationVariance, &ConfigFile);

	ConfigFileSetFloat(NOISE_SECTION, TEXT("accelerationXSkew"), PSMoveSpaceAccelerationSkew.X, &ConfigFile);
	ConfigFileSetFloat(NOISE_SECTION, TEXT("accelerationYSkew"), PSMoveSpaceAccelerationSkew.Y, &ConfigFile);
	ConfigFileSetFloat(NOISE_SECTION, TEXT("accelerationZSkew"), PSMoveSpaceAccelerationSkew.Z, &ConfigFile);

	ConfigFileSetFloat(NOISE_SECTION, TEXT("variance_P00"), MeasurementNoiseMatrix[0][0], &ConfigFile);
	ConfigFileSetFloat(NOISE_SECTION, TEXT("variance_P11"), MeasurementNoiseMatrix[1][1], &ConfigFile);
	ConfigFileSetFloat(NOISE_SECTION, TEXT("variance_P22"), MeasurementNoiseMatrix[2][2], &ConfigFile);

	if (!ConfigFile.Write(ConfigPath))
	{
		UE_LOG(LogPSMove, Log, TEXT("FPSMoveCalibration: SaveCalibrationFile: Failed to save calibration config. Read-Only?"));
	}
}

bool FPSMoveCalibration::IsNoiseUncalibrated() const
{
	return 
		AccelerationVariance == 0.f ||
		MeasurementNoiseMatrix[0][0] == 0.f || 
		MeasurementNoiseMatrix[1][1] == 0.f ||
		MeasurementNoiseMatrix[2][2] == 0.f;
}

// -- Affine Transform Builder ------
AffineTransformBuilder::AffineTransformBuilder()
{
	AffineTransform.SetIdentity();
}

bool AffineTransformBuilder::ComputeAffineLeastSquareFit(
	const TArray<FVector> &from_pts, 
	const TArray<FVector> &to_pts)
{
	bool success = true;

	/*Fit an affine transformation to given point sets.
	More precisely: 
	
	solve (least squares fit) matrix 'A'and 't' from
	p ~= A*q+t, given vectors 'p' and 'q'.

	Goal is to minimized S(A,t), where S(A,t) = Sum[1->N] (p_i - A*q_i - t)^2

	Works with arbitrary dimensional vectors (2d, 3d, 4d...).

	Adapted from the Python code here http://elonen.iki.fi/code/misc-notes/affine-fit/
	which was written by Jarno Elonen <elonen@iki.fi> in 2007 and placed in the Public Domain.

	Based on paper "Fitting affine and orthogonal transformations
	between two sets of points, by Helmuth Spath (2003).*/

	if ((from_pts.Num() != to_pts.Num()) || (from_pts.Num() < 1))
	{
		// from_pts and to_pts must be of same size
		success= false;
	}

	if (success && from_pts.Num() < DIMENSION_COUNT)
	{
		// Too few points => under-determined system.
		success = false;
	}

	if (success)
	{
		FMatrixMxN q(from_pts.Num(), DIMENSION_COUNT + 1);
		FMatrixMxN p(to_pts.Num(), DIMENSION_COUNT + 1);
		FMatrixMxN c(DIMENSION_COUNT + 1, DIMENSION_COUNT);
		FMatrixMxN Q(DIMENSION_COUNT + 1, DIMENSION_COUNT + 1);
		FMatrixMxN a(DIMENSION_COUNT + 1, DIMENSION_COUNT + 1 + DIMENSION_COUNT);

		// copy 'from' array and append a set of 1s
		for (int32 i = 0; i < q.GetRowCount(); i++)
		{
			q[i][0] = from_pts[i].X;
			q[i][1] = from_pts[i].Y;
			q[i][2] = from_pts[i].Z;
			q[i][3] = 1.0f;
		}

		// copy 'to' array
		for (int32 i = 0; i < p.GetRowCount(); i++)
		{
			p[i][0] = to_pts[i].X;
			p[i][1] = to_pts[i].Y;
			p[i][2] = to_pts[i].Z;
			p[i][3] = 1.0f;
		}

		// c_j,k = Sum[1,N] (q_k,i * p_j,i)
		// (DIMENSION_COUNT) x (DIMENSION_COUNT+1)
		for (int32 c_colomn = 0; c_colomn < c.GetColomnCount(); c_colomn++)
		{
			for (int c_row = 0; c_row < c.GetRowCount(); c_row++)
			{
				for (int q_row = 0; q_row < q.GetRowCount(); q_row++)
				{
					c[c_row][c_colomn] += q[q_row][c_row] * p[q_row][c_colomn];
				}
			}
		}

		// Q = Sum[1, N] (q_i * Transpose(q_i))
		// (DIMENSION_COUNT+1) x (DIMENSION_COUNT+1)
		for (int32 q_row = 0; q_row < q.GetRowCount(); q_row++)
		{
			for (int Q_row = 0; Q_row < Q.GetRowCount(); Q_row++)
			{
				for (int Q_colomn = 0; Q_colomn < Q.GetColomnCount(); Q_colomn++)
				{
					Q[Q_row][Q_colomn] += q[q_row][Q_row] * q[q_row][Q_colomn];
				}
			}
		}

		// Augment Q with c into matrix a, i.e. a= [Q|c]
		for (int32 m_row = 0; m_row < a.GetRowCount(); m_row++)
		{
			for (int32 Q_colomn = 0; Q_colomn < Q.GetColomnCount(); Q_colomn++)
			{
				a[m_row][Q_colomn] = Q[m_row][Q_colomn];
			}

			for (int32 c_colomn = 0; c_colomn < c.GetColomnCount(); c_colomn++)
			{
				a[m_row][Q.GetColomnCount() + c_colomn]= c[m_row][c_colomn];
			}
		}

		// solve Q * a' = c by Gauss-Jordan
		// Error = singular matrix. Points are probably coplanar
		success = a.ApplyGaussJordanSolver();

		// Extract the affine transform back out of M
		if (success)
		{
			const int colomn_offset = DIMENSION_COUNT + 1;

			FVector row0(a[0][colomn_offset + 0], a[0][colomn_offset + 1], a[0][colomn_offset + 2]);
			FVector row1(a[1][colomn_offset + 0], a[1][colomn_offset + 1], a[1][colomn_offset + 2]);
			FVector row2(a[2][colomn_offset + 0], a[2][colomn_offset + 1], a[2][colomn_offset + 2]);
			FVector row3(a[3][colomn_offset + 0], a[3][colomn_offset + 1], a[3][colomn_offset + 2]);

			AffineTransform.SetAxes(&row0, &row1, &row2, &row3);
		}
		else
		{
			UE_LOG(LogPSMove, Log, TEXT("Failed to compute affine transform. Co-planar points?"));
		}
	}

	return success;
}

void AffineTransformBuilder::LogTransform()
{
	UE_LOG(LogPSMove, Log, TEXT("x' = x*%f + y*%f + z*%f + %f"), 
		AffineTransform.M[0][0], AffineTransform.M[1][0], AffineTransform.M[2][0], AffineTransform.M[3][0]);
	UE_LOG(LogPSMove, Log, TEXT("y' = x*%f + y*%f + z*%f + %f"),
		AffineTransform.M[0][1], AffineTransform.M[1][1], AffineTransform.M[2][1], AffineTransform.M[3][1]);
	UE_LOG(LogPSMove, Log, TEXT("z' = x*%f + y*%f + z*%f + %f"),
		AffineTransform.M[0][2], AffineTransform.M[1][2], AffineTransform.M[2][2], AffineTransform.M[3][2]);
}

void AffineTransformBuilder::Test()
{
	/* Should produce the output
	*
	Transformation is:

	x' = x * 2.000000 + y * 2.000000 + z * 0.000000 + 0.000000
	y' = x * -2.000000 + y * 2.000000 + z * 0.000000 + 4.000000
	z' = x * 0.000000 + y * 0.000000 + z * 2.000000 + 0.000000

	(1,1,1) => (4.000000,4.000000,2.000000) ~= (4.000000,4.000000,2.000000)
	(1,2,2) => (6.000000,6.000000,4.000000) ~= (6.000000,6.000000,2.000000)
	(2,2,3) => (8.000000,4.000000,6.000000) ~= (8.000000,4.000000,2.000000)
	(2,1,4) => (6.000000,2.000000,8.000000) ~= (6.000000,2.000000,2.000000)

	*/

	AffineTransformBuilder builder;
	TArray<FVector> fromPoints;
	TArray<FVector> toPoints;

	fromPoints.Add(FVector(1, 1, 1));
	fromPoints.Add(FVector(1, 2, 2));
	fromPoints.Add(FVector(2, 2, 3));
	fromPoints.Add(FVector(2, 1, 4));

	toPoints.Add(FVector(4, 4, 2));
	toPoints.Add(FVector(6, 6, 4));
	toPoints.Add(FVector(8, 4, 6));
	toPoints.Add(FVector(6, 2, 8));

	builder.ComputeAffineLeastSquareFit(fromPoints, toPoints);

	UE_LOG(LogPSMove, Log, TEXT("Transformation is:"));
	builder.LogTransform();

	float fitting_error = 0.f;

	for (int i = 0; i < fromPoints.Num(); i++)
	{ 
		FVector fromPoint = fromPoints[i];
		FVector transformedPoint = builder.GetAffineTransform().TransformPosition(fromPoint);
		FVector toPoint = toPoints[i];

		fitting_error += FVector::Dist(transformedPoint, toPoint);
		UE_LOG(LogPSMove, Log, TEXT("(%f,%f,%f) => (%f,%f,%f) ~= (%f,%f,%f)"),
			fromPoint.X, fromPoint.Y, fromPoint.Z,
			transformedPoint.X, transformedPoint.Y, transformedPoint.Z,
			toPoint.X, toPoint.Y, toPoint.Z);
	}
	UE_LOG(LogPSMove, Log, TEXT("Fitting Error = %f"), fitting_error);
	assert(fitting_error < 0.0001f);
}

// -- Average Quaternion Builder --
AverageQuaternionBuilder::AverageQuaternionBuilder() :
	AverageQuaternion(FQuat::Identity)
{

}

bool AverageQuaternionBuilder::ComputeAverageQuaternionFast(
	const TArray<FQuat> &samples)
{
	bool success = false;

	if (samples.Num() > 0)
	{
		const FQuat firstSample = samples[0];
		const float divisor = (float)samples.Num();

		AverageQuaternion = FQuat(ForceInitToZero);

		for (int32 index = 0; index < samples.Num(); ++index)
		{
			FQuat sample = samples[index];

			if (FQuat::Error(firstSample, sample) < 0.f)
			{
				sample = sample.Inverse();
			}

			AverageQuaternion += sample;
		}

		AverageQuaternion = AverageQuaternion / divisor;
		AverageQuaternion.Normalize();

		success = true;
	}

	return success;
}

bool AverageQuaternionBuilder::ComputeAverageQuaternionAccurate(
	const TArray<FQuat> &samples)
{
	bool success = false;

	if (samples.Num() > 0)
	{
		FMatrixMxN q(4, samples.Num());
		FMatrixMxN q_transpose(samples.Num(), 4);

		for (int32 index = 0; index < samples.Num(); ++index)
		{
			const FQuat &sample = samples[index];

			q[0][index] = sample.X;
			q[1][index] = sample.Y;
			q[2][index] = sample.Z;
			q[3][index] = sample.W;

			q_transpose[index][0] = sample.X;
			q_transpose[index][1] = sample.Y;
			q_transpose[index][2] = sample.Z;
			q_transpose[index][3] = sample.W;
		}

		FMatrixMxN M(4, 4);
		MatrixMultiply(q, q_transpose, M);

		TArray<float> eigen_values;
		FMatrixMxN eigen_vectors(4, 4);
		
		if (M.ApplyJacobiEigenSolver(eigen_values, eigen_vectors))
		{
			int32 largest_colomn = 0;
			float largest_eigenvalue = eigen_values[0];
			for (int32 col = 1; col < 4; ++col)
			{
				if (eigen_values[col] > largest_eigenvalue)
				{
					largest_eigenvalue = eigen_values[col];
					largest_colomn = col;
				}
			}

			AverageQuaternion.X = eigen_vectors[0][largest_colomn];
			AverageQuaternion.Y = eigen_vectors[1][largest_colomn];
			AverageQuaternion.Z = eigen_vectors[2][largest_colomn];
			AverageQuaternion.W = eigen_vectors[3][largest_colomn];

			success = true;
		}
	}

	return success;
}

void AverageQuaternionBuilder::LogQuaternion()
{
	UE_LOG(LogPSMove, Log, TEXT("x:%f, y:%f, z:%f, w:%f"),
		AverageQuaternion.X, AverageQuaternion.Y, AverageQuaternion.Z, AverageQuaternion.W);
}

void AverageQuaternionBuilder::Test()
{
	AverageQuaternionBuilder fastBuilder;
	AverageQuaternionBuilder accurateBuilder;
	TArray<FQuat> samples;

	const float k_45degrees = (float)PI / 4.0f;
	const float k_30degrees = (float)PI / 6.0f;

	samples.Add(FQuat(FVector(0, 1, 0), k_45degrees));
	samples.Add(FQuat(FVector(0, 1, 0), -k_45degrees));
	samples.Add(FQuat(FVector(0, 0, 1), k_45degrees));
	samples.Add(FQuat(FVector(0, 0, 1), -k_45degrees));
	samples.Add(FQuat(FVector(0, 1, 0), k_30degrees));
	samples.Add(FQuat(FVector(0, 1, 0), -k_30degrees));
	samples.Add(FQuat(FVector(0, 0, 1), k_30degrees));
	samples.Add(FQuat(FVector(0, 0, 1), -k_30degrees));

	fastBuilder.ComputeAverageQuaternionFast(samples);
	accurateBuilder.ComputeAverageQuaternionAccurate(samples);

	UE_LOG(LogPSMove, Log, TEXT("Fast Quaternion is:"));
	fastBuilder.LogQuaternion();

	const float fastError = FQuat::Error(fastBuilder.GetAvageQuaternion(), FQuat::Identity);
	UE_LOG(LogPSMove, Log, TEXT("  Fitting Error = %f"), fastError);
	assert(fastError < 0.001f);

	UE_LOG(LogPSMove, Log, TEXT("Accurate Quaternion is:"));
	accurateBuilder.LogQuaternion();

	const float accurateError = FQuat::Error(accurateBuilder.GetAvageQuaternion(), FQuat::Identity);
	UE_LOG(LogPSMove, Log, TEXT("  Fitting Error = %f"), accurateError);
	assert(accurateError < 0.001f);
	assert(accurateError < fastError);
}

// -- Point CLoud Covariance Matrix Builder --
PointCloudCovarianceMatrixBuilder::PointCloudCovarianceMatrixBuilder() :
	Mean(FVector::ZeroVector),
	CovarianceMatrix(ForceInitToZero)
{
}

bool PointCloudCovarianceMatrixBuilder::ComputeCovarianceMatrix(
	const TArray<FVector> &samples)
{
	return ComputeCovarianceMatrix(samples.GetData(), samples.Num());
}

bool PointCloudCovarianceMatrixBuilder::ComputeCovarianceMatrix(
	const FVector *samples, 
	const int32 sampleCount)
{
	bool success = false;

	Mean = FVector::ZeroVector;
	CovarianceMatrix = FMatrix(ForceInitToZero);

	if (sampleCount > 1)
	{
		const float N = (float)sampleCount;
		const float N_minus_one = N - 1.f;

		for (int32 index = 0; index < sampleCount; ++index)
		{
			Mean += samples[index];
		}
		Mean /= N;

		for (int32 index = 0; index < sampleCount; ++index)
		{
			const float &x = samples[index].X - Mean.X;
			const float &y = samples[index].Y - Mean.Y;
			const float &z = samples[index].Z - Mean.Z;

			CovarianceMatrix.M[0][0] += x*x; CovarianceMatrix.M[0][1] += y*x; CovarianceMatrix.M[0][2] += z*x;
			CovarianceMatrix.M[1][0] += x*y; CovarianceMatrix.M[1][1] += y*y; CovarianceMatrix.M[1][2] += z*y;
			CovarianceMatrix.M[2][0] += z*x; CovarianceMatrix.M[2][1] += z*z; CovarianceMatrix.M[2][2] += z*z;
		}

		CovarianceMatrix.M[0][0] /= N_minus_one; CovarianceMatrix.M[0][1] /= N_minus_one; CovarianceMatrix.M[0][2] /= N_minus_one;
		CovarianceMatrix.M[1][0] /= N_minus_one; CovarianceMatrix.M[1][1] /= N_minus_one; CovarianceMatrix.M[1][2] /= N_minus_one;
		CovarianceMatrix.M[2][0] /= N_minus_one; CovarianceMatrix.M[2][1] /= N_minus_one; CovarianceMatrix.M[2][2] /= N_minus_one;

		success = true;
	}

	return success;
}

void PointCloudCovarianceMatrixBuilder::LogMean()
{
	UE_LOG(LogPSMove, Log, TEXT("<%.4f %.4f %.4f>"), Mean.X, Mean.Y, Mean.Z);
}

void PointCloudCovarianceMatrixBuilder::LogCovarianveMatrix()
{
	UE_LOG(LogPSMove, Log, TEXT("|%.4f %.4f %.4f|"),
		CovarianceMatrix.M[0][0], CovarianceMatrix.M[1][0], CovarianceMatrix.M[2][0]);
	UE_LOG(LogPSMove, Log, TEXT("|%.4f %.4f %.4f|"),
		CovarianceMatrix.M[0][1], CovarianceMatrix.M[1][1], CovarianceMatrix.M[2][1]);
	UE_LOG(LogPSMove, Log, TEXT("|%.4f %.4f %.4f|"),
		CovarianceMatrix.M[0][2], CovarianceMatrix.M[1][2], CovarianceMatrix.M[2][2]);
}

void PointCloudCovarianceMatrixBuilder::Test()
{
	PointCloudCovarianceMatrixBuilder builder;
	TArray<FVector> samples;

	const int32 sampleCount = 100;
	const FVector expectedMean(5.f, 5.f, 5.f);
	const float expectedXSigma = 1.f;
	const float expectedYSigma = 2.f;
	const float expectedZSigma = 3.f;

	std::default_random_engine generator;
	std::normal_distribution<float> x_distribution(expectedMean.X, expectedXSigma);
	std::normal_distribution<float> y_distribution(expectedMean.Y, expectedYSigma);
	std::normal_distribution<float> z_distribution(expectedMean.Z, expectedZSigma);

	for (int32 index = 0; index < sampleCount; ++index)
	{
		float x = x_distribution(generator);
		float y = y_distribution(generator);
		float z = z_distribution(generator);

		samples.Add(FVector(x, y, z));
	}

	builder.ComputeCovarianceMatrix(samples);

	UE_LOG(LogPSMove, Log, TEXT("Mean is:"));
	builder.LogMean();

	const FVector errorVector = builder.GetMean() - expectedMean;
	const float meanError = FMath::Sqrt(FVector::DotProduct(errorVector, errorVector));
	UE_LOG(LogPSMove, Log, TEXT("  Mean Error = %f"), meanError);
	assert(meanError < 0.5f);

	UE_LOG(LogPSMove, Log, TEXT("Covariance Matrix is:"));
	builder.LogCovarianveMatrix();

	FMatrix covMatrix= builder.GetCovarianvceMatrix();
	const float x_sigma_error = FMath::Abs(FMath::Sqrt(covMatrix.M[0][0]) - expectedXSigma);
	const float y_sigma_error = FMath::Abs(FMath::Sqrt(covMatrix.M[1][1]) - expectedYSigma);
	const float z_sigma_error = FMath::Abs(FMath::Sqrt(covMatrix.M[2][2]) - expectedZSigma);
	UE_LOG(LogPSMove, Log, TEXT("  X Variance Error = %f"), x_sigma_error);
	UE_LOG(LogPSMove, Log, TEXT("  Y Variance Error = %f"), y_sigma_error);
	UE_LOG(LogPSMove, Log, TEXT("  Z Variance Error = %f"), z_sigma_error);
	assert(x_sigma_error < 0.1f);
	assert(y_sigma_error < 0.1f);
	assert(z_sigma_error < 0.1f);
}