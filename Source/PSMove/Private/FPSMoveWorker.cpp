//
//  FPSMoveWorker.cpp
//  TestPSMove
//
//  Created by Chadwick Boulay on 2015-02-20.
//  Copyright (c) 2015 EpicGames. All rights reserved.
//
#include "PSMovePrivatePCH.h"
#include "FPSMoveWorker.h"
#include "FPSMoveCalibration.h"
#include "FPSMove.h"
#include <math.h>
#include <assert.h>

// -- prototypes ----
static void ControllerUpdateRawPositions(
	FPSMoveCalibration *calibration,
	PSMoveTracker *psmove_tracker,
	PSMoveFusion *psmove_fusion,
	PSMove *psmove,
	FPSMoveRawControllerData_Base *controllerData);
static void ControllerUpdateFilteredOrientations(
	PSMove *psmove,
	FPSMoveRawControllerData_Base *controllerData);
static void ControllerUpdateButtonState(
	PSMove *psmove,
	FPSMoveRawControllerData_Base *controllerData);
static void ControllerUpdateAcceleration(
	FPSMoveCalibration *calibration,
	PSMove *psmove,
	FPSMoveRawControllerData_Base *controllerData);
static void ControllerUpdateFilteredPositionAndVelocity(
	FPSMoveCalibration *calibration,
	PSMove *psmove,
	FPSMoveRawControllerWorkerData_TLS *controllerData);

// -- Worker Thread --
FPSMoveWorker* FPSMoveWorker::WorkerInstance = NULL;

FPSMoveWorker::FPSMoveWorker(
	FPSMoveRawSharedData_Concurrent* &PSMoveSharedDataPtr,
	TArray<FPSMoveRawControllerData_Concurrent>* &PSMoveControllerDataArrayPtr) : 
	StopTaskCounter(0), 
	WorkerSharedData(),
	WorkerSharedData_Concurrent(),
	Calibration(&WorkerSharedData),
	PSMoveCount(0), 
	MoveCheckRequested(false)
{
    // Keep a reference to the Module's Data pointers 
	ModuleSharedDataPtrPtr = &PSMoveSharedDataPtr;
    ModuleControllerDataArrayPtrPtr = &PSMoveControllerDataArrayPtr;

	// Bind the concurrent data to the thread-safe container
	WorkerSharedData.ConcurrentData = &WorkerSharedData_Concurrent;
    
    // Need to count the number of controllers before starting the thread to guarantee the data arrays exist before returning.
    bool updated = UpdateMoveCount();
    
    // This Inits and Runs the thread.
    Thread = FRunnableThread::Create(this, TEXT("FPSMoveWorker"), 0, TPri_AboveNormal);
}

bool FPSMoveWorker::UpdateMoveCount()
{
	//TODO: We have a multi-threading issue here if the number of moves decreases here
	// the the component is trying to access controller that went away

    MoveCheckRequested = false;
    int newcount = psmove_count_connected();
    UE_LOG(LogPSMove, Log, TEXT("Found %d PSMove controllers."), newcount);
    if (PSMoveCount != newcount)
    {
		int32 existingCount = PSMoveCount;

        PSMoveCount = newcount;

        // Instantiate the local array of raw data frames (calls constructor on elements)
        WorkerControllerDataArray.SetNum(PSMoveCount);
		WorkerControllerDataArray_Concurrent.SetNum(PSMoveCount);

		// For newly allocated controller data entries,
		// bind the concurrent data to the thread-safe container
		for (int32 index = existingCount; index < PSMoveCount; ++index)
		{
			FPSMoveRawControllerWorkerData_TLS &workerControllerData = WorkerControllerDataArray[index];

			workerControllerData.ConcurrentData = &WorkerControllerDataArray_Concurrent[index];
			workerControllerData.Clock.Initialize();
			workerControllerData.PositionFilter.Clear();
		}

        // Set the module's ModuleControllerDataArrayPtr to point to this thread's array of controller data frames.
		(*ModuleControllerDataArrayPtrPtr) = &WorkerControllerDataArray_Concurrent;
        
        return true;
    } else {
        return false;
    }
}

FPSMoveWorker::~FPSMoveWorker()
{
    UE_LOG(LogPSMove, Log, TEXT("FPSMove Destructor"));
    delete Thread;
    Thread = NULL;
}

bool FPSMoveWorker::Init()
{
    bool updated = UpdateMoveCount();

	// Set the module's ModuleSharedDataPtr to point to this thread's concurrent shared data.
	// This pointer will get bound to a FPSMoveRawSharedData_TLS in the modules thread.
	// This stay fixed regardless of number of controllers.
	(*ModuleSharedDataPtrPtr) = &WorkerSharedData_Concurrent;

    return true;
}

uint32 FPSMoveWorker::Run()
{
	if (!psmove_init(PSMOVE_CURRENT_VERSION)) 
	{
		UE_LOG(LogPSMove, Error, TEXT("PS Move API init failed (wrong version?)"));
		return -1;
	}

    // I want the psmoves and psmove_tracker to be local variables in the thread.    
    //// Initialize an empty array of psmove controllers
    TArray<PSMove*> psmoves;
    
    // Initialize and configure the psmove_tracker
    PSMoveTracker *psmove_tracker = psmove_tracker_new(); // Unfortunately the API does not have a way to change the resolution and framerate.
    int tracker_width = 640;
    int tracker_height = 480;
    if (psmove_tracker)
    {
        UE_LOG(LogPSMove, Log, TEXT("PSMove tracker initialized."));
        
        //Set exposure. TODO: Make this configurable.
        psmove_tracker_set_exposure(psmove_tracker, Exposure_LOW);  //Exposure_LOW, Exposure_MEDIUM, Exposure_HIGH
        
        psmove_tracker_get_size(psmove_tracker, &tracker_width, &tracker_height);
        UE_LOG(LogPSMove, Log, TEXT("Camera Dimensions: %f x %f"), tracker_width, tracker_height);
    }
    else {
        UE_LOG(LogPSMove, Log, TEXT("PSMove tracker failed to initialize."));
    }

	PSMoveFusion *psmove_fusion = NULL;
	if (psmove_tracker != NULL)
	{ 
		psmove_fusion = psmove_fusion_new(psmove_tracker, 1.0f, 1000.0f);
		UE_LOG(LogPSMove, Log, TEXT("PSMove creating fusion tracking"));
	}
    
    // TODO: Move this inside the loop to run if PSMoveCount changes.
    for (int i = 0; i < PSMoveCount; i++)
    {
        psmoves.Add(psmove_connect_by_id(i));
        assert(psmoves[i] != NULL);
        
        psmove_enable_orientation(psmoves[i], PSMove_True);
        assert(psmove_has_orientation(psmoves[i]));
        
        WorkerControllerDataArray[i].IsConnected = true;
        
        if (psmove_tracker)
        {
            while (psmove_tracker_enable(psmove_tracker, psmoves[i]) != Tracker_CALIBRATED); // Keep attempting to enable.
            
            //TODO: psmove_tracker_enable_with_color(psmove_tracker, psmoves[i], r, g, b)
            //TODO: psmove_tracker_get_color(psmove_tracker, psmoves[i], unisgned char &r, &g, &b);
            
            PSMove_Bool auto_update_leds = psmove_tracker_get_auto_update_leds(psmove_tracker, psmoves[i]);
            
            WorkerControllerDataArray[i].IsTracked = true;
        }
    }

	// - Set calibration defaults and load the calibration data from the config file
	// - Reset the calibration clock used by the Kalman filter
	Calibration.Initialize();

    //Initial wait before starting.
    FPlatformProcess::Sleep(0.03);

    while (StopTaskCounter.GetValue() == 0 && PSMoveCount > 0)
    {
		QUICK_SCOPE_CYCLE_COUNTER(Stat_FPSMoveWorker_RunLoop)

		bool wasCalibrationButtonPressedThisFrame = false;

		//--------------
		// Read the published data from the component
		//--------------
		WorkerSharedData.WorkerRead();

        if (MoveCheckRequested && UpdateMoveCount())
        {
            // TODO: Fix psmoves, psmove_enable_orientation, psmove_tracker_enable
			Calibration.OnControllerCountChanged();
        }
		else
		{
			// Reset the calibration status if the component has acknowledged the status changes
			Calibration.SharedUpdate();
		}
        
        // Update the raw position
        if (psmove_tracker)
        {
			{
				QUICK_SCOPE_CYCLE_COUNTER(Stat_FPSMoveWorker_UpdateImage)

					// Renew the image on camera
					psmove_tracker_update_image(psmove_tracker); // Sometimes libusb crashes here.
			}

			{
				QUICK_SCOPE_CYCLE_COUNTER(Stat_FPSMoveWorker_RawPosition)

				for (int controllerIndex = 0; controllerIndex < PSMoveCount; controllerIndex++)
				{
					FPSMoveRawControllerWorkerData_TLS &localControllerData = WorkerControllerDataArray[controllerIndex];

					// Update the raw positions on the local controller data
					ControllerUpdateRawPositions(
						&Calibration,
						psmove_tracker,
						psmove_fusion,
						psmoves[controllerIndex],
						&localControllerData);
				}
			}
        } 
		else 
		{
            FPlatformProcess::Sleep(0.001);
        }
        
		{
			QUICK_SCOPE_CYCLE_COUNTER(Stat_FPSMoveWorker_Polling)

			// Do bluetooth IO: Orientation, Buttons, Rumble
			for (int controllerIndex = 0; controllerIndex < PSMoveCount; controllerIndex++)
			{
				//TODO: Is it necessary to keep polling until no frames are left?
				while (psmove_poll(psmoves[controllerIndex]) > 0)
				{
					FPSMoveRawControllerWorkerData_TLS &localControllerData = WorkerControllerDataArray[controllerIndex];

					// Update the controller status (via bluetooth)
					psmove_poll(psmoves[controllerIndex]);

					// [Store the controller orientation]
					ControllerUpdateFilteredOrientations(psmoves[controllerIndex], &localControllerData);

					// [Store the button state]
					ControllerUpdateButtonState(psmoves[controllerIndex], &localControllerData);

					//--------------
					// Publish the worker data to the component
					// Read back the published data from the component
					//--------------
					localControllerData.WorkerPostAndRead();

					// Set the controller rumble (uint8; 0-255)
					psmove_set_rumble(psmoves[controllerIndex], localControllerData.RumbleRequest);

					// See if the reset orientation request has been posted by the component
					if (localControllerData.ResetOrientationRequest)
					{
						UE_LOG(LogPSMove, Log, TEXT("FPSMoveWorker:: RESET ORIENTATION"));

						psmove_reset_orientation(psmoves[controllerIndex]);

						// Reset the position filter too.
						// If the orientation was wrong then we have been getting
						// mis-aligned acceleration data.
						localControllerData.PositionFilter.Clear();

						// Clear the request flag now that we've handled the request
						localControllerData.ResetOrientationRequest = false;
					}
				}
			}
		}

		{
			QUICK_SCOPE_CYCLE_COUNTER(Stat_FPSMoveWorker_Filtering)

			// Update the acceleration, filtered position, and filtered velocity
			for (int controllerIndex = 0; controllerIndex < PSMoveCount; controllerIndex++)
			{
				FPSMoveRawControllerWorkerData_TLS &localControllerData = WorkerControllerDataArray[controllerIndex];

				// Refresh the clock for the controller.
				// This is needed for the positional filter to update.
				localControllerData.Clock.Update();

				// Update the acceleration of the psmove for the filter
				ControllerUpdateAcceleration(
					&Calibration, psmoves[controllerIndex], &localControllerData);

				ControllerUpdateFilteredPositionAndVelocity(
					&Calibration, psmoves[controllerIndex], &localControllerData);
			}

			// Potentially update the calibration state based on controller input
			for (int controllerIndex = 0; controllerIndex < PSMoveCount; controllerIndex++)
			{
				FPSMoveRawControllerWorkerData_TLS &localControllerData = WorkerControllerDataArray[controllerIndex];

				// See if the calibration button was pressed for any controller
				Calibration.ControllerUpdate(controllerIndex, psmoves[controllerIndex], &localControllerData);
			}

			//--------------
			// Publish the worker data to the component (Calibration status)
			//--------------
			WorkerSharedData.WorkerPost();

			//Sleeping the thread seems to crash libusb.
			//FPlatformProcess::Sleep(0.005);   
		}
    }
    
    // Delete the controllers
    for (int i = 0; i<PSMoveCount; i++)
    {
        psmove_disconnect(psmoves[i]);
    }

	// Delete the tracking fusion object
	if (psmove_fusion)
	{
		psmove_fusion_free(psmove_fusion);
	}
    
    // Delete the tracker
    if (psmove_tracker)
    {
        psmove_tracker_free(psmove_tracker);
    }

	// Free any dependant APIs
	psmove_shutdown();

    return 0;
}

void FPSMoveWorker::Stop()
{
    StopTaskCounter.Increment();
}

FPSMoveWorker* FPSMoveWorker::PSMoveWorkerInit(
	FPSMoveRawSharedData_Concurrent* &PSMoveSharedDataPtr,
	TArray<FPSMoveRawControllerData_Concurrent>* &PSMoveControllerDataArrayPtr)
{
    UE_LOG(LogPSMove, Log, TEXT("FPSMoveWorker::PSMoveWorkerInit"));
    if (!WorkerInstance && FPlatformProcess::SupportsMultithreading())
    {
        UE_LOG(LogPSMove, Log, TEXT("Creating new FPSMoveWorker instance."));
		WorkerInstance = new FPSMoveWorker(PSMoveSharedDataPtr, PSMoveControllerDataArrayPtr);
    } else if (WorkerInstance) {
        UE_LOG(LogPSMove, Log, TEXT("FPSMoveWorker already instanced."));
    }
    return WorkerInstance;
}

void FPSMoveWorker::Shutdown()
{
    if (WorkerInstance)
    {
        UE_LOG(LogPSMove, Log, TEXT("Shutting down PSMoveWorker instance."));
        WorkerInstance->Stop();
        WorkerInstance->Thread->WaitForCompletion();
        delete WorkerInstance; // Destructor SHOULD turn off tracker.
        WorkerInstance = NULL;
        UE_LOG(LogPSMove, Log, TEXT("PSMoveWorker instance destroyed."));
    }
}

//-- private methods ----
static void ControllerUpdateRawPositions(
	FPSMoveCalibration *calibration,
	PSMoveTracker *psmove_tracker,
	PSMoveFusion *psmove_fusion,
	PSMove *psmove, 
	FPSMoveRawControllerData_Base *controllerData)
{
	// Find the sphere position in the camera
	int found_sphere_count = 0;

	{
		QUICK_SCOPE_CYCLE_COUNTER(Stat_FPSMoveWorker_TrackerUpdate)
		found_sphere_count = psmove_tracker_update(psmove_tracker, psmove);

	}

	// Can we actually see the controller this frame?
	controllerData->CanSee = (found_sphere_count > 0);

	// Update the position of the controller
	if (controllerData->CanSee)
	{
		QUICK_SCOPE_CYCLE_COUNTER(Stat_FPSMoveWorker_TrackerFusion)

		float xcm, ycm, zcm;
		const FMatrix &PSMoveSpaceToHMDSpace = calibration->GetPSMoveSpaceToHMDSpace();

		//psmove_tracker_get_location(psmove_tracker, psmove, &xcm, &ycm, &zcm);
		psmove_fusion_get_position(psmove_fusion, psmove, &xcm, &ycm, &zcm);

		// [Store the unfiltered controller position]
		// Remember the position the ps move controller thinks it's at relative to the PS Eye camera 
		controllerData->PSMoveRelativeRawPosition = FVector(xcm, ycm, zcm);

		// The given position of the controller is in the space of the PSEye Camera.
		// Use the PSMoveSpaceToHMDSpace transform to put it in the space of the HMD.
		controllerData->WorldRelativeRawPosition =
			PSMoveSpaceToHMDSpace.TransformPosition(controllerData->PSMoveRelativeRawPosition);
	}
}

static void ControllerUpdateFilteredOrientations(
	PSMove *psmove,
	FPSMoveRawControllerData_Base *controllerData)
{
	float oriw, orix, oriy, oriz;

	// Get the controller orientation (uses IMU).
	psmove_get_orientation(psmove, &oriw, &orix, &oriy, &oriz);
	
	controllerData->PSMoveRelativeFilteredOrientation = FQuat(orix, oriy, oriz, oriw);

	controllerData->WorldRelativeFilteredOrientation = 
		// Convert from the controller coordinate system to Unreal's coordinate system where
		FQuat(oriz, -orix, -oriy, oriw);

}

static void ControllerUpdateButtonState(
	PSMove *psmove,
	FPSMoveRawControllerData_Base *controllerData)
{
	// Get the controller button state
	controllerData->Buttons = psmove_get_buttons(psmove);  // Bitwise; tells if each button is down.
	psmove_get_button_events(psmove, &controllerData->Pressed, &controllerData->Released);  // i.e., state change

	// Get the controller trigger value (uint8; 0-255)
	controllerData->TriggerValue = psmove_get_trigger(psmove);
}

static void ControllerUpdateAcceleration(
	FPSMoveCalibration *calibration,
	PSMove *psmove,
	FPSMoveRawControllerData_Base *controllerData)
{
	const float gravityUnits = calibration->GetGravityUnits();
	const FVector skew = calibration->GetPSMoveSpaceAccelerationSkew();

	FQuat orientation = controllerData->WorldRelativeFilteredOrientation;
	float ax, ay, az;

	// Get the raw accelerometer data from the psmove
	psmove_get_accelerometer_frame(psmove, Frame_SecondHalf, &ax, &ay, &az);

	// Remember the raw (non-skew corrected) acceleration in PSMove Space
	controllerData->PSMoveRelativeRawAcceleration = FVector(ax, ay, az);

	// Correct for accelerometer reading skew along each axis
	ax += skew.X;
	ay += skew.Y;
	az += skew.Z;

	// Convert the raw acceleration from PSMove Space into Unreal World Space
	controllerData->WorldRelativeRawAcceleration=
		orientation.GetAxisX() * ay + // +x in Unreal is +x in PSMoveLib  
		orientation.GetAxisY() * ax + // +y in Unreal is +y in PSMoveLib 
		orientation.GetAxisZ() * az;  // +z in Unreal is +z in PSMoveLib

	// The acceleration is currently in units of 'g'.
	// World space units are millimeters.
	controllerData->WorldRelativeRawAcceleration = 
		controllerData->WorldRelativeRawAcceleration * gravityUnits;
}

static void ControllerUpdateFilteredPositionAndVelocity(
	FPSMoveCalibration *calibration,
	PSMove *psmove,
	FPSMoveRawControllerWorkerData_TLS *controllerData)
{
	if (controllerData->CanSee)
	{
		const float gravityUnits = calibration->GetGravityUnits();
		FVector WorldSpaceControlAcceleration;

		// Subtract off acceleration due to gravity to get acceleration player is imparting on the controller.
		// When the controller is standing still upright, the accelerometer will report <0g,+1g,0g>.
		// Since +Z is up in Unreal, we need to subtract off <0, 0, g>
		WorldSpaceControlAcceleration =
			controllerData->WorldRelativeRawAcceleration
			- FVector(0.f, 0.f, gravityUnits);

		// Initialize the filter if it hasn't been yet been
		if (!controllerData->PositionFilter.GetIsInitialized())
		{
			controllerData->PositionFilter.Initialize(
				calibration->GetAccelerationVariance(),
				calibration->GetMeasurementNoiseMatrix(),
				controllerData->WorldRelativeRawPosition);
		}

		// Update the positional Kalman Filter
		controllerData->PositionFilter.Update(
			controllerData->WorldRelativeRawPosition,
			FVector(0, 0, 0), // WorldSpaceControlAcceleration
			controllerData->Clock.TimeDeltaInSeconds);

		// Get the filtered position and velocity from the filter
		controllerData->WorldRelativeFilteredPosition =
			controllerData->PositionFilter.GetFilteredPosition();
		controllerData->WorldRelativeFilteredVelocity =
			controllerData->PositionFilter.GetFilteredVelocity();
	}
	else
	{
		// The filter becomes irrelevant when we can see the controller
		// TODO: Maybe not? We could make a prediction of the position using the acceleration and filter that? 
		controllerData->PositionFilter.Clear();
	}
}