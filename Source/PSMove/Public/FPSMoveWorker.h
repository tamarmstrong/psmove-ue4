//
//  FPSMoveWorker.h
//  TestPSMove
//
//  Created by Chadwick Boulay on 2015-02-20.
//  Copyright (c) 2015 EpicGames. All rights reserved.
//
#pragma once

#include "FPSMoveClock.h"
#include "FPSMoveFilter.h"
#include "FPSMoveCalibration.h"

struct FPSMoveRawControllerWorkerData_TLS : public FPSMoveRawControllerData_TLS
{
	FPSMoveControllerClock Clock;
	FPSMovePositionKalmanFilter PositionFilter;

	FPSMoveRawControllerWorkerData_TLS() :
		Clock(),
		PositionFilter()
	{
	}
};

class FPSMoveWorker : public FRunnable
{
public:
    FPSMoveWorker(
		FPSMoveRawSharedData_Concurrent* &PSMoveSharedDataPtr,
		TArray<FPSMoveRawControllerData_Concurrent>* &PSMoveControllerDataArrayPtr); // Usually called by singleton access via Init
    virtual ~FPSMoveWorker();

    /** Thread for polling the controller and tracker */
    FRunnableThread* Thread;

    /** Thread Safe Counter. ?? */
    FThreadSafeCounter StopTaskCounter;
    
	/** Shared data shared amongst all controllers  (i.e. configuration data)  */
	FPSMoveRawSharedData_TLS WorkerSharedData;

    /** An array of raw data structures, one for each controller */
	TArray<FPSMoveRawControllerWorkerData_TLS> WorkerControllerDataArray;

    /** Request the Worker check to see if the number of controllers has changed. */
    void RequestMoveCheck();

    /** FRunnable Interface */
    virtual bool Init(); // override?
    virtual uint32 Run(); // override?
    virtual void Stop(); // override?

    /** Singleton instance for static access. */
    static FPSMoveWorker* WorkerInstance;
    /** Static access to start the thread.*/
	static FPSMoveWorker* PSMoveWorkerInit(
		FPSMoveRawSharedData_Concurrent* &PSMoveSharedDataPtr,
		TArray<FPSMoveRawControllerData_Concurrent>* &PSMoveControllerDataArrayPtr);
    /** Static access to stop the thread.*/
    static void Shutdown();

private:
	/** Published worker data that shouldn't touched directly.
	    Access through _ThreadSafe version of the structures. */
	FPSMoveRawSharedData_Concurrent WorkerSharedData_Concurrent;
	TArray<FPSMoveRawControllerData_Concurrent> WorkerControllerDataArray_Concurrent;
	
	/** We need a reference to the Module's RawData pointers because it will be passed in on initialization but not used until the Thread runs. */
	FPSMoveRawSharedData_Concurrent **ModuleSharedDataPtrPtr;
    TArray<FPSMoveRawControllerData_Concurrent>** ModuleControllerDataArrayPtrPtr;

	/** Calibration State - Used to map the psmove controller state into Unreal */
	FPSMoveCalibration Calibration;

    uint8 PSMoveCount;
    bool MoveCheckRequested;

    bool UpdateMoveCount();
};