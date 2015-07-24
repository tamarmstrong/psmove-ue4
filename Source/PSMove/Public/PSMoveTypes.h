#pragma once

#include "EnumAsByte.h"
#include "FPSMoveClock.h"

//---------------------------------------------------
// Enum types
//---------------------------------------------------

UENUM(BlueprintType)
namespace EPSMoveCalibrationStatusEnum
{
	//256 entries max
	enum Type
	{
		Inactive			UMETA(DisplayName = "Inactive"),
		WaitForStationary	UMETA(DisplayName = "WaitForStationary"),
		StationarySampling	UMETA(DisplayName = "StationarySampling"),
		HMDSampling			UMETA(DisplayName = "HMDSampling"),
		Succeeded			UMETA(DisplayName = "Succeeded"),
		Failed				UMETA(DisplayName = "Failed"),

		//~~~

		//256th entry
		VE_Max				UMETA(Hidden),
	};
}

UENUM(BlueprintType)
namespace EPSMoveCalibrationStepResultEnum
{
	//256 entries max
	enum Type
	{
		Waiting		UMETA(DisplayName = "Waiting"),
		Valid		UMETA(DisplayName = "Valid"),
		Invalid		UMETA(DisplayName = "Invalid"),

		//~~~

		//256th entry
		VE_Max        UMETA(Hidden),
	};
}

//---------------------------------------------------
// Delegate types
//---------------------------------------------------
#include "PSMoveTypes.generated.h"

DECLARE_DYNAMIC_MULTICAST_DELEGATE_OneParam(FPSMoveAcknowledgeCalibrationPointDelegate, EPSMoveCalibrationStepResultEnum::Type, Result);
DECLARE_DYNAMIC_MULTICAST_DELEGATE_OneParam(FPSMoveCalibrationStateChangeDelegate, EPSMoveCalibrationStatusEnum::Type, Status);
DECLARE_DYNAMIC_MULTICAST_DELEGATE_FiveParams(FPSMoveDataUpdatedDelegate, FVector, RawPosition, FVector, FilteredPosition, FRotator, FilteredRotation, FVector, FilteredVelocity, FVector, RawAcceleration);
DECLARE_DYNAMIC_MULTICAST_DELEGATE_OneParam(FPSMoveButtonStateDelegate, bool, IsDown);
DECLARE_DYNAMIC_MULTICAST_DELEGATE_OneParam(FPSMoveTriggerButtonDelegate, uint8, Value);

//---------------------------------------------------
// Structures
//---------------------------------------------------

// Data needed by the worker common to all active move controllers.
USTRUCT()
struct FPSMoveRawSharedData_Base
{
	GENERATED_USTRUCT_BODY();

	// Worker -> Component
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = Enum)
	TEnumAsByte<EPSMoveCalibrationStatusEnum::Type> CalibrationStatusEnum;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = Enum)
	TEnumAsByte<EPSMoveCalibrationStepResultEnum::Type> CalibrationPointResultEnum;

	// Component -> Worker
	UPROPERTY()
	FVector HMDTrackingSpacePosition;

	UPROPERTY()
	FQuat HMDTrackingSpaceOrientation;

	UPROPERTY()
	bool HMDIsTracked;

	UPROPERTY()
	bool ComponentHasNotReadWorkerData;

	FPSMoveRawSharedData_Base()
	{
		CalibrationStatusEnum = EPSMoveCalibrationStatusEnum::Inactive;
		CalibrationPointResultEnum = EPSMoveCalibrationStepResultEnum::Waiting;
		HMDTrackingSpacePosition = FVector::ZeroVector;
		HMDTrackingSpaceOrientation = FQuat::Identity;
		HMDIsTracked = false;
		ComponentHasNotReadWorkerData = false;
	}
};

USTRUCT()
struct FPSMoveRawSharedData_Concurrent : public FPSMoveRawSharedData_Base
{
	GENERATED_USTRUCT_BODY();

	// Used to make sure we're accessing the data in a thread safe way
	FCriticalSection Lock;

	FPSMoveRawSharedData_Concurrent() :
		FPSMoveRawSharedData_Base(),
		Lock()
	{
	}
};

USTRUCT()
struct FPSMoveRawSharedData_TLS : public FPSMoveRawSharedData_Base
{
	GENERATED_USTRUCT_BODY();

	// This is a pointer to the shared data
	FPSMoveRawSharedData_Concurrent *ConcurrentData;

	// Flag to denote the worker has modified it's TLS data
	bool WorkerHasModifiedTLSData;

	FPSMoveRawSharedData_TLS() :
		FPSMoveRawSharedData_Base(),
		ConcurrentData(nullptr),
		WorkerHasModifiedTLSData(false)
	{
	}

	bool IsValid() const
	{
		return ConcurrentData != nullptr;
	}

	void MarkDirty()
	{
		this->WorkerHasModifiedTLSData = true;
	}

	void ComponentPostAndRead()
	{
		FPSMoveHitchWatchdog watchdog(TEXT("FPSMoveRawSharedData_TLS::ComponentPostAndRead"), 500);

		{
			FScopeLock scopeLock(&ConcurrentData->Lock);

			// Post the component thread's data to the worker thread's data
			ConcurrentData->HMDTrackingSpacePosition = this->HMDTrackingSpacePosition;
			ConcurrentData->HMDTrackingSpaceOrientation = this->HMDTrackingSpaceOrientation;
			ConcurrentData->HMDIsTracked = this->HMDIsTracked;

			// Read the worker thread's data into the component thread's data
			this->CalibrationStatusEnum = ConcurrentData->CalibrationStatusEnum;
			this->CalibrationPointResultEnum = ConcurrentData->CalibrationPointResultEnum;

			// We've read the data the worker posted, so it's no longer dirty
			ConcurrentData->ComponentHasNotReadWorkerData = false;
		}
	}

	void WorkerRead()
	{
		FPSMoveHitchWatchdog watchdog(TEXT("FPSMoveRawSharedData_TLS::WorkerRead"), 500);

		{
			FScopeLock scopeLock(&ConcurrentData->Lock);

			// Read the component thread's data into the worker thread's data
			this->HMDTrackingSpacePosition = ConcurrentData->HMDTrackingSpacePosition;
			this->HMDTrackingSpaceOrientation = ConcurrentData->HMDTrackingSpaceOrientation;
			this->HMDIsTracked = ConcurrentData->HMDIsTracked;
		}
	}

	void WorkerPost()
	{
		FPSMoveHitchWatchdog watchdog(TEXT("FPSMoveRawSharedData_TLS::WorkerPost"), 500);

		if (WorkerHasModifiedTLSData)
		{
			FScopeLock scopeLock(&ConcurrentData->Lock);

			// Post the worker thread's data to the component thread's data
			ConcurrentData->CalibrationStatusEnum = this->CalibrationStatusEnum;
			ConcurrentData->CalibrationPointResultEnum = this->CalibrationPointResultEnum;

			// The component has new data to read from the worker thread
			ConcurrentData->ComponentHasNotReadWorkerData = true;

			// The worker threads TLS data is no longer dirty
			WorkerHasModifiedTLSData = false;
		}
	}
};

// Per-Move Controller data needed by the worker.
USTRUCT()
struct FPSMoveRawControllerData_Base
{
    GENERATED_USTRUCT_BODY();

	// Worker -> Component
	UPROPERTY()
	FVector PSMoveRelativeRawPosition;

	UPROPERTY()
	FVector PSMoveRelativeRawAcceleration;

	UPROPERTY()
	FQuat PSMoveRelativeFilteredOrientation;

    UPROPERTY()
    FVector WorldRelativeRawPosition;

	UPROPERTY()
	FVector WorldRelativeFilteredPosition;

    UPROPERTY()
	FQuat WorldRelativeFilteredOrientation;

	UPROPERTY()
	FVector WorldRelativeFilteredVelocity;

	UPROPERTY()
	FVector WorldRelativeRawAcceleration;

    UPROPERTY()
    uint32 Buttons;
    
    UPROPERTY()
    uint32 Pressed;
    
    UPROPERTY()
    uint32 Released;
    
    UPROPERTY()
    uint8 TriggerValue;
       
    UPROPERTY()
    bool IsConnected;
    
    UPROPERTY()
    bool IsTracked; // Means that we registered the camera at startup

	UPROPERTY()
	bool CanSee; // The camera can see this controller right at this moment

	// Component -> Worker
	UPROPERTY()
	uint8 RumbleRequest;

	UPROPERTY()
	bool ResetOrientationRequest;

	UPROPERTY()
	bool CalibrationActionRequest;
    
    //TODO: LEDs
    
    // Constructor
	FPSMoveRawControllerData_Base()
    {
		PSMoveRelativeRawPosition = FVector::ZeroVector;
		PSMoveRelativeRawAcceleration = FVector::ZeroVector;
		PSMoveRelativeFilteredOrientation = FQuat::Identity;
		WorldRelativeRawPosition = FVector::ZeroVector;
		WorldRelativeFilteredPosition = FVector::ZeroVector;
		WorldRelativeFilteredOrientation = FQuat::Identity;
		WorldRelativeFilteredVelocity = FVector::ZeroVector;
		WorldRelativeRawAcceleration = FVector::ZeroVector;
        Buttons = 0;
        Pressed = 0;
        Released = 0;
        TriggerValue = 0;
        IsConnected = false;
        IsTracked = false;
		CanSee = false;

		RumbleRequest = 0;
		ResetOrientationRequest = false;
		CalibrationActionRequest = false;
    }
};

USTRUCT()
struct FPSMoveRawControllerData_Concurrent : public FPSMoveRawControllerData_Base
{
	GENERATED_USTRUCT_BODY();

	// Used to make sure we're accessing the data in a thread safe way
	FCriticalSection Lock;

	FPSMoveRawControllerData_Concurrent() :
		FPSMoveRawControllerData_Base(),
		Lock()
	{
	}
};

USTRUCT()
struct FPSMoveRawControllerData_TLS : public FPSMoveRawControllerData_Base
{
	GENERATED_USTRUCT_BODY();

	// This is a pointer to the 
	FPSMoveRawControllerData_Concurrent *ConcurrentData;

	FPSMoveRawControllerData_TLS() :
		FPSMoveRawControllerData_Base(),
		ConcurrentData(nullptr)
	{
	}

	bool IsValid() const
	{
		return ConcurrentData != nullptr;
	}

	void ComponentPostAndRead()
	{
		FPSMoveHitchWatchdog watchdog(TEXT("FPSMoveRawControllerData_TLS::ComponentPostAndRead"), 500);

		{
			FScopeLock scopeLock(&ConcurrentData->Lock);

			// Post the component thread's data to the worker thread's data
			ConcurrentData->RumbleRequest = this->RumbleRequest;
			ConcurrentData->ResetOrientationRequest = this->ResetOrientationRequest;
			ConcurrentData->CalibrationActionRequest = this->CalibrationActionRequest;

			// Read the worker thread's data into the component thread's data
			this->PSMoveRelativeRawPosition = ConcurrentData->PSMoveRelativeRawPosition;
			this->PSMoveRelativeRawAcceleration = ConcurrentData->PSMoveRelativeRawAcceleration;
			this->PSMoveRelativeFilteredOrientation = ConcurrentData->PSMoveRelativeFilteredOrientation;
			this->WorldRelativeRawPosition = ConcurrentData->WorldRelativeRawPosition;
			this->WorldRelativeFilteredPosition = ConcurrentData->WorldRelativeFilteredPosition;
			this->WorldRelativeFilteredOrientation = ConcurrentData->WorldRelativeFilteredOrientation;
			this->WorldRelativeFilteredVelocity = ConcurrentData->WorldRelativeFilteredVelocity;
			this->WorldRelativeRawAcceleration = ConcurrentData->WorldRelativeRawAcceleration;
			this->Buttons = ConcurrentData->Buttons;
			this->Pressed = ConcurrentData->Pressed;
			this->Released = ConcurrentData->Released;
			this->TriggerValue = ConcurrentData->TriggerValue;
			this->IsConnected = ConcurrentData->IsConnected;
			this->IsTracked = ConcurrentData->IsTracked;
			this->CanSee = ConcurrentData->CanSee;
		}
	}

	void WorkerPostAndRead()
	{
		FPSMoveHitchWatchdog watchdog(TEXT("FPSMoveRawControllerData_TLS::ComponentPostAndRead"), 500);

		{
			FScopeLock scopeLock(&ConcurrentData->Lock);

			// Post the worker thread's data to the component thread's data
			ConcurrentData->PSMoveRelativeRawPosition = this->PSMoveRelativeRawPosition;
			ConcurrentData->PSMoveRelativeRawAcceleration = this->PSMoveRelativeRawAcceleration;
			ConcurrentData->PSMoveRelativeFilteredOrientation = this->PSMoveRelativeFilteredOrientation;
			ConcurrentData->WorldRelativeRawPosition = this->WorldRelativeRawPosition;
			ConcurrentData->WorldRelativeFilteredPosition = this->WorldRelativeFilteredPosition;
			ConcurrentData->WorldRelativeFilteredOrientation = this->WorldRelativeFilteredOrientation;
			ConcurrentData->WorldRelativeFilteredVelocity = this->WorldRelativeFilteredVelocity;
			ConcurrentData->WorldRelativeRawAcceleration = this->WorldRelativeRawAcceleration;
			ConcurrentData->Buttons = this->Buttons;
			ConcurrentData->Pressed = this->Pressed;
			ConcurrentData->Released = this->Released;
			ConcurrentData->TriggerValue = this->TriggerValue;
			ConcurrentData->IsConnected = this->IsConnected;
			ConcurrentData->IsTracked = this->IsTracked;
			ConcurrentData->CanSee = this->CanSee;

			// Read the component thread's data into the worker thread's data
			this->RumbleRequest = ConcurrentData->RumbleRequest;
			this->ResetOrientationRequest = ConcurrentData->ResetOrientationRequest;
			this->CalibrationActionRequest = ConcurrentData->CalibrationActionRequest;
		}
	}
};

USTRUCT()
struct FPSMoveDataContext
{
    GENERATED_USTRUCT_BODY();
    
    UPROPERTY()
    int32 PSMoveID;
    
	FPSMoveRawSharedData_TLS RawSharedData;
    FPSMoveRawControllerData_TLS RawControllerData;

	void ComponentPostAndRead()
	{
		if (RawSharedData.IsValid())
		{
			RawSharedData.ComponentPostAndRead();
		}

		if (RawControllerData.IsValid())
		{
			RawControllerData.ComponentPostAndRead();
		}
	}

	// Shared Data Functions
	EPSMoveCalibrationStepResultEnum::Type GetCalibrationPointResult()
	{
		if (RawSharedData.IsValid())
		{
			return RawSharedData.CalibrationPointResultEnum;
		}
		else
		{
			return EPSMoveCalibrationStepResultEnum::Waiting;
		}
	}

	EPSMoveCalibrationStatusEnum::Type GetCalibrationStatus()
	{
		if (RawSharedData.IsValid())
		{
			return RawSharedData.CalibrationStatusEnum;
		}
		else
		{
			return EPSMoveCalibrationStatusEnum::Inactive;
		}
	}

	bool IsCalibrating()
	{
		bool isCalibrating = false;

		if (RawSharedData.IsValid())
		{
			switch (RawSharedData.CalibrationStatusEnum)
			{
			case EPSMoveCalibrationStatusEnum::Inactive:
			case EPSMoveCalibrationStatusEnum::Succeeded:
			case EPSMoveCalibrationStatusEnum::Failed:
				isCalibrating = false;
				break;
			case EPSMoveCalibrationStatusEnum::WaitForStationary:
			case EPSMoveCalibrationStatusEnum::StationarySampling:
			case EPSMoveCalibrationStatusEnum::HMDSampling:
				isCalibrating = true;
				break;
			default:
				checkNoEntry();
			}
		}

		return isCalibrating;
	}

	void SetHMDIsTracked(bool hmdIsTracked)
	{
		if (RawSharedData.IsValid())
		{
			RawSharedData.HMDIsTracked= hmdIsTracked;
		}
	}

	void SetPSMoveHMDTrackingSpacePosition(FVector hmdPosition)
	{
		if (RawSharedData.IsValid())
		{
			RawSharedData.HMDTrackingSpacePosition = hmdPosition;
		}
	}

	void SetPSMoveHMDTrackingSpaceOrientation(FQuat hmdOrientation)
	{
		if (RawSharedData.IsValid())
		{
			RawSharedData.HMDTrackingSpaceOrientation = hmdOrientation;
		}
	}

	// Controller Data Functions
	FVector GetRawPosition()
	{
		if (RawControllerData.IsValid() && RawControllerData.CanSee)
		{
			return RawControllerData.WorldRelativeRawPosition;
		}
		else
		{
			return FVector(0.0);
		}
	}

	FVector GetRawAcceleration()
	{
		if (RawControllerData.IsValid() && RawControllerData.CanSee)
		{
			return RawControllerData.WorldRelativeRawAcceleration;
		}
		else
		{
			return FVector(0.0);
		}
	}

    FVector GetPosition()
    {
        if (RawControllerData.IsValid() && RawControllerData.CanSee)
        {
			return RawControllerData.WorldRelativeFilteredPosition;
        } 
		else 
		{
            return FVector(0.0);
        }
    }

	FVector GetVelocity()
	{
		if (RawControllerData.IsValid() && RawControllerData.CanSee)
		{
			return RawControllerData.WorldRelativeFilteredVelocity;
		}
		else
		{
			return FVector(0.0);
		}
	}
    
    FQuat GetRotation()
    {
        if (RawControllerData.IsValid() && RawControllerData.IsConnected)
        {
			return RawControllerData.WorldRelativeFilteredOrientation;
        }
		else
		{
			return FQuat::Identity;
        }
    }
    
	uint8 GetTriggerValue()
	{
		if (RawControllerData.IsValid() && RawControllerData.IsConnected)
		{
			return RawControllerData.TriggerValue;
		}
		else {
			return 0;
		}
	}

	void SetRumbleRequest(uint8 RequestedRumbleValue)
	{
		if (RawControllerData.IsValid() && RawControllerData.IsConnected)
		{
			RawControllerData.RumbleRequest = RequestedRumbleValue;
		}
	}

	void SetOrientationResetRequest(bool ResetOrientationRequest)
	{
		if (RawControllerData.IsValid() && RawControllerData.IsConnected)
		{
			RawControllerData.ResetOrientationRequest = ResetOrientationRequest;
		}
	}

	void SetCalibrationActionRequest(bool CalibrationActionRequest)
	{
		if (RawControllerData.IsValid() && RawControllerData.IsConnected)
		{
			RawControllerData.CalibrationActionRequest = CalibrationActionRequest;
		}
	}

	// Currently Pressed
    bool GetButtonTriangle()
    {
		if (RawControllerData.IsValid() && RawControllerData.IsConnected)
        {
            return (RawControllerData.Buttons & PSMove_Button::Btn_TRIANGLE) != 0;
        } else {
            return 0;
        }
    }
    
    bool GetButtonCircle()
    {
		if (RawControllerData.IsValid() && RawControllerData.IsConnected)
        {
            return (RawControllerData.Buttons & PSMove_Button::Btn_CIRCLE) != 0;
        } else {
            return 0;
        }
    }
    
    bool GetButtonCross()
    {
		if (RawControllerData.IsValid() && RawControllerData.IsConnected)
        {
            return (RawControllerData.Buttons & PSMove_Button::Btn_CROSS) != 0;
        } else {
            return 0;
        }
    }
    
    bool GetButtonSquare()
    {
		if (RawControllerData.IsValid() && RawControllerData.IsConnected)
        {
            return (RawControllerData.Buttons & PSMove_Button::Btn_SQUARE) != 0;
        } else {
            return 0;
        }
    }
    
    bool GetButtonSelect()
    {
		if (RawControllerData.IsValid() && RawControllerData.IsConnected)
        {
            return (RawControllerData.Buttons & PSMove_Button::Btn_SELECT) != 0;
        } else {
            return 0;
        }
    }
    
    bool GetButtonStart()
    {
		if (RawControllerData.IsValid() && RawControllerData.IsConnected)
        {
            return (RawControllerData.Buttons & PSMove_Button::Btn_START) != 0;
        } else {
            return 0;
        }
    }
    
    bool GetButtonPS()
    {
		if (RawControllerData.IsValid() && RawControllerData.IsConnected)
        {
            return (RawControllerData.Buttons & PSMove_Button::Btn_PS) != 0;
        } else {
            return 0;
        }
    }
    
    bool GetButtonMove()
    {
		if (RawControllerData.IsValid() && RawControllerData.IsConnected)
        {
            return (RawControllerData.Buttons & PSMove_Button::Btn_MOVE) != 0;
        } else {
            return 0;
        }
    }
   
    FPSMoveDataContext()
    {
        PSMoveID = -1;
    }
    
    void Destroy()
    {
        RawControllerData.ConcurrentData = nullptr;
		RawSharedData.ConcurrentData = nullptr;
    }
    
};

UCLASS()
class UPSMoveTypes : public UObject
{
	GENERATED_BODY()
};
