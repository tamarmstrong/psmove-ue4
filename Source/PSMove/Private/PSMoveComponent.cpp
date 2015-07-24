#include "PSMovePrivatePCH.h"
#include "PSMoveComponent.h"
#include "FPSMove.h"

UPSMoveComponent::UPSMoveComponent(const FObjectInitializer &init)
    : PSMoveID(0)
{
    bWantsInitializeComponent = true;
    PrimaryComponentTick.bCanEverTick = true;
	CachedHMDTrackingToWorldSpaceTransform = FTransform::Identity;
}

// Called when the game starts
void UPSMoveComponent::InitializeComponent()
{
    Super::InitializeComponent();
    if (FPSMove::IsAvailable())
    {
        FPSMove::Get().InitWorker();

		// Bind the Concurrent Data to the data context
		FPSMove::Get().GetRawSharedDataPtr(DataContext.RawSharedData.ConcurrentData);
        FPSMove::Get().GetRawControllerDataPtr(PSMoveID, DataContext.RawControllerData.ConcurrentData);
    }
}

// Called every frame
void UPSMoveComponent::TickComponent( float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction )
{
    Super::TickComponent( DeltaTime, TickType, ThisTickFunction );

    if (FPSMove::IsAvailable())
    {
		FTransform HMDLocalToWorldSpaceTransform= GetHMDLocalToWorldSpaceTransform();
		FTransform HMDTrackingToWorldSpaceTransform = GetHMDTrackingToWorldSpaceTransform();

		// TODO: Check to see if it is necessary to update the DataFrame.RawDataPtr
        //FPSMove::Get().GetRawDataFramePtr(PSMoveID, DataFrame.RawDataPtr);
       
		// Post component driven requests
		DataContext.SetRumbleRequest(this->RumbleRequest);
		DataContext.SetOrientationResetRequest(this->ResetOrientationRequest);
		DataContext.SetCalibrationActionRequest(this->CalibrationActionRequest);

		// Post the HMD tracking space transform to the worker thread if we calibrating
		if (this->CalibrationActionRequest || DataContext.IsCalibrating())
		{
			// This assumes the following:
			// * We know the transform that goes from the "HMD tracking space to world space"
			// * The player camera manager's transform represents the world space transform of the HMD
			// * The ps move controller is physically attached to the HMD and we know the
			//   the offset from the center of the HMD to the center of the PS Move's bulb.
			
			// Given these, we can correlate the PS Move position in HMD tracking space
			// with PS Move positions in PS Move tracking space.
			// With enough samples, we can compute an affine transform (a.k.a. "calibration matrix")
			// that converts points in PS Move tracking space to points in HMD tracking space.

			// The "HMD tracking space to world space" transform is the same in every level
			// so we should only need to compute this calibration matrix once, and then use
			// the "HMD tracking space to world space" transform to put 
			// "PS Move HMD tracking space" positions back into world space
			FTransform HMDLocalSpaceOffset(this->CalibrationOffset);
			FTransform WorldToHMDTrackingSpaceTransform = HMDTrackingToWorldSpaceTransform.Inverse();
			FTransform PSMoveHMDTrackingSpaceTransform = 
				HMDLocalSpaceOffset * HMDLocalToWorldSpaceTransform * WorldToHMDTrackingSpaceTransform;

			DataContext.SetPSMoveHMDTrackingSpacePosition(PSMoveHMDTrackingSpaceTransform.GetLocation());
			DataContext.SetPSMoveHMDTrackingSpaceOrientation(PSMoveHMDTrackingSpaceTransform.GetRotation());
			DataContext.SetHMDIsTracked(UHeadMountedDisplayFunctionLibrary::HasValidTrackingPosition());
		}

		// Post the above data to the worker thread
		// and read the worker thread data the component cares about.
		DataContext.ComponentPostAndRead();

		OnAcknowledgeCalibrationPoint.Broadcast(DataContext.GetCalibrationPointResult());
		OnCalibrationStateChanged.Broadcast(DataContext.GetCalibrationStatus());

		// Convert the PSMove tracking data from HMD tracking space back into world space
		{
			FVector WorldSpaceRawPosition = HMDTrackingToWorldSpaceTransform.TransformPosition(DataContext.GetRawPosition());
			FVector WorldSpaceFilteredPosition = HMDTrackingToWorldSpaceTransform.TransformPosition(DataContext.GetPosition());
			FQuat WorldSpaceFilteredRotation = DataContext.GetRotation() * HMDTrackingToWorldSpaceTransform.GetRotation();
			FVector WorldSpaceFilteredVelocity = HMDTrackingToWorldSpaceTransform.TransformVector(DataContext.GetVelocity());
			FVector WorldSpaceFilteredAcceleration = HMDTrackingToWorldSpaceTransform.TransformVector(DataContext.GetRawAcceleration());

			OnDataUpdated.Broadcast(
				WorldSpaceRawPosition,
				WorldSpaceFilteredPosition,
				WorldSpaceFilteredRotation.Rotator(),
				WorldSpaceFilteredVelocity,
				WorldSpaceFilteredAcceleration);
		}

        OnTriangleButton.Broadcast(DataContext.GetButtonTriangle());
        OnCircleButton.Broadcast(DataContext.GetButtonCircle());
        OnCrossButton.Broadcast(DataContext.GetButtonCross());
        OnSquareButton.Broadcast(DataContext.GetButtonSquare());
        OnSelectButton.Broadcast(DataContext.GetButtonSelect());
        OnStartButton.Broadcast(DataContext.GetButtonStart());
        OnPSButton.Broadcast(DataContext.GetButtonPS());
        OnMoveButton.Broadcast(DataContext.GetButtonMove());
        OnTriggerButton.Broadcast(DataContext.GetTriggerValue());
    }
}

FTransform UPSMoveComponent::GetHMDLocalToWorldSpaceTransform()
{
	UObject* WorldContextObject = this;
	APlayerCameraManager* CameraManager = UGameplayStatics::GetPlayerCameraManager(WorldContextObject, this->PlayerIndex);
	
	return CameraManager->GetTransform();
}

FTransform UPSMoveComponent::GetHMDTrackingToWorldSpaceTransform()
{
	FTransform HMDTrackingToWorldSpaceTransform = CachedHMDTrackingToWorldSpaceTransform;

	if (UHeadMountedDisplayFunctionLibrary::HasValidTrackingPosition())
	{
		FTransform HMDLocalToWorldSpaceTransform = GetHMDLocalToWorldSpaceTransform();
		FVector HMDTrackingWorldSpaceOrigin;
		FRotator HMDTrackingWorldSpaceOrientation;
		UHeadMountedDisplayFunctionLibrary::GetOrientationAndPosition(
			HMDTrackingWorldSpaceOrientation,
			HMDTrackingWorldSpaceOrigin);

		FTransform HMDLocalToTrackingSpaceTransform(HMDTrackingWorldSpaceOrientation, HMDTrackingWorldSpaceOrigin);
		FTransform HMDTrackingToLocalSpaceTransform = HMDLocalToTrackingSpaceTransform.Inverse();
		
		HMDTrackingToWorldSpaceTransform = HMDTrackingToLocalSpaceTransform * HMDLocalToWorldSpaceTransform;
		CachedHMDTrackingToWorldSpaceTransform = HMDTrackingToWorldSpaceTransform;
	}

	return HMDTrackingToWorldSpaceTransform;
}