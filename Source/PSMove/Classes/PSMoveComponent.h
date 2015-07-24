#pragma once

#include "PSMoveTypes.h"
#include "FPSMove.h"
#include "PSMoveComponent.generated.h"

UCLASS(ClassGroup="Input Controller", meta=(BlueprintSpawnableComponent))
class UPSMoveComponent : public USceneComponent//, public IPSMoveInterface
{
    GENERATED_UCLASS_BODY()

public:

    //Sets default values for properties.
    UPSMoveComponent();

    virtual void InitializeComponent() override;

    virtual void TickComponent( float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction ) override;

	// Player Index - 0-based
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = PSMove)
	int32 PlayerIndex;

    // PSMove controller ID - 0-based
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = PSMove)
    int32 PSMoveID;

	// When the PSMove controller is attached to the HMD during calibration.
	// This is the offset of the center of the PSMove bulb from the center of the HMD.
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = PSMove)
	FVector CalibrationOffset;
 
    UPROPERTY()
    FPSMoveDataContext DataContext;

	// Request the orientation of the controller to be reset (assumes controller is pointing down +X axis)
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = PSMove)
	bool ResetOrientationRequest;

	// Request an interation with the calibration state machine.
	// If calibration isn't running, then this starts the calibration process:
	//  - If the configuration file is set to defaults (or non-existent) we go to "Stationary Sampling" state
	//  - If the configuration has been run at least once we go to "HMDSampling" state
	// When calibration is in the "HMDSampling" state, then this action sets a calibration sample point.
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = PSMove)
	bool CalibrationActionRequest;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = PSMove)
    uint8 RumbleRequest;

	// Delegate triggered when the user submits a calibration point.
 	// This is used to trigger acknowledgment FX. 
	// NOTE: 
	// The "result" parameter is true if the given calibration point was valid or not.
	// A calibration point is only valid if the HMD and the PSMove controller are visible to their 
	// respective tracking cameras.
	UPROPERTY(BlueprintAssignable, Category = PSMove)
	FPSMoveAcknowledgeCalibrationPointDelegate OnAcknowledgeCalibrationPoint;

	// Delegate triggered when the calibration state machine changes state.
	// This is used to show/hide calibration objects.
	UPROPERTY(BlueprintAssignable, Category = PSMove)
	FPSMoveCalibrationStateChangeDelegate OnCalibrationStateChanged;

    // Delegate triggered once per frame update
    UPROPERTY(BlueprintAssignable, Category = PSMove)
    FPSMoveDataUpdatedDelegate OnDataUpdated;
    
    // Delegates for buttons. Triggered once per frame if button is down or changes state
    UPROPERTY(BlueprintAssignable, Category = PSMove)
	FPSMoveButtonStateDelegate OnTriangleButton;
    
	UPROPERTY(BlueprintAssignable, Category = PSMove)
	FPSMoveButtonStateDelegate OnCircleButton;   
    
	UPROPERTY(BlueprintAssignable, Category = PSMove)
	FPSMoveButtonStateDelegate OnCrossButton;    
    
	UPROPERTY(BlueprintAssignable, Category = PSMove)
	FPSMoveButtonStateDelegate OnSquareButton;    
    
	UPROPERTY(BlueprintAssignable, Category = PSMove)
	FPSMoveButtonStateDelegate OnSelectButton;    
    
	UPROPERTY(BlueprintAssignable, Category = PSMove)
	FPSMoveButtonStateDelegate OnStartButton;    
    
	UPROPERTY(BlueprintAssignable, Category = PSMove)
	FPSMoveButtonStateDelegate OnPSButton;    
    
	UPROPERTY(BlueprintAssignable, Category = PSMove)
	FPSMoveButtonStateDelegate OnMoveButton;

	UPROPERTY(BlueprintAssignable, Category = PSMove)
    FPSMoveTriggerButtonDelegate OnTriggerButton;

protected:
	FTransform CachedHMDTrackingToWorldSpaceTransform;
	FTransform GetHMDLocalToWorldSpaceTransform();
	FTransform GetHMDTrackingToWorldSpaceTransform();
};