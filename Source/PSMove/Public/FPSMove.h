#pragma once

#include "ModuleManager.h"

DEFINE_LOG_CATEGORY_STATIC(LogPSMove, Log, All);

/**
 * The public interface to this module.
 */

class FPSMove : public IModuleInterface //, public IModularFeature?
{
public:
    virtual void StartupModule();
    virtual void ShutdownModule();
    
    /**
     * Singleton-like access to this module's interface.  This is just for convenience!
     * Beware of calling this during the shutdown phase, though.  Your module might have been unloaded already.
     *
     * @return Returns singleton instance, loading the module on demand if needed
     */
    static inline FPSMove& Get()
    {
        return FModuleManager::LoadModuleChecked< FPSMove >( "PSMove" );
    }
    
    /**
     * Checks to see if this module is loaded and ready.  It is only valid to call Get() if IsAvailable() returns true.
     *
     * @return True if the module is loaded and ready to use
     */
    static inline bool IsAvailable()
    {
        return FModuleManager::Get().IsModuleLoaded( "PSMove" );
    }

	/**
	* A pointer to the data shared amongst all controllers (i.e. calibration state).
	*/
	FPSMoveRawSharedData_Concurrent* ModuleRawSharedDataPtr;

    /**
     * A pointer to an array of raw data frames, one for each connected controller.
     */
    TArray<FPSMoveRawControllerData_Concurrent>* ModuleRawControllerDataArrayPtr;
    
    /**
     * Here we declare functions that will be accessed via the module instance from within the game.
     */
    virtual void InitWorker();

	/**
	* A component passes in a (typically null) pointer to a raw shared data object.
	* This updates the pointer so it points to the same place as the module's raw shared data pointer
	* i.e., it now points to the Worker's raw shared data.
	*/
	virtual void  GetRawSharedDataPtr(FPSMoveRawSharedData_Concurrent* &RawSharedDataPtrOut);

    /**
     * A component passes in a (typically null) pointer to a raw controller data object.
     * This updates the pointer so it points to the same place as the module's raw data frame pointer for this ID
     * i.e., it now points to the Worker's raw data frame.
     */
    virtual void  GetRawControllerDataPtr(uint8 PSMoveID, FPSMoveRawControllerData_Concurrent* &RawControllerDataPtrOut);
};