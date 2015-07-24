// Copyright 1998-2015 Epic Games, Inc. All Rights Reserved.

    // You should place include statements to your module's private header files here.  You only need to
    // add includes for headers that are used in most of your module's source files though.

#include "Engine.h"
#include "CoreUObject.h"

#define USING_STATIC_LIBRARY

#include "../../ThirdParty/psmoveapi/include/psmove.h"
#include "../../ThirdParty/psmoveapi/include/psmove_tracker.h"
#include "../../ThirdParty/psmoveapi/include/psmove_fusion.h"

#include "PSMoveTypes.h"