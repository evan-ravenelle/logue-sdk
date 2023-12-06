/**
 *  @file header.c
 *  @brief drumlogue SDK unit header
 *
 *  Copyright (c) 2020-2022 KORG Inc. All rights reserved.
 *
 */

#include "unit.h"  // Note: Include common definitions for all units

// ---- Unit header definition  --------------------------------------------------------------------

const __unit_header unit_header_t unit_header = {
    .header_size = sizeof(unit_header_t),                     // leave as is, size of this header
    .target = UNIT_TARGET_PLATFORM  | k_unit_module_masterfx,  // target platform and module for this unit
    .api = UNIT_API_VERSION,                                  // logue sdk API version against which unit was built
    .dev_id = 0x65726176U,                                           // developer id
    .unit_id = 0x0000001U,                                          // Id for this unit, should be unique within the scope of a given dev_id
    .version = 0x00010000U,                                   // This unit's version: major.minor.patch (major<<16 minor<<8 patch).
    .name = "CompDrive",                                      // Name for this unit, will be displayed on device
    .num_presets = 0,                                         // Number of internal presets this unit has
    .num_params = 16,                                          // Number of parameters for this unit, max 24
    .params = {
        // Format: min, max, center, default, type, fractional, frac. type, <reserved>, name

        // See common/runtime.h for type enum and unit_param_t structure

// Page 1
    {5, (125 << 3), 0, 30, k_unit_param_type_msec , 0, 0, 0, {"Attack"}},
    {10, (125 << 3), 0, 300, k_unit_param_type_msec , 0, 0, 0, {"Release"}},
    {-128, 0, 0, 0, k_unit_param_type_db, 1, 0, 0, {"THold"}},
    {1, 30, 0, 1, k_unit_param_type_none, 0, 0, 0, {"Ratio"}},
    
    // Page 2
    {0, 0, 0, 0, k_unit_param_type_bitmaps, 0, 0, 0, {"RMS"}},
    {0, 24, 0, 0, k_unit_param_type_db, 1, 0, 0, {"Knee"}},
    {0, 24, 0, 0, k_unit_param_type_db, 0, 0, 0, {"Clipping"}},
    {0, 24, 0, 0, k_unit_param_type_db, 1, 0, 0, {"Makeup"}},

   
    // Page 3
    {-100, 100, 0, 0, k_unit_param_type_drywet, 0, 0, 0, {"Dry/Wet"}},
    {0, 36, 0, 0, k_unit_param_type_db, 1, 0, 0, {"PreGain"}},
    {0, 1, 0, 0, k_unit_param_type_onoff, 0, 0, 0, {"Sidechain"}},
    {0, 0, 0, 0, k_unit_param_type_none, 0, 0, 0, {""}},
    
    
    // Page 4 Noise Gate
    {-128, 0, 0, 0, k_unit_param_type_db, 1, 0, 0, {"NThold"}},
    {0, (125 << 3), 0, 200, k_unit_param_type_msec, 0, 0, 0, {"NRelease"}},
    {0, 1, 0, 0, k_unit_param_type_onoff, 0, 0, 0, {"NGate"}},
    {0, 0, 0, 0, k_unit_param_type_none, 0, 0, 0, {""}},
    
    // Page 5
    {0, 0, 0, 0, k_unit_param_type_none, 1, 0, 0, {""}},
    {0, 0, 0, 0, k_unit_param_type_none, 1, 0, 0, {""}},
    {0, 0, 0, 0, k_unit_param_type_none, 0, 0, 0, {""}},
    {0, 0, 0, 0, k_unit_param_type_none, 0, 0, 0, {""}},
    
    // Page 6
    {0, 0, 0, 0, k_unit_param_type_none, 1, 0, 0, {""}},
    {0, 0, 0, 0, k_unit_param_type_none, 1, 0, 0, {""}},
    {0, 0, 0, 0, k_unit_param_type_none, 1, 0, 0, {""}},
    {0, 0, 0, 0, k_unit_param_type_none, 1, 0, 0, {""}},
        
        }};
