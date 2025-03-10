//
//    AutoDGS: Show Marshaller or VDGS at default airports
//
//    Copyright (C) 2006-2013 Jonathan Harris
//    Copyright (C) 2023, 2025 Holger Teutsch
//
//    This library is free software; you can redistribute it and/or
//    modify it under the terms of the GNU Lesser General Public
//    License as published by the Free Software Foundation; either
//    version 2.1 of the License, or (at your option) any later version.
//
//    This library is distributed in the hope that it will be useful,
//    but WITHOUT ANY WARRANTY; without even the implied warranty of
//    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
//    Lesser General Public License for more details.
//
//    You should have received a copy of the GNU Lesser General Public
//    License along with this library; if not, write to the Free Software
//    Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301
//    USA
//

#include "autodgs.h"

enum {
    API_OPERATION_MODE,
    API_STATE,
    API_ON_GROUND,
    API_DGS_RAMP_DIST_DEFAULT,

    API_STATE_STR,  // convenience drefs
    API_OPERATION_MODE_STR,
    API_RAMP
};

// API accessor routines
static int
api_getint(XPLMDataRef ref)
{
    switch ((long long)ref) {
        case API_STATE:
            return state;
        case API_OPERATION_MODE:
            return operation_mode;
        case API_ON_GROUND:
            return on_ground;
    }

    return 0;
}

static float
api_getfloat(XPLMDataRef ref)
{
    switch ((long long)ref) {
        case API_DGS_RAMP_DIST_DEFAULT:
            return dgs_ramp_dist_default;
    }

    return 0;
}

static void
api_setint(XPLMDataRef ref, int val)
{
    switch ((long long)ref) {
        case API_OPERATION_MODE:
            ; // required by some gcc versions
            opmode_t mode = (opmode_t)val;
            if (mode != MODE_AUTO && mode != MODE_MANUAL) {
                LogMsg("API: trying to set invalid operation_mode %d, ignored", val);
                return;
            }

            if (mode == operation_mode) // Lua hammers writeable drefs in a frame loop
                return;

            LogMsg("API: operation_mode set to %s", opmode_str[mode]);
            operation_mode = mode;
            break;
    }
}

static void
api_setfloat(XPLMDataRef ref, float val)
{
    switch ((long long)ref) {
        case API_DGS_RAMP_DIST_DEFAULT:
            if (val == dgs_ramp_dist_default) // Lua hammers writeable drefs in a frame loop
                return;

            dgs_ramp_dist_default = val;
            dgs_ramp_dist_override = 1;
            LogMsg("API: dgs_ramp_dist_default set to %0.1f", dgs_ramp_dist_default);
            break;
    }
}

static int
api_getbytes(XPLMDataRef ref, void *out, int ofs, int n)
{
    static const int buflen = 34;
    char buf[buflen];

    if (out == NULL)
        return buflen;

    if (n <= 0 || ofs < 0 || ofs >= buflen)
        return 0;

    if (n + ofs > buflen)
        n = buflen - ofs;

    memset(buf, 0, buflen);

    switch ((long long)ref) {
        case API_STATE_STR:
            strncpy(buf, state_str[state], buflen - 1);
            break;

        case API_OPERATION_MODE_STR:
            strncpy(buf, opmode_str[operation_mode], buflen - 1);
            break;

        case API_RAMP:
            if (state >= ENGAGED)
                strncpy(buf, nearest_ramp->name.c_str(), buflen - 1);
            break;
    }

    memcpy(out, buf, n);
    return n;
}

void
create_api_drefs()
{
        // API datarefs
    XPLMRegisterDataAccessor("AutoDGS/operation_mode", xplmType_Int, 1, api_getint, api_setint, NULL,
                             NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL,
                             (void *)API_OPERATION_MODE, (void *)API_OPERATION_MODE);

    XPLMRegisterDataAccessor("AutoDGS/operation_mode_str", xplmType_Data, 0, NULL, NULL, NULL,
                             NULL, NULL, NULL, NULL, NULL, NULL, NULL, api_getbytes, NULL,
                             (void *)API_OPERATION_MODE_STR, NULL);

    XPLMRegisterDataAccessor("AutoDGS/state", xplmType_Int, 0, api_getint, NULL, NULL,
                             NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL,
                             (void *)API_STATE, NULL);

    XPLMRegisterDataAccessor("AutoDGS/on_ground", xplmType_Int, 0, api_getint, NULL, NULL,
                             NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL,
                             (void *)API_ON_GROUND, NULL);

    XPLMRegisterDataAccessor("AutoDGS/dgs_ramp_dist_default", xplmType_Float, 1, NULL, NULL,
                             api_getfloat, api_setfloat,
                             NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL,
                             (void *)API_DGS_RAMP_DIST_DEFAULT, (void *)API_DGS_RAMP_DIST_DEFAULT);

    XPLMRegisterDataAccessor("AutoDGS/state_str", xplmType_Data, 0, NULL, NULL, NULL,
                             NULL, NULL, NULL, NULL, NULL, NULL, NULL, api_getbytes, NULL,
                             (void *)API_STATE_STR, NULL);

    XPLMRegisterDataAccessor("AutoDGS/ramp", xplmType_Data, 0, NULL, NULL, NULL,
                             NULL, NULL, NULL, NULL, NULL, NULL, NULL, api_getbytes, NULL,
                             (void *)API_RAMP, NULL);
}
