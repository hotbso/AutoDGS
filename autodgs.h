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

#define XPLM200
#define XPLM210
#define XPLM300
#define XPLM301

#include "XPLMPlugin.h"
#include "XPLMProcessing.h"
#include "XPLMDataAccess.h"
#include "XPLMMenus.h"
#include "XPLMGraphics.h"
#include "XPLMInstance.h"
#include "XPLMNavigation.h"
#include "XPLMUtilities.h"
#include "XPLMPlanes.h"
#include "XPWidgets.h"
#include "XPStandardWidgets.h"

#include <acfutils/assert.h>
#include <acfutils/airportdb.h>

#define RAMP_NAME_LEN sizeof(((ramp_start_t *)0)->name)

extern XPLMCommandRef cycle_dgs_cmdr;
extern XPLMDataRef vr_enabled_dr;

extern const airport_t *arpt;
extern int dgs_type;
extern void toggle_ui(void);
extern void update_ui(int only_if_visible);

extern void set_selected_ramp(const char *ui_sr);
extern void set_dgs_type(int new_dgs_type);
