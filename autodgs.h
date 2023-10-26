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
extern XPLMDataRef ref_vr_enabled;

extern const airport_t *arpt;
extern int dgs_type;
extern void toggle_ui(void);
extern void update_ui(int only_if_visible);

extern void set_selected_ramp(const char *ui_sr);
extern void set_dgs_type(int new_dgs_type);