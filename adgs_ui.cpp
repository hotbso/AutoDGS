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
#include <XPListBox.h>

typedef struct _widget_ctx
{
    XPWidgetID widget;
    int in_vr;          /* currently in vr */
    int l, t, w, h;     /* last geometry before bringing into vr */
} widget_ctx_t;

static widget_ctx_t ui_widget_ctx;

static XPWidgetID ui_widget, list_box, selected_stand, marshaller_btn, vdgs_btn;

/* current status of ui */
static const airport_t *ui_arpt;
static char ui_arpt_icao[AIRPORTDB_ICAO_LEN];
static int ui_dgs_type;
static char ui_selected_ramp[RAMP_NAME_LEN];

static void
show_widget(widget_ctx_t *ctx)
{
    if (XPIsWidgetVisible(ctx->widget))
        return;

    /* force window into visible area of screen
       we use modern windows under the hut so UI coordinates are in boxels */

    /* Note that (0,0) is the top left while for widgets it's bottom left
     * so we pass the y* arguments that the outcome is in widget coordinates */

    int xl, yl, xr, yr;
    XPLMGetScreenBoundsGlobal(&xl, &yr, &xr, &yl);

    ctx->l = (ctx->l + ctx->w < xr) ? ctx->l : xr - ctx->w - 50;
    ctx->l = (ctx->l <= xl) ? 20 : ctx->l;

    ctx->t = (ctx->t - ctx->h > yl) ? ctx->t : (yr - ctx->h - 50);
    ctx->t = (ctx->t >= ctx->h) ? ctx->t : (yr / 2);

    logMsg("show_widget: s: (%d, %d) -> (%d, %d), w: (%d, %d) -> (%d,%d)",
           xl, yl, xr, yr, ctx->l, ctx->t, ctx->l + ctx->w, ctx->t - ctx->h);

    XPSetWidgetGeometry(ctx->widget, ctx->l, ctx->t, ctx->l + ctx->w, ctx->t - ctx->h);
    XPShowWidget(ctx->widget);

    int in_vr = XPLMGetDatai(vr_enabled_dr);
    if (in_vr) {
        logMsg("VR mode detected");
        XPLMWindowID window =  XPGetWidgetUnderlyingWindow(ctx->widget);
        XPLMSetWindowPositioningMode(window, xplm_WindowVR, -1);
        ctx->in_vr = 1;
    } else {
        if (ctx->in_vr) {
            logMsg("widget now out of VR, map at (%d,%d)", ctx->l, ctx->t);
            XPLMWindowID window =  XPGetWidgetUnderlyingWindow(ctx->widget);
            XPLMSetWindowPositioningMode(window, xplm_WindowPositionFree, -1);

            /* A resize is necessary so it shows up on the main screen again */
            XPSetWidgetGeometry(ctx->widget, ctx->l, ctx->t, ctx->l + ctx->w, ctx->t - ctx->h);
            ctx->in_vr = 0;
        }
    }
}

static void
close_ui()
{
    XPGetWidgetGeometry(ui_widget_ctx.widget, &ui_widget_ctx.l, &ui_widget_ctx.t, NULL, NULL);
    XPHideWidget(ui_widget_ctx.widget);
}

static int
ui_widget_cb(XPWidgetMessage msg, XPWidgetID widget_id, intptr_t param1, intptr_t param2)
{
    if (msg == xpMessage_CloseButtonPushed) {
        close_ui();
        return 1;
    }

	if (msg == xpMessage_ListBoxItemSelected) {
        char buffer[100];
		XPGetWidgetDescriptor(list_box, ui_selected_ramp, sizeof(ui_selected_ramp));
        ui_selected_ramp[RAMP_NAME_LEN - 1] = '\0';
        snprintf(buffer, sizeof buffer, "%s @ %s",
                 ui_selected_ramp, ui_arpt_icao[0] ? ui_arpt_icao : "unknown");
		XPSetWidgetDescriptor(selected_stand, buffer);
        logMsg("selected ramp is '%s'", ui_selected_ramp);
        set_selected_ramp(ui_selected_ramp);
        return 1;
    }

    if ((widget_id == marshaller_btn) && (msg == xpMsg_ButtonStateChanged)) {
        int state = (int)param2;
        XPSetWidgetProperty(vdgs_btn, xpProperty_ButtonState, !state);
        ui_dgs_type = 0;
        set_dgs_type(ui_dgs_type);
        return 1;
    }

    if ((widget_id == vdgs_btn) && (msg == xpMsg_ButtonStateChanged)) {
        int state = (int)param2;
        XPSetWidgetProperty(marshaller_btn, xpProperty_ButtonState, !state);
        ui_dgs_type = 1;
        set_dgs_type(ui_dgs_type);
        return 1;
    }

    return 0;
}

void
update_ui(int only_if_visible)
{
    if (ui_widget == NULL || (only_if_visible && !XPIsWidgetVisible(ui_widget))) {
        logMsg("update_ui: widget is not visible");
        return;
    }

    logMsg("update_ui started");

    if (ui_dgs_type != dgs_type) {
        XPSetWidgetProperty(marshaller_btn, xpProperty_ButtonState, !dgs_type);
        XPSetWidgetProperty(vdgs_btn, xpProperty_ButtonState, dgs_type);
        ui_dgs_type = dgs_type;
    }

    if (arpt == NULL) {
        if (ui_arpt != NULL) {
            logMsg("arpt changed to %p", arpt);
            strcpy(ui_selected_ramp, "Automatic");
            XPSetWidgetDescriptor(list_box, ui_selected_ramp);
            XPSetWidgetProperty(list_box, xpProperty_ListBoxAddItemsWithClear, 1);
            ui_arpt = NULL;
            ui_arpt_icao[0] = '\0';
        }
    } else if (0 != strcmp(arpt->icao, ui_arpt_icao)) {
        logMsg("airport changed to %s", arpt->icao);
        strcpy(ui_arpt_icao, arpt->icao);
        ui_arpt = arpt;

        logMsg("load ramps");
        strcpy(ui_selected_ramp, "Automatic");
        XPSetWidgetDescriptor(list_box, ui_selected_ramp);
        XPSetWidgetProperty(list_box, xpProperty_ListBoxAddItemsWithClear, 1);

        for (const ramp_start_t *ramp = (const ramp_start_t *)avl_first(&arpt->ramp_starts); ramp != NULL;
            ramp = (const ramp_start_t *)AVL_NEXT(&arpt->ramp_starts, ramp)) {
            XPSetWidgetDescriptor(list_box, ramp->name);
            XPSetWidgetProperty(list_box, xpProperty_ListBoxAddItem, 1);
        }
    }

    // that's cheap so we do it always
    if (ui_arpt == NULL)
        XPSetWidgetDescriptor(selected_stand, "inactive");
    else {
        char buffer[100];
        snprintf(buffer, sizeof buffer, "%s @ %s",
                 ui_selected_ramp, ui_arpt_icao[0] ? ui_arpt_icao : "unknown");
        XPSetWidgetDescriptor(selected_stand, buffer);
    }

    logMsg("update_ui finished");
}

static void
create_ui()
{
    /* Note that (0,0) is the top left while for widgets it's bottom left
     * so we pass the y* arguments that the outcome is in widget coordinates */

    int xl, yr;
    XPLMGetScreenBoundsGlobal(&xl, &yr, NULL, NULL);

    int left = xl + 50;
    int top = yr - 100;
    int width = 250;
    int height = 450;
    int lb_height = 350;
    int left1;

    ui_widget_ctx.l = left;
    ui_widget_ctx.t = top;
    ui_widget_ctx.w = width;
    ui_widget_ctx.h = height;

    ui_widget = XPCreateWidget(left, top, left + width, top - height,
                               0, "AutoDGS " VERSION, 1, NULL, xpWidgetClass_MainWindow);
    ui_widget_ctx.widget = ui_widget;

    XPSetWidgetProperty(ui_widget, xpProperty_MainWindowHasCloseBoxes, 1);
    XPAddWidgetCallback(ui_widget, ui_widget_cb);
    left += 5;

    left1 = left + 60;
    top -= 20;
    XPCreateWidget(left, top, left + 50, top - 20,
                                 1, "Marshaller", 0, ui_widget, xpWidgetClass_Caption);
    marshaller_btn = XPCreateWidget(left1, top, left1 + 20, top - 20,
                                    1, "", 0, ui_widget, xpWidgetClass_Button);
    XPSetWidgetProperty(marshaller_btn, xpProperty_ButtonType, xpRadioButton);
    XPSetWidgetProperty(marshaller_btn, xpProperty_ButtonBehavior, xpButtonBehaviorRadioButton);
    XPAddWidgetCallback(marshaller_btn, ui_widget_cb);
    XPSetWidgetProperty(marshaller_btn, xpProperty_ButtonState, 1);

    top -= 20;
    XPCreateWidget(left, top, left + 50, top - 20,
                                 1, "VDGS", 0, ui_widget, xpWidgetClass_Caption);
    vdgs_btn = XPCreateWidget(left1, top, left1 + 20, top - 20,
                              1, "", 0, ui_widget, xpWidgetClass_Button);
    XPSetWidgetProperty(vdgs_btn, xpProperty_ButtonType, xpRadioButton);
    XPSetWidgetProperty(vdgs_btn, xpProperty_ButtonBehavior, xpButtonBehaviorRadioButton);
    XPAddWidgetCallback(vdgs_btn, ui_widget_cb);

    top -= 20;
    selected_stand = XPCreateWidget(left, top, left + width - 30, top - 20,
                                    1, "", 0, ui_widget, xpWidgetClass_Caption);

    top -= 30;
    list_box = XPCreateListBox(left, top, left + width - 10, top - lb_height,
                               1, "Automatic", ui_widget);
}

void
toggle_ui(void) {
    logMsg("toggle_ui called");

    if (ui_widget == NULL)
        create_ui();

    if (XPIsWidgetVisible(ui_widget)) {
        close_ui();
        return;
    }

    update_ui(0);
    show_widget(&ui_widget_ctx);
}
