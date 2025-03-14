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

#include <cstring>
#include <cassert>

#include "autodgs.h"
#include "airport.h"

#include "XPWidgets.h"
#include "XPStandardWidgets.h"

#include "XPListBox.h"

typedef struct _widget_ctx
{
    XPWidgetID widget;
    int in_vr;          // currently in vr
    int l, t, w, h;     // last geometry before bringing into vr
} widget_ctx_t;

static widget_ctx_t ui_widget_ctx;

static XPWidgetID ui_widget, list_box, selected_stand,
    marshaller_btn, vdgs_btn;

// current status of ui
static std::string ui_arpt_icao;
static int ui_selected_stand;

static void
show_widget(widget_ctx_t *ctx)
{
    if (XPIsWidgetVisible(ctx->widget))
        return;

    // force window into visible area of screen
    //   we use modern windows under the hut so UI coordinates are in boxels

    // Note that (0,0) is the top left while for widgets it's bottom left
    // so we pass the y* arguments that the outcome is in widget coordinates

    int xl, yl, xr, yr;
    XPLMGetScreenBoundsGlobal(&xl, &yr, &xr, &yl);

    ctx->l = (ctx->l + ctx->w < xr) ? ctx->l : xr - ctx->w - 50;
    ctx->l = (ctx->l <= xl) ? 20 : ctx->l;

    ctx->t = (ctx->t - ctx->h > yl) ? ctx->t : (yr - ctx->h - 50);
    ctx->t = (ctx->t >= ctx->h) ? ctx->t : (yr / 2);

    LogMsg("show_widget: s: (%d, %d) -> (%d, %d), w: (%d, %d) -> (%d,%d)",
           xl, yl, xr, yr, ctx->l, ctx->t, ctx->l + ctx->w, ctx->t - ctx->h);

    XPSetWidgetGeometry(ctx->widget, ctx->l, ctx->t, ctx->l + ctx->w, ctx->t - ctx->h);
    XPShowWidget(ctx->widget);

    int in_vr = XPLMGetDatai(vr_enabled_dr);
    if (in_vr) {
        LogMsg("VR mode detected");
        XPLMWindowID window =  XPGetWidgetUnderlyingWindow(ctx->widget);
        XPLMSetWindowPositioningMode(window, xplm_WindowVR, -1);
        ctx->in_vr = 1;
    } else {
        if (ctx->in_vr) {
            LogMsg("widget now out of VR, map at (%d,%d)", ctx->l, ctx->t);
            XPLMWindowID window =  XPGetWidgetUnderlyingWindow(ctx->widget);
            XPLMSetWindowPositioningMode(window, xplm_WindowPositionFree, -1);

            /* A resize is necessary so it shows up on the main screen again */
            XPSetWidgetGeometry(ctx->widget, ctx->l, ctx->t, ctx->l + ctx->w, ctx->t - ctx->h);
            ctx->in_vr = 0;
        }
    }
}

static void
CloseUI()
{
    XPGetWidgetGeometry(ui_widget_ctx.widget, &ui_widget_ctx.l, &ui_widget_ctx.t, NULL, NULL);
    XPHideWidget(ui_widget_ctx.widget);
}

static int
UIWidgetCb(XPWidgetMessage msg, XPWidgetID widget_id, intptr_t param1, intptr_t param2)
{
    if (msg == xpMessage_CloseButtonPushed) {
        CloseUI();
        return 1;
    }

	if (msg == xpMessage_ListBoxItemSelected) {
        ui_selected_stand = XPGetWidgetProperty(list_box, xpProperty_ListBoxCurrentItem, NULL);
        assert(ui_selected_stand >= 0); // be paranoid
        ui_selected_stand--;            // 0 is "Automatic"

        char ss_name[100];
        ss_name[99] = '\0';
		XPGetWidgetDescriptor(list_box, ss_name, sizeof(ss_name) - 1);
        std::string txt{ss_name + 2};   // skip type, blank
        txt += " @ " + (ui_arpt_icao.empty() ? "unknown" : ui_arpt_icao);
		XPSetWidgetDescriptor(selected_stand, txt.c_str());
        LogMsg("selected ramp is '%s'", ss_name);
        if (arpt) {
            arpt->SetSelectedStand(ui_selected_stand);
            int dgs_type = arpt->GetDgsType();
            XPSetWidgetProperty(marshaller_btn, xpProperty_ButtonState, 0);
            XPSetWidgetProperty(vdgs_btn, xpProperty_ButtonState, 0);
            if (dgs_type == kMarshaller)
                XPSetWidgetProperty(marshaller_btn, xpProperty_ButtonState, 1);
            else
                XPSetWidgetProperty(vdgs_btn, xpProperty_ButtonState, 1);
        }

        return 1;
    }

    if (msg != xpMsg_ButtonStateChanged)
        return 0;

    // From here on the radio buttons only

    // radio buttons get this message only when they are clicked to "selected"
    int dgs_type = -1;
    if (widget_id == marshaller_btn) {
        dgs_type = kMarshaller;
        XPSetWidgetProperty(vdgs_btn, xpProperty_ButtonState, 0);
    } else if (widget_id == vdgs_btn) {
        dgs_type = kVDGS;
        XPSetWidgetProperty(marshaller_btn, xpProperty_ButtonState, 0);
    }

    assert(dgs_type != -1);

    if (arpt) {
        arpt->SetDgsType(dgs_type);
        int selected_stand = XPGetWidgetProperty(list_box, xpProperty_ListBoxCurrentItem, NULL);
        selected_stand--;            // 0 is "Automatic"
        if (selected_stand >= 0) {
            XPSetWidgetProperty(list_box, xpProperty_ListBoxDeleteItem, 1);
            auto [dgs_type, name] = arpt->GetStand(selected_stand);
            char entry[100];
            snprintf(entry, sizeof(entry), "%c %s", (dgs_type == kMarshaller ? 'M' : 'V'), name.c_str());
            XPSetWidgetDescriptor(list_box, entry);
            XPSetWidgetProperty(list_box, xpProperty_ListBoxInsertItem, 1);
        }
    }

    return 1;
}

void
UpdateUI(bool only_if_visible)
{
    if (ui_widget == NULL || (only_if_visible && !XPIsWidgetVisible(ui_widget))) {
        LogMsg("update_ui: widget is not visible");
        return;
    }

    LogMsg("update_ui started");

    if (arpt == nullptr) {
        if (!ui_arpt_icao.empty()) {
            ui_selected_stand = -1;
            XPSetWidgetDescriptor(list_box, "Automatic");
            XPSetWidgetProperty(list_box, xpProperty_ListBoxAddItemsWithClear, 1);
            ui_arpt_icao.clear();
        }
    } else if (arpt->name() != ui_arpt_icao) {
        LogMsg("airport changed to %s", arpt->name().c_str());
        ui_arpt_icao = arpt->name();

        LogMsg("load ramps");
        ui_selected_stand = -1;
        XPSetWidgetDescriptor(list_box, "Automatic");
        XPSetWidgetProperty(list_box, xpProperty_ListBoxAddItemsWithClear, 1);

        int n = arpt->nstands();
        for (int i = 0; i < n; i++) {
            auto [dgs_type, name] = arpt->GetStand(i);
            char entry[100];
            snprintf(entry, sizeof(entry), "%c %s", (dgs_type == kMarshaller ? 'M' : 'V'), name.c_str());
            XPSetWidgetDescriptor(list_box, entry);
            XPSetWidgetProperty(list_box, xpProperty_ListBoxAddItem, 1);
        }
    }

    // that's cheap so we do it always
    if (arpt == nullptr)
        XPSetWidgetDescriptor(selected_stand, "inactive");
    else {
        if (ui_selected_stand == -1) {
            XPSetWidgetProperty(marshaller_btn, xpProperty_ButtonState, 1);
            XPSetWidgetProperty(vdgs_btn, xpProperty_ButtonState, 0);
            std::string txt = "Automatic @ " + ui_arpt_icao;
            XPSetWidgetDescriptor(selected_stand, txt.c_str());
        } else {
            auto [dgs_type, name] = arpt->GetStand(ui_selected_stand);
            XPSetWidgetProperty(marshaller_btn, xpProperty_ButtonState, (dgs_type == kMarshaller ? 1 : 0));
            XPSetWidgetProperty(vdgs_btn, xpProperty_ButtonState, (dgs_type == kVDGS ? 1 : 0));
            std::string txt = name + " @ " + ui_arpt_icao;
            XPSetWidgetDescriptor(selected_stand, txt.c_str());
        }
    }

    LogMsg("update_ui finished");
}

static void
CreateUI()
{
    // Note that (0,0) is the top left while for widgets it's bottom left
    // so we pass the y* arguments that the outcome is in widget coordinates

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
    XPAddWidgetCallback(ui_widget, UIWidgetCb);
    left += 5;
    left1 = left + 60;

    top -= 20;
    XPCreateWidget(left, top, left + 50, top - 20,
                                 1, "Marshaller", 0, ui_widget, xpWidgetClass_Caption);
    marshaller_btn = XPCreateWidget(left1, top, left1 + 20, top - 20,
                                    1, "", 0, ui_widget, xpWidgetClass_Button);
    XPSetWidgetProperty(marshaller_btn, xpProperty_ButtonType, xpRadioButton);
    XPSetWidgetProperty(marshaller_btn, xpProperty_ButtonBehavior, xpButtonBehaviorRadioButton);
    XPAddWidgetCallback(marshaller_btn, UIWidgetCb);
    XPSetWidgetProperty(marshaller_btn, xpProperty_ButtonState, 1);

    top -= 20;
    XPCreateWidget(left, top, left + 50, top - 20,
                                 1, "VDGS", 0, ui_widget, xpWidgetClass_Caption);
    vdgs_btn = XPCreateWidget(left1, top, left1 + 20, top - 20,
                              1, "", 0, ui_widget, xpWidgetClass_Button);
    XPSetWidgetProperty(vdgs_btn, xpProperty_ButtonType, xpRadioButton);
    XPSetWidgetProperty(vdgs_btn, xpProperty_ButtonBehavior, xpButtonBehaviorRadioButton);
    XPAddWidgetCallback(vdgs_btn, UIWidgetCb);

    top -= 20;
    selected_stand = XPCreateWidget(left, top, left + width - 30, top - 20,
                                    1, "", 0, ui_widget, xpWidgetClass_Caption);

    top -= 30;
    list_box = XPCreateListBox(left, top, left + width - 10, top - lb_height,
                               1, "Automatic", ui_widget);
}

void
ToggleUI(void) {
    LogMsg("toggle_ui called");

    if (ui_widget == NULL)
        CreateUI();

    if (XPIsWidgetVisible(ui_widget)) {
        CloseUI();
        return;
    }

    UpdateUI(false);
    show_widget(&ui_widget_ctx);
}
