-- autodgs_activate_on_taxi_light_off.lua
--
-- Purpose:
-- Activate AutoDGS when a taxi_light on/off transition is detected
--
-- Sample code how to activate AutoDGS depending on aircraft state.
-- This code is neither meant to be universal nor reliable.
-- It works with the standard C172, whether it works with your favorite airliner
-- is a different story.
--

dataref("adgs_operation_mode_dr", "AutoDGS/operation_mode", "writeable")
dataref("taxi_light_on_dr", "sim/cockpit/electrical/taxi_light_on")

local taxi_light    -- current state

-- called by a (possibly high frequency) loop, careful coding required
function adgs_detect_tl_off()
    local tl = taxi_light_on_dr

    -- detect on/off transition
    if tl == 0 and taxi_light == 1 then
        command_once("AutoDGS/activate")
    end

    taxi_light = tl
end

-- set AutoDGS plugin to manual mode
logMsg("set AutoDGS mode to manual")
adgs_operation_mode_dr = 1

-- init state
taxi_light = taxi_light_on_dr

-- run once per second
do_often("adgs_detect_tl_off()")