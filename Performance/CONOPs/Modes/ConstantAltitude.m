%{
For FBHALE software
Copyright (C) 2018-present Facebook, Inc.

This program is free software; you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation; either version 2 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License along
with this program; if not, write to the Free Software Foundation, Inc.,
51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
%}
  
function [aircraft, freeStream, dt_s] = ConstantAltitude(aircraft, freeStream, mission, environment)
% This function attempts at keeping altitude constant. This may be called
% when the aircraft is operating at maximum altitude and still receiving
% power or when it is at minimum altitude.

% Use extra power to accelerate flag
useExtraSolarPower = false;

freeStream.gamma_rad = 0;

% Drop time integration step
dt_s = mission.dt_s;

if useExtraSolarPower

    % 1. Prediction of new speed
    freeStreamPred     = freeStream;
    aircraft.aero.mode = 'specifiedV';
    [aircraftSLF, freeStreamPred] = NetPower(aircraft, freeStreamPred, mission, environment);
    aircraft.SLF = aircraftSLF;

    % Check how far from satisfied is the power condition
    PeSLF_W = aircraft.SLF.batteries.Pe_W;
    Pk_W    = -PeSLF_W * aircraft.propulsion.propulsor(1).propeller.efficiency *  aircraft.propulsion.propulsor(1).motor.eta * aircraft.propulsion.propulsor(1).motor.controller_eta;
    freeStreamPred.dVdt0_ms2= Pk_W / aircraft.massProperties.MGTOW_kg / freeStream.VTAS_ms;
    freeStreamPred.V_ms= freeStreamPred.VTAS_ms + freeStreamPred.dVdt0_ms2*dt_s;

    % 2. Correction
    [aircraftSLF, freeStreamPred] = NetPower(aircraft, freeStreamPred, mission, environment);
    aircraft.SLF = aircraftSLF;
    PeSLF_W  = aircraft.SLF.batteries.Pe_W;
    Pk_W     = -PeSLF_W * aircraft.propulsion.propulsor(1).propeller.efficiency *  aircraft.propulsion.propulsor(1).motor.eta * aircraft.propulsion.propulsor(1).motor.controller_eta;
    dVdt1_ms2= Pk_W / aircraft.massProperties.MGTOW_kg / freeStreamPred.VTAS_ms;
    freeStream.dVdt_ms2 = (dVdt1_ms2 + freeStreamPred.dVdt_ms2)/2;
    freeStream.VTAS_ms  = freeStream.VTAS_ms + freeStream.dVdt_ms2*dt_s;

    % 3. Limit to minV if applicable
    if freeStream.VTAS_ms < freeStream.windSpeed_ms
        freeStream.VTAS_ms = freeStream.windSpeed_ms;
        freeStream.dVdt_ms2 = 0;
    end

    % 4. Make sure we also don't slow past the min Power speed which could
    % result in NaNs
    if isnan(freeStream.VTAS_ms) || freeStream.VTAS_ms < freeStream.SLF.VTAS_ms 
        freeStream.VTAS_ms = freeStream.SLF.VTAS_ms;
        freeStream.dVdt_ms2 = 0;
    end

else
    freeStream.VTAS_ms = freeStream.SLF.VTAS_ms;
    freeStream.dVdt_ms2 = 0;
    aircraft = aircraft.SLF;
end

% Now that we have the correct speed, recompute and move on
[aircraft, freeStream] = NetPower(aircraft, freeStream, mission, environment);

end
