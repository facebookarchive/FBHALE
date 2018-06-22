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
  
function [aircraft, freeStream] = SLF(aircraft, freeStream, mission, environment)
% This function computes the trim and power required to accomplish steady
% level flight (SLF) at the current altitude and aero mode.

% Compute CL required for minimum power at steady level flight (SLF). If it
% is greater than that corresponding to minimum speed then use this speed.
freeStream.SLF = setfield(freeStream, 'gamma_rad', 0);  % SLF condition
freeStream.SLF.dVdt_ms2 = 0;                            % SLF condition

if ~isfield(freeStream, 'windSpeed_ms')
    freeStream.windSpeed_ms  = 0;
end
if ~isfield(freeStream, 'VTAS_ms')
    freeStream.VTAS_ms     = freeStream.windSpeed_ms;
    freeStream.SLF.VTAS_ms = freeStream.windSpeed_ms;
end
if ~isfield(freeStream, 'dVdt_ms2')
    freeStream.dVdt_ms2 = 0;
end

% If a wind heading was provided, compute the wind speed in the
% actual aircraft track direction
if isfield(freeStream, 'windHeading_rad')
    groundTrackVect = [cos(aircraft.states.groundTrackAngle_rad); sin(aircraft.states.groundTrackAngle_rad)];
    Vwind_ms        = freeStream.windSpeed_ms * [cos(freeStream.windHeading_rad); sin(freeStream.windHeading_rad)];
    minVTAS_ms      = dot(Vwind_ms, groundTrackVect);
else
    minVTAS_ms      = freeStream.windSpeed_ms;
end

aircraft.aero.mode = 'minPower';
aero0  = Polar(aircraft.aero, freeStream.SLF, aircraft, mission);

CLminV = aircraft.massProperties.MGTOW_N/(1/2 * freeStream.SLF.rho_kgm3 * minVTAS_ms^2 * aircraft.aero.Sref_m2);
if CLminV < aero0.CL
    aircraft.aero.mode = 'specifiedV';
    freeStream.VTAS_ms = minVTAS_ms;
    freeStream.SLF.VTAS_ms = minVTAS_ms;
end
[aircraft.SLF, freeStream.SLF] = NetPower(aircraft, freeStream.SLF, mission, environment);

end
