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
  
function aero = Polar(aero, freeStream, aircraft, mission)
% This function looks-up the integrated forces (lift, drag) on the aircraft
% given a polar. Depending on the mode (flying minimum power, flying at a
% specified speed, etc...), the Reynolds number is computed as well as the
% required angle of attack to achieve a target lift coefficient. Finally,
% the drag is computed.

%If min power mode then trim for min power
if strcmp(aero.mode, 'minPower')
    aero.CL    = interp1q(aero.minPower.h_m(end:-1:1), aero.minPower.CL(end:-1:1), aircraft.states.h_m);
    
    % Compute V based on known gamma or hdot
    if isfield(freeStream, 'gamma_rad')
        VTAS_ms = sqrt((aircraft.massProperties.MGTOW_N*2*cos(freeStream.gamma_rad))/(freeStream.rho_kgm3*aero.CL*aircraft.aero.Sref_m2)); 
    elseif isfield(freeStream, 'hdot_ms') % Solve for a non-linear equation as small angles can't be assumed
        FV = @(X) 1/2*freeStream.rho_kgm3*aero.CL*aircraft.aero.Sref_m2*X(1)^2 - aircraft.massProperties.MGTOW_N*2*cos( asin(freeStream.hdot_m2/X(1)) );
        VTAS_ms = Newton(FV, sqrt(2*aircraft.massProperties.MGTOW_N/ (freeStream.rho_kgm3 * aircraft.aero.Sref_m2 )), 'relax', 1);
    else
        VTAS_ms = sqrt((aircraft.massProperties.MGTOW_N*2)/(freeStream.rho_kgm3*aero.CL*aircraft.aero.Sref_m2)); 
    end
    
    Re_actual  = VTAS_ms* aircraft.aero.Cref_m*freeStream.rho_kgm3/freeStream.dynamicViscosity_Nsm2;
elseif strcmp(aero.mode, 'specifiedV')
    Re_actual  = freeStream.VTAS_ms * aircraft.aero.Cref_m*freeStream.rho_kgm3/freeStream.dynamicViscosity_Nsm2;
end

% Look-up alpha, and CD from the stored polar
% Invert AoA, Re vs CL relationship to find AoA
CLvsAoA = ScaleTimeBulletProof(aero.polar.CL, aero.polar.Re, Re_actual);
CDvsAoA = ScaleTimeBulletProof(aero.polar.CD, aero.polar.Re, Re_actual);

% Bullet proof against excursions outside of the polar
if aero.CL < min(CLvsAoA)
    aero.CL = min(CLvsAoA); 
elseif aero.CL > max(CLvsAoA)
    aero.CL = max(CLvsAoA);
end

aero.alpha_rad = interp1q(CLvsAoA,  aero.polar.alpha_rad, aero.CL);
aero.CD        = interp1q(aero.polar.alpha_rad, CDvsAoA', aero.alpha_rad);
aero.Re        = Re_actual;

% Compute forces
if isfield(freeStream, 'VTAS_ms')
    aero.lift_N   = 1/2 * freeStream.rho_kgm3 * freeStream.VTAS_ms^2 * aero.Sref_m2 * aero.CL;
    aero.drag_N   = 1/2 * freeStream.rho_kgm3 * freeStream.VTAS_ms^2 * aero.Sref_m2 * aero.CD;
    aero.forces_N = [aero.drag_N; aero.lift_N];
end

end
