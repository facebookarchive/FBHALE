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
  
function optim = MinPowerCL(optim)
% This function takes the airframe polars vs Re and finds the CL
% corresponding to minimum power various Re. This is later used in mission
% calculations.

% Minimum power criterion
P  = optim.aircraft.aero.polar.CD./optim.aircraft.aero.polar.CL.^(3/2);
P(optim.aircraft.aero.polar.CL < .5) = NaN;

% At each Re, find AoA that minimizes power
[~, indices] = min(P');
AoAMatrix    = optim.aircraft.aero.polar.AoA_deg * ones(1, length(optim.aircraft.aero.polar.Re));
optim.aircraft.aero.minPower.Re      = optim.aircraft.aero.polar.Re;
optim.aircraft.aero.minPower.AoA_deg = AoAMatrix(indices);
optim.aircraft.aero.minPower.CL      = diag(optim.aircraft.aero.polar.CL(:, indices));
optim.aircraft.aero.minPower.CD      = diag(optim.aircraft.aero.polar.CD(:, indices));
 
% Compute min power CL vs altitude
h_m = [0 linspace(optim.mission.minh_m, optim.mission.maxh_m, 10)];
[rho_kgm3, dynamicViscosity]  =  GetAtmosphereProperties(h_m);
K = 1/2*rho_kgm3.*(dynamicViscosity./(rho_kgm3 * optim.aircraft.cref_m)).^2 * optim.aircraft.Sref_m2;

for i = 1:length(h_m)
    F = @(x) optim.MGTOW_kg*optim.constants.g_ms2 - K(i)*x.^2.*interp1(optim.aircraft.aero.minPower.Re, optim.aircraft.aero.minPower.CL, x);
    X = F(optim.aircraft.aero.minPower.Re);
    Y = optim.aircraft.aero.minPower.Re;
    
    % Bullet proof
    Y(isnan(X)) = [];
    X(isnan(X)) = [];
    Re_minPower(i) = interp1(X, Y, 0, 'linear', 'extrap');
end

optim.aircraft.aero.minPower.h_m = interp1(Re_minPower, h_m, optim.aircraft.aero.minPower.Re, 'linear', 'extrap');

end
