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
  
function optim = SizeAndBalance(optim, iterCnt)
% This function runs through the sizing of major subsystems and is the top
% level of the design loop. As subsystems are sized, pitch trim is also
% enforced. Exiting this function, the sum of the weights is not equal to
% the assigned MGTOW and MDOTopLevel enforces weight compatibility by
% adjusting battery weight accordingly.

% For the first iteration erase a potential bad guess
if iterCnt == 1
    optim = UpdateMassProperties(optim);
end

% Compute aerodynamic coefficients and quantities required for sizing tails
% and enforcing pitch trim i.e maximum adverse yaw, CG under deflection.
% Check for aileron reversal across envelope.
disp('  Gathering aerodynamic coefficients for tail sizings...');
optim = RunASWINGAeroCoefficients(optim);
    
if iterCnt > 1
    % Size ailerons to achieve a target non-dimentional roll rate and check 
    % for aileron reversal
    disp('  Sizing/Checking roll control...');
    optim = CheckRollRate(optim, iterCnt);
    
end

optim = UpdateMassProperties(optim, 'analytical');

% Enforce pitch trim. Evaluate static margin (SM) for flexible aircraft
disp('  Pitch trimming...');
optim = PitchTrim(optim, iterCnt);
disp('      Checking vehicle stability...');
optim = RunASWINGFullVehicleStability(optim);
SM    = -optim.wing.aero.vehicleCmAlphaVc/optim.wing.aero.vehicleCZa_VC;
optim.aircraft.SM = SM;

if iterCnt == 1
    % Now that the aircraft is balanced, we can size ailerons for the first
    % time
    disp('  Sizing/Checking roll control...');
    optim = CheckRollRate(optim, iterCnt);
end

% Capture aerodynamic performance
disp('  Running polar sweep...');
optim = RunASWINGPolar(optim, [1,1,1,1]);
optim = AerodynamicPerformance(optim);
optim = GenerateVn(optim);

% Size propulsion system
disp('  Sizing propulsion system...');
optim = SizePropulsion(optim);

% Reshape fuselage to accomodate for updated component positions
optim = ShapeFuselage(optim);

% Update wing position for pitch trim and record static margin
optim = UpdateMassLocations(optim);
optim = UpdateMassProperties(optim);

end