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
  
function optim = SizePropulsion(optim)
% Sizes the propulsion system given propeller sizing requirement (max tip
% speed, target loading, RPM margin) and climb requirement. First the
% propeller is sized and and the worst case torque is computed. Then, a
% torque based sizing is used for the motor and compared against a power
% based sizing. The worst case of the two is retained. Increments
% are added for mounts, ESC, propeller. Lastly, the yaw due to a motor out
% case in climb is computed and can be used to size a vertical tail.

% Size and Characterize Propeller
optim = SizeNCharacterizePropeller(optim);

%% Power-based sizing
PowerBased.motor.mass_kg = optim.propulsion.propeller.PMax_W * optim.propulsion.kgW;

%% Alternative sizing: torque-based
% Empirical fit based sizing
tau_specific_Nmkg = optim.propulsion.torqueWeightConstants(1) * optim.propulsion.propeller.Qmax_Nm^optim.propulsion.torqueWeightConstants(2);
TorqueBased.motor.mass_kg = optim.propulsion.propeller.Qmax_Nm / tau_specific_Nmkg; % Per motor

%% Take the max weights for conservatism

if TorqueBased.motor.mass_kg > PowerBased.motor.mass_kg
    optim.propulsion.motor.mass_kg   = TorqueBased.motor.mass_kg;
else
    optim.propulsion.motor.mass_kg   = PowerBased.motor.mass_kg;
end

% Inculde other masses (propeller, mounts, variable pitch, etc.)
optim.propulsion.mass_kg = optim.propulsion.motor.mass_kg ...
    * optim.propulsion.propulsionNonMotorComponentsRelativeMass ...
    * ones(1, length(optim.propulsion.N));

% Record maximum motor power for performance calculations. If this didn't
% size the propulsion, then it provides extra margin
optim.propulsion.PpropMax_W = optim.propulsion.propeller.PMax_W;

%% Compute residual yaw when the outter motor goes down at max thrust / sea
% level climb
optim = MotorOutCase(optim);

end
