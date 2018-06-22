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
  
function propulsion = ConstantEfficiencyMotor(propulsion)
% This is the simplest motor model there can be, a simple constant
% efficiency linking shaft power to electrical power.

% Active propulsor
aP = propulsion.activePropulsor;

% Bullet proofing
if ~isfield(propulsion.propulsor(aP).motor, 'eta')
    warning('No efficiency specified for motor despite constant efficiency model selected. Using 80% efficiency as default');
    propulsion.propulsor(aP).motor.eta = 0.8;
end
if ~isfield(propulsion.propulsor(aP).propeller, 'shaftPower_W')
    error(['No shaft power was assigned to propulsor ' num2str(aP) '.']);
end
    
% Power conversion
propulsion.propulsor(aP).motor.electricalPower_W = propulsion.propulsor(aP).propeller.shaftPower_W / (propulsion.propulsor(aP).motor.eta * propulsion.propulsor(aP).motor.controller_eta);

end
