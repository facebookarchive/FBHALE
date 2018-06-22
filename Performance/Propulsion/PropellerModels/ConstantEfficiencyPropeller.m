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
  
function propulsion = ConstantEfficiencyPropeller(propulsion, freeStream)
% This is the simplest propeller model there can be, a simple constant
% efficiency linking propulsive power to shaft power. This function
% requires a required thrust to be specified to calculate propulsive
% power.

% Selected propulsor
aP = propulsion.activePropulsor;

% Bullet proofing
if ~isfield(propulsion.propulsor(aP).propeller, 'thrust_N')
    error('No required thrust was specified for the constant efficiency propeller model.');
end
if ~isfield(propulsion.propulsor(aP).propeller, 'efficiency')
    error('No efficiency specified for the constant efficiency propeller model.');
end
if ~isfield(freeStream, 'VTAS_ms')
    error('No freestream speed was specified. Cannot look-up propeller performance.');
end

% Propulsive then shaft power
propulsivePower_W = propulsion.propulsor(aP).propeller.thrust_N * freeStream.VTAS_ms;
propulsion.propulsor(aP).propeller.shaftPower_W = propulsivePower_W / propulsion.propulsor(aP).propeller.efficiency;

end

