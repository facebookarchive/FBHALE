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
  
% This function is called to either figure out how much thrust can be
% obtained at known RPM and airspeed (useful during aircraft trimming), or
% to calculate how much shaft power this condition corresponds to (useful
% when calling Propulsion.m).
function propulsion = Propeller(propulsion, freeStream)

% Selected propulsor
aP = propulsion.activePropulsor;

% Bullet proofing
if ~isfield(propulsion.propulsor(aP).propeller, 'description')
    error(['No propeller model was specified for the propulsor ' num2str(aP)]); 
end

% Select appropriate model
switch propulsion.propulsor(aP).propeller.description
    case 'constantEfficiency'
        propulsion = ConstantEfficiencyPropeller(propulsion, freeStream);
	case 'variableEfficiency'
        propulsion = VariableEfficiencyPropeller(propulsion, freeStream);
end

end

