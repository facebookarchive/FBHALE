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
  
function propulsion = Motor(propulsion)
% This function calls the specified motor model to convert shaft power into
% electrical power.

% Selected propulsor
aP = propulsion.activePropulsor;

% Bullet proofing
if ~isfield(propulsion.propulsor(aP).motor, 'description')
    error(['No motor model was specified for the propulsor ' num2str(aP)]); 
end

% Select the appropriate model
switch propulsion.propulsor(aP).motor.description
    case 'constantEfficiency'
        propulsion = ConstantEfficiencyMotor(propulsion);
    case 'Map'
        error('No motor performance map table-lookup routine developed yet');
end
    
end


