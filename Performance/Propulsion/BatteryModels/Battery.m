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
  
function batteries = Battery(aircraft)
% This function calls the specified battery model and provides electrical
% power to the propulsion system.

% Bullet proofing
if ~isfield(aircraft.batteries, 'description')
    error('No battery model was specified !'); 
end

% Select the appropriate model
switch aircraft.batteries.description
    case 'constantSpecificEnergy'
        batteries = ConstantSpecificEnergyBattery(aircraft);
    case 'equivalentCircuit'
        error('No equivalent circuit battery model developed yet');
end

end
