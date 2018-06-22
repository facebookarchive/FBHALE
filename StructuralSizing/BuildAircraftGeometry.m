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
  
function optim = BuildAircraftGeometry (optim)
% This function builds the aircraft stick geometry in global coordinates from
% relative coordinates (w.r.t beam axis) using the appropriate
% transformations. 

for i = 1:length(optim.aircraft.beamNames)
    entity = optim.aircraft.beamNames{i};
    for k =1:length(optim.(entity).N)
        if strcmp(entity, 'vtail')
            [optim.(entity).globalx_m{k}, optim.(entity).globaly_m{k}, optim.(entity).globalz_m{k}] = TransformCoordinates(optim.(entity).structure.sRefX_m{k}, ...
                optim.(entity).structure.sRefY_m{k}, optim.(entity).structure.sRefZ_m{k}, optim.(entity).x_m(k), optim.(entity).y_m(k), optim.(entity).z_m(k), 90, 0);
        elseif strcmp(entity, 'boom') || strcmp(entity, 'pod')
            [optim.(entity).globalx_m{k}, optim.(entity).globaly_m{k}, optim.(entity).globalz_m{k}] = TransformCoordinates(optim.(entity).structure.sRefX_m{k}, ...
                optim.(entity).structure.sRefY_m{k}, optim.(entity).structure.sRefZ_m{k}, optim.(entity).x_m(k), optim.(entity).y_m(k), optim.(entity).z_m(k), 0, -90);
        else           
            [optim.(entity).globalx_m{k}, optim.(entity).globaly_m{k}, optim.(entity).globalz_m{k}] = TransformCoordinates(optim.(entity).structure.sRefX_m{k}, ...
                optim.(entity).structure.sRefY_m{k}, optim.(entity).structure.sRefZ_m{k}, optim.(entity).x_m(k), optim.(entity).y_m(k), optim.(entity).z_m(k), 0, 0);
        end
    end
    
end

end