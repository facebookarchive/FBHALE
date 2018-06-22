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
  
function [optim] = FindRollControlSurface(optim)
%This function returns the name of the aero surface that is used for roll
%and returns the index of the actuator on that surface used for roll.
actuationType = {'roll' 'pitch' 'yaw'};
actuationTypeSchedule = {1 2 3};

for t = 1:length(actuationType)
    for b = 1 :length(optim.aircraft.beamNames)
        %loop through all control surfaces on surface and check if used for roll
        if isfield(optim.(optim.aircraft.beamNames{b}), 'controlSurface')
            for n = 1:length(optim.(optim.aircraft.beamNames{b}).controlSurface)
                if isequal(optim.(optim.aircraft.beamNames{b}).controlSurface(n).schedule(t), actuationTypeSchedule{t})
                    if exist('rollSurfaceName') || exist('rollContolSurfaceIndex')
                        error('Multiple control surfaces effecting same response not supported. Mixing logic needs to be set up to support this capability')
                    else
                        optim.aircraft.controlSurfaces.(actuationType{t}).surfaceName = optim.aircraft.beamNames{b};
                        optim.aircraft.controlSurfaces.(actuationType{t}).contolSurfaceIndex = n;
                    end
                end
            end
        end
    end
end

end

