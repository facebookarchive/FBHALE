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
  
function [ optim ] = WriteBeams( optim, aswingIn, useStructure, aeroADB )
%This function loops through the various beams(wings, tail, pods, booms)
%and writes them into the aswing file using the provided aero database and
%structural switches

% loop through beam numbers and write beam to file
beamNumber = 1;
for i = 1: length(optim.aircraft.beamNames)
    entity = optim.aircraft.beamNames{i};
    
    % if beam is relected (.N = 2) write twice otherwise only once
    for y = 1:length(optim.(entity).N)
        if optim.(entity).N(y) == 2
            optim = WriteEntity( optim,aswingIn, entity, useStructure,  'right', beamNumber, aeroADB, y);
            beamNumber = beamNumber+1;
            optim = WriteEntity( optim,aswingIn, entity, useStructure,  'left', beamNumber, aeroADB, y);
            beamNumber = beamNumber+1;
        else
            optim = WriteEntity( optim,aswingIn, entity, useStructure,  'nonReflected', beamNumber, aeroADB, y);
            beamNumber = beamNumber+1;
        end
    end
end
end

function [optim] = WriteEntity( optim,aswingIn, entity, useStructure,  placement, beamNumber, aeroADB, i)
    reflectionFactor = 1;
    entityName = entity;
    if strcmp(placement, 'left')
        reflectionFactor = -1;
        entityName = [entity ' Reflect'];
    end

    if strcmp(entity, 'wing')
        optim = WriteWing(optim, aswingIn, useStructure, aeroADB, beamNumber, entityName, i);  
    elseif strcmp(entity , 'htail')
        optim = WriteHtail(optim, aswingIn,  useStructure,  reflectionFactor, beamNumber, aeroADB, entityName, i);
    elseif strcmp(entity, 'vtail')
        optim = WriteVtail( optim,aswingIn, useStructure, reflectionFactor, beamNumber, aeroADB, entityName, i);
    elseif strcmp(entity, 'boom')
        optim = WriteBoom(optim, aswingIn,  useStructure,  reflectionFactor, beamNumber, entityName, i);
    elseif strcmp(entity, 'pod')
        optim =  WritePods( optim, aswingIn, useStructure, reflectionFactor, beamNumber,  entityName, i);
    else
        error('Unknown beam type specified')
    end

end






