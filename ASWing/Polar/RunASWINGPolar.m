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
  
function [ optim ] = RunASWINGPolar( optim, useStructure )
% This function takes in optim and returns the data needed to generate a full vehicle polar
% First it generates a .asw file then it writes an input string that will
% run ASWing through the desired conditions and save as the output values
% then parses out the desired outputs

% Runs a polar for the whole airframe and stores corresponding data in
% optim.(wing).aero.ASWINGpolar as well as htail and vtail values.
% At each AoA, CL and CDi are recorded in addition to the span distribution
% of the local lift coefficient:

% optim.wing.aero.ASWINGpolar.alpha_deg  (Nx1) vector
% optim.wing.aero.ASWINGpolar.CL         (Nx1) vector
% optim.wing.aero.ASWINGpolar.CDi        (Nx1) vector
% optim.wing.aero.ASWINGpolar.y_m        (Mx1) vector
% optim.wing.aero.ASWINGpolar.cl         (NxM) matrix % and all other
% values specified in optim.polar.outputColumNames

%% Write ASWing File
optim = WriteASWingFile(optim, [optim.ASWINGIODir 'PolarAircraft' num2str(optim.designID) '.asw' ], 'polarNoSensors', useStructure);

% Find velocity range for the polar
vmax_ms = sqrt((2*optim.MGTOW_kg*optim.constants.g_ms2)/(optim.constants.rhoSL_kgm3*optim.aircraft.Sref_m2*optim.aircraft.aero.ASWINGpolar.minCL));
vmin_ms = sqrt((2*optim.MGTOW_kg*optim.constants.g_ms2)/(optim.constants.rhoSL_kgm3*optim.aircraft.Sref_m2*optim.aircraft.aero.ASWINGpolar.maxCL));
optim.aircraft.aero.ASWINGpolar.V_ms = linspace(vmax_ms, vmin_ms, 15);

% Write Input String
WriteASWingPolarInputString( optim )

%Run ASWING
system(['aswing<' optim.ASWINGIODir 'PolarInputString' num2str(optim.designID) '.txt>' optim.ASWINGIODir 'ASWingPolarRawOutputData.txt ' ' 2>&1']);
%% Parse output values and organise as expected
ASwingOutput = ParseASWingPolarOutput([optim.ASWINGIODir 'ASWingPolarRawOutputData.txt'], optim);

% Populate CL and Y vectors for each aero surface
for i = 1:length(fieldnames(ASwingOutput))
    eNames = fieldnames(ASwingOutput);
    entityName = eNames{i};
    
    for w = 1:length(optim.(entityName).N)     
        ASwingOutput.(entityName).CL{w}     = ASwingOutput.(entityName).cl{w}(:,1:end);
        ASwingOutput.(entityName).CLy_m{w}  = ASwingOutput.(entityName).y_m{w}(:,1:end);
        ASwingOutput.(entityName).t         = ASwingOutput.(entityName).t(:,1:end);
    end
end

%%%%%% Sensor Quantities %%%%%%
%%  Parse Aircraft Quantities
filename = [optim.ASWINGIODir 'PolarOutputs' num2str(optim.designID) '.txt'];
[~, ~, Flap1_out, Flap2_out, Flap3_out, alpha_out, CDi_out, CL_out] = ParsePolarVehileOutputs(filename);

optim.aircraft.aero.ASWINGpolar.AoA_deg        = alpha_out;
for b = 1:length(optim.aircraft.beamNames)
    if ~max(strcmp(optim.aircraft.beamNames{b}, {'pod', 'boom'}))
        optim.(optim.aircraft.beamNames{b}).aero.ASWINGpolar.AoA_deg = alpha_out;
    end
end

optim.aircraft.aero.ASWINGpolar.CL             = CL_out;
optim.aircraft.aero.ASWINGpolar.CDi            = CDi_out;

% Control Surface Deflections
for b = 1 :length(optim.aircraft.beamNames)
    if isfield(optim.(optim.aircraft.beamNames{b}), 'controlSurface')
        for i = 1:length(optim.(optim.aircraft.beamNames{b}).controlSurface)
            if ~isfield(optim.(optim.aircraft.beamNames{b}).controlSurface(i), 'deflection_deg')
                if isequal(optim.(optim.aircraft.beamNames{b}).controlSurface(i).schedule(1), 1)
                    optim.(optim.aircraft.beamNames{b}).aero.ASWINGpolar.controlSurface(i).deflection_deg = zeros(2, size(Flap1_out,1));
                    usedForRollReplicationFactor = 2;
                elseif isequal(optim.(optim.aircraft.beamNames{b}).controlSurface(i).schedule(2), 1) || isequal(optim.(optim.aircraft.beamNames{b}).controlSurface(i).schedule(3), 1)
                    optim.(optim.aircraft.beamNames{b}).aero.ASWINGpolar.controlSurface(i).deflection_deg = zeros(1, size(Flap1_out,1));
                    usedForRollReplicationFactor = 1;
                end
            end
            
            if isequal(optim.(optim.aircraft.beamNames{b}).controlSurface(i).schedule(1), 1) %Roll
                optim.(optim.aircraft.beamNames{b}).aero.ASWINGpolar.controlSurface(i).deflection_deg = ...
                    [Flap1_out'; -1*Flap1_out'*1/optim.(optim.aircraft.beamNames{b}).controlSurface(i).gearRatio] + optim.(optim.aircraft.beamNames{b}).aero.ASWINGpolar.controlSurface(i).deflection_deg;
            end
            if isequal(optim.(optim.aircraft.beamNames{b}).controlSurface(i).schedule(2), 1) %Pitch
                optim.(optim.aircraft.beamNames{b}).aero.ASWINGpolar.controlSurface(i).deflection_deg = ...
                    repmat(Flap2_out',usedForRollReplicationFactor,1)+optim.(optim.aircraft.beamNames{b}).aero.ASWINGpolar.controlSurface(i).deflection_deg;
            end
            if isequal(optim.(optim.aircraft.beamNames{b}).controlSurface(i).schedule(3), 1) %Yaw
                optim.(optim.aircraft.beamNames{b}).aero.ASWINGpolar.controlSurface(i).deflection_deg = ...
                    repmat(Flap3_out',usedForRollReplicationFactor,1) + optim.(optim.aircraft.beamNames{b}).aero.ASWINGpolar.controlSurface(i).deflection_deg;
            end
        end
    end
end
       
% Select Aero Entities 
aeroEntities = {};
previousLastIndex = 0;
for b = 1:length(optim.aircraft.beamNames)
    if ~max(strcmp(optim.aircraft.beamNames{b}, {'pod', 'boom'}))
        aeroEntities{end+1} = optim.aircraft.beamNames{b};
        for n = 1:length(optim.(aeroEntities{end}).N)
            lastIndex.(aeroEntities{end})(n) = previousLastIndex + 2*length(optim.(aeroEntities{end}).structure.spanLocation);
            previousLastIndex = lastIndex.(aeroEntities{end})(n);
        end
    end
end

% Now post preprocess data into expected output format
for z = 1:length(aeroEntities)
    for n = 1:length(optim.(aeroEntities{z}).N)      
            for i = 1:length(optim.aircraft.aero.ASWINGpolar.outputColumNames)
                optim.(aeroEntities{z}).aero.ASWINGpolar.(optim.aircraft.aero.ASWINGpolar.outputColumNames{i}) = ASwingOutput.(aeroEntities{z}).(optim.aircraft.aero.ASWINGpolar.outputColumNames{i});
            end
            optim.(aeroEntities{z}).aero.ASWINGpolar.t{n}  = ASwingOutput.(aeroEntities{z}).t;
    end
    
end

end

