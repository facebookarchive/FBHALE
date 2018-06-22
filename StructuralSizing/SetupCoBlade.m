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
  
function [optim SIM ANLS OPT ENV BLADE WEB OUT MATS AF Coord] = SetupCoBlade (optim, entity, idx)
% This routine prepares inputs and initializes for coblade processing. 

%% Define Geometry:

    structData                = optim.(entity).structure;
    structData.sRefX_m{idx}        = structData.spanLocation*...
                                structData.length_m(idx)*sind(optim.(entity).sweep_deg);
    structData.sRefY_m{idx}        = structData.spanLocation(:)*...
                                structData.length_m(idx)*cosd(optim.(entity).sweep_deg);                        
    structData.sRefZ_m{idx}        = zeros(structData.numStations,1);
    spanLocation              = structData.spanLocation;
    structData.t{idx}              = spanLocation;
    structData.RemoveStationsStressCalc = 0;
                                                                                      
    if strcmp(entity,'wing')  
        
        if isfield(optim.(entity), 'winglet') && optim.wing.winglet.zoc > 0
            
            structData.length_m(idx)        = optim.wing.bref_m*0.5/cosd(optim.wing.sweep_deg)+ ...
                                         abs(optim.wing.winglet.zoc * structData.chord_m{idx}(end));
            spanLocation               = structData.spanLocation*(optim.wing.bref_m*0.5/cosd(optim.wing.sweep_deg))...
                                          /structData.length_m(idx);
            spanLocation(end-1)        = spanLocation(end);   
            spanLocation(end)          = 1;                         
                                                                        
            structData.chord_m{idx}(end-1)  =  structData.chord_m{idx}(end);
            structData.airfoil{idx}(end-1)  =  structData.airfoil{idx}(end);
            structData.toc{idx}(end-1)      =  structData.toc{idx}(end);
            structData.twist_deg{idx}(end)  =  0;


            structData.sRefX_m{idx}         = spanLocation(1:end-1)*...
                                         structData.length_m(idx)*sind(optim.wing.sweep_deg);
            structData.sRefX_m{idx}         = [structData.sRefX_m{idx}; structData.sRefX_m{idx}(end)];
            structData.sRefY_m{idx}         = spanLocation(1:end-1)*...
                                         structData.length_m(idx)*cosd(optim.wing.sweep_deg);
            structData.sRefY_m{idx}         = [structData.sRefY_m{idx}; structData.sRefY_m{idx}(end)+1e-8];
            structData.sRefZ_m{idx}         = zeros(structData.numStations-1,1);
            structData.sRefZ_m{idx}         = [structData.sRefZ_m{idx}; -optim.wing.winglet.zoc*structData.chord_m{idx}(end)];
            structData.t{idx}               =  spanLocation/(optim.wing.bref_m*0.5/cosd(optim.wing.sweep_deg))...
                                          *structData.length_m(idx);
            structData.RemoveStationsStressCalc = 2;                          

        end   
    end


%% Co-Blade initialization settings:

SIM.iSIM = 1;
SIM.case{SIM.iSIM} = strcat(num2str(structData.designID),'_',structData.name,'_structData.temp');
SIM.inpFile{SIM.iSIM} = [optim.CoBladeIODir strcat(num2str(structData.designID),'_',structData.name,'_structData.temp')];
SIM.version = '1.23.00-dcs';

SIM.rootDir     = fileparts(mfilename('fullpath'));
SIM.sourceDir   = [SIM.rootDir filesep 'Source'];
SIM.airfoilDir  = [SIM.rootDir filesep 'Airfoil_Data'];
SIM.materialDir = [SIM.rootDir filesep 'Material_Data'];
SIM.laminateDir = [SIM.rootDir filesep 'Laminate_Data'];
SIM.optimDir    = [SIM.rootDir filesep 'Optimization_Data'];

%% Set global properties:

G = regexp(fileread('coblade.main'), '\n', 'split');
G{18} = sprintf('%d OUB_STN', structData.numStations);
G{19} = sprintf('%d NUM_CP:         Number of control points between INB_STN and OUB_STN',2);
G{33} = sprintf('%d No .of blade sections', structData.numStations);
G{34} = sprintf('%2.3f  Blade Length', structData.length_m(idx));
if isfield(structData,'materialsFile')
G{43} = sprintf('%s  Material File', structData.materialsFile);
end
F = {G{56:end}};
F{6} = sprintf('%d \t %d \t %d \t %2.6f %2.6f',1,1,structData.numStations,...
    0.4, 0.4);
F{7} = sprintf('%d \t %d \t %d \t %2.6f %2.6f',2,1,structData.numStations,...
    0.6, 0.6);

%% Set distributed properties:

for i=1:structData.numStations
    % le position, chord, twist, airfoil, layup:
    pitchRefAxis = structData.pitchAxis;
    G{46+i} = sprintf('%2.6f \t %2.6f \t %2.6f \t %2.6f \t %2.6f \t %2.6f \t %2.6f \t %s',...
        spanLocation(i),  0, structData.chord_m{idx}(i),pitchRefAxis, 0,...
        0,0, structData.airfoil{idx}{i});
end

fid = fopen(SIM.inpFile{SIM.iSIM}, 'w');
nlines = length({G{1:46+structData.numStations}})+length({F{:}});
C = cell(nlines,1);
[C{:}] = deal(G{1:46+structData.numStations},F{:});
fprintf(fid, '%s\n', C{:});
fclose(fid);

%% Initialize Coblade:

[ANLS OPT ENV BLADE WEB OUT MATS AF Coord] = CoBlade_init(SIM);

WEB.inbStn = OPT.INB_STN .* ones(WEB.NUM_WEBS, 1);
WEB.oubStn = OPT.OUB_STN .* ones(WEB.NUM_WEBS, 1);

% Scale densities for uncertainty analysis
if ~isfield(structData,'uncertaintyAnalysis')
structData.uncertaintyAnalysis.specificModulus = 0;
structData.uncertaintyAnalysis.secondaryStruct = 0;
end

MATS.density = MATS.density/(1+structData.uncertaintyAnalysis.specificModulus);
optim.(entity).structure = structData;
