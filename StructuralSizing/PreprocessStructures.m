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
  

function optim = PreprocessStructures (optim)
% This script preprocesses structural input data:
% 
% 1. Create airfoil geometry and coblade shape files from aero database
% 2. Perform shape interpolation/blending between breakpoints
% 3. Initialize coblade inputs and interpolate quantities
%    from breakpoint definition through the structural grid. 

% Select all lifting beams
entities = optim.aircraft.beamNames;

for j = 1:length(entities)
    
    %% Create Airfoil Shape Files
    cd (optim.BoxShapeDir);
    
    % create airfoil dat files
    optim = WriteAirfoilFiles (optim, entities{j});
    
    % settings
    s = optim.(entities{j}).structure;
    s.name = entities{j};
    s.designID = optim.designID;
    s.recordMaxValues = false;
    s.init = true;
    s.FOS  = optim.environment.FactorOfSafety;
        
    % discrete interpolation:
    numBreaks  = length(s.airfoil);
    s.mass_kg = 1;
    
    % interpolate airfoils:
    airfoils = s.airfoil;
    interpolatingGrid = linspace(0,1,s.DivsPerSection);
    for i = 1:numBreaks-1
        airfoil1 = [airfoils{i} '.dat'];
        airfoil2 = [airfoils{i+1} '.dat'];
        if ~strcmp(airfoil1,airfoil2)
            for k = 1+(i>1):s.DivsPerSection 
                interpolatingFraction = interpolatingGrid(k);
                interpolatedAirfoilName = [num2str(s.designID) '_' s.name '_' num2str(i) '_' num2str(k)];
                BlendAirfoil(airfoil1,airfoil2,interpolatingFraction,[interpolatedAirfoilName '.dat']);
                CreateCoBladeGeometry ([interpolatedAirfoilName '.dat']);
                s.airfoil{(i-1)*s.DivsPerSection+k} = [interpolatedAirfoilName '.pcf'];
            end
        else
                CreateCoBladeGeometry ([airfoils{i} '.dat']);
            for k = 1:s.DivsPerSection              
                s.airfoil{(i-1)*s.DivsPerSection+k} = [airfoils{i} '.pcf'];
            end
            
        end
    end
    
    cd (optim.rootDir);
    
    % clean up empty cells 
    airfoil = s.airfoil(~cellfun(@isempty, s.airfoil)); 
    s.numStations = length(airfoil);
    s.airfoil = {};
    for idx = 1:length(optim.(entities{j}).N)
       s.airfoil{idx} = airfoil;
    end
       
     %% Set Defaults:
   
    if ~isfield(s,'limitLoads')  || ~isfield(s,'ResLoads')
        ResLoads.Vx = zeros(s.numStations,1);
        ResLoads.Vy = zeros(s.numStations,1);
        ResLoads.Vz = zeros(s.numStations,1);
        ResLoads.Mx = zeros(s.numStations,1);
        ResLoads.My = zeros(s.numStations,1);
        ResLoads.Mz = zeros(s.numStations,1);
        ResLoads.dVx_dz = zeros(s.numStations,1);
        ResLoads.dVy_dz = zeros(s.numStations,1);
        ResLoads.dVz_dz = zeros(s.numStations,1);
        ResLoads.dMx_dz = zeros(s.numStations,1);
        ResLoads.dMy_dz =zeros(s.numStations,1);
        ResLoads.dMz_dz = zeros(s.numStations,1);
    end
    s.ResLoads = ResLoads;
        
    if ~isfield(s,'materialsFile')
        s.materialsFile = 'materials.inp';
    end
    if ~isfield(optim.(entities{j}),'sweep_deg')
        optim.(entities{j}).sweep_deg = 0;
    end
    if ~isfield(s,'sizingLocations')
    	s.sizingLocations = [0 1];
    end
    if ~isfield(s,'twist_deg')
        s.twist_deg = zeros(1, length(s.sizingLocations));
    end
    if ~isfield(s,'skinWeightPerArea_kgm2')
        s.skinWeightPerArea_kgm2 = 0;
    end
    if ~isfield(s,'NribsPerSpan_m')
        s.NribsPerSpan_m = 0;
    end
    if ~isfield(s,'ribMassPerChord_kgm')
        s.ribMassPerChord_kgm = 0;
    end
    if ~isfield(s,'fairingWeightPerArea_kgm2')
        s.fairingWeightPerArea_kgm2 = 0;
    end
    if ~isfield(s,'fuselageCanopySectionsR_m')
        for idx = 1:length(optim.(entities{j}).N)
           s.fuselageCanopySectionsR_m{idx} = 0;
        end
    end
    if ~isfield(s,'UDScaleStations')
        s.UDScaleStations = 1:s.numStations;
    end
    if ~isfield(s,'TopUDScale')
    	s.TopUDScale = ones(1, s.numStiffeners);
    end
    if ~isfield(s,'BotUDScale')
        s.BotUDScale = ones(1, s.numStiffeners);
    end
    if ~isfield(s,'boxSkin_nPly')
    	s.boxSkin_nPly = [0 0];
    end
    if ~isfield(s,'sparCapWidthInboard') || ~isfield(s,'sparCapWidthOutboard')
        s.sparCapWidthInboard  = s.sparCapWidth;
        s.sparCapWidthOutboard = s.sparCapWidth;
    end
    if ~isfield(s,'length_m')
    	s.length_m  = 1;
    end
    if ~isfield(s,'chord_m')
    	s.chord_m  = ones(1, s.numStations);
    end
    if ~isfield(s,'UD_nPly')
    	s.UD_nPly = zeros(1, s.numStations);
    end
    if ~isfield(s,'UDScaleSpanExtent')
    	s.UDScaleSpanExtent  = [0 1];
    end
    if ~isfield(s,'airfoilBreakPoints')
    s.airfoilBreakPoints = s.sizingLocations;
    end
     
    %% interpolate distributed quantities and package
    s.UDScaleStations = ceil(s.UDScaleSpanExtent(1)*s.numStations):ceil(s.UDScaleSpanExtent(2)*s.numStations);
    s.spanLocation    = ReplaceNearest (ScaleTime (s.airfoilBreakPoints, 1, length(s.airfoilBreakPoints), s.numStations),...
                     union(s.airfoilBreakPoints, s.sizingLocations));
                 
    chord_m   = s.chord_m;
    twist_deg = s.twist_deg;
    length_m  = s.length_m;
    if isfield(s,'toc')
    toc       = s.toc;
    end
    s.chord_m = {};
    s.twist_deg = {};
    s.airfoilNames = {};
    s.toc = {};
    s.length_m = [];
    for idx = 1:length(optim.(entities{j}).N)
    s.chord_m{idx}         = interp1q (s.sizingLocations', chord_m', s.spanLocation')';
    s.twist_deg{idx}       = interp1q (s.sizingLocations', twist_deg', s.spanLocation')'; 
    s.airfoilNames{idx}    = airfoils;	
    s.length_m(idx)        = length_m;
    if isfield(s,'toc')
    s.toc{idx}             = interp1q (s.airfoilBreakPoints', toc', s.spanLocation')';
    end
    end
        
   %% Initialize Structure:
   
    optim.(entities{j}).structure = s;
    optim = CoBlade(optim, entities{j});   
    optim.(entities{j}).structure.init = false;
                
end


end

function optim = WriteAirfoilFiles (optim, entity)

%% Create airfoil shapes from toc using xfoil geo routine 

if isfield (optim.(entity),'aero')

    for i = 1:length(optim.(entity).toc)

        % assign airfoil/shape name
        optim.(entity).structure.airfoil{i} = [entity '_' num2str(optim.(entity).toc(i))];

        % get airfoil coordinates
        idx = findnearest (optim.(entity).toc(i),optim.(entity).aero.airfoilADB.dim1.vector);
        coords = optim.(entity).aero.airfoilADB.geom.xy{idx};

        % write xfoil data file
        fid = fopen(sprintf('%s.dat',optim.(entity).structure.airfoil{i}), 'w');
        fprintf(fid,'%s\n',optim.(entity).structure.airfoil{i});
        for j = 1:length(coords)
            fprintf(fid,'%f %f\n',coords(j,1), coords(j,2));
        end
        fclose(fid);
        
        % scale airfoil shape based on new thickness 
        ScaleAirfoil(sprintf('%s.dat',optim.(entity).structure.airfoil{i}),optim.(entity).toc(i)); 
        
    end
    
else
    
    % if aero isn't specified, assume it's a boom/pod with a circular
    % profile:
	
	optim.(entity).structure.airfoil = {'circle','circle'};
    x = [1:-0.025:0, 0.025:0.025:1];
    y = sqrt(0.5^2-(x-0.5).^2);
    y(find(x==0):end) = -y(find(x==0):end);
    coords = [x' y'];
    
    % write circle shape file:
    fid = fopen(sprintf('%s.dat',optim.(entity).structure.airfoil{1}), 'w');
    fprintf(fid,'%s\n',optim.(entity).structure.airfoil{1});
    for j = 1:length(coords)
        fprintf(fid,'%f %f\n',coords(j,1), coords(j,2));
    end
    fclose(fid);
    
end    

end