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
  
function [ optim ] = WritePointMasses( optim, aswingIn )
% Places point masses in aswing file
% Any point mass with a non zero y_m is assumed to be reflected

tolerance = .000001;
% Get battery mass
optim = UpdateMassProperties(optim);

% In-line function to fetch/add masses
AddPointMassAswingFile = @(aswingFile, beam, t, x_m, y_m, z_m, weight_N) fprintf(aswingIn, '\t%3.4f\t%3.4f\t%3.4f\t%3.4f\t%3.4f\t%3.4f\t%3.4f\t%3.4f\t%3.4f\t%3.4f\t%3.4f\n', ...
                                                                            beam, t, x_m, y_m, z_m, weight_N, 0, 0, 0, 0, 0);
% Header
fprintf(aswingIn,'#======================\n');
fprintf(aswingIn,'Weight\n');
fprintf(aswingIn,'#  Nbeam     t      Xp      Yp      Zp      Mg      CDA     Vol     Hx      Hy      Hz\n');

% Harvest all span-wise locations of considered masses and lump equivalent
% mass for each y location
ys_m = [];
for m = 1:length(optim.aircraft.pointMassNames)
    [~, ~, yMass_m, ~] = GetMass(optim.aircraft.pointMassNames{m}, optim);
    ys_m = [ys_m yMass_m];
end
ys_m = unique(abs(ys_m));

% Harvest all x-locations of lifting beams to compare against point mass
% locations and appropriate pairing
for b = 1:length(optim.aircraft.beamNames)
    if strcmp(optim.aircraft.beamNames{b}, 'boom') || strcmp(optim.aircraft.beamNames{b}, 'pod')% Attach -Inf value to booms to avoid pairing
        xBeams_m(b) = -Inf;
    else
        xBeams_m(b) = optim.(optim.aircraft.beamNames{b}).xCG_m;
    end
end

% For each y-location, lump the masses and attach to the appropriate beam
for y = 1:length(ys_m)   
    
    % Initialize
    masses_kg   = [];
    massesLocX_m = [];
    massesLocZ_m = [];
    massesLocY_m = [];
    for m = 1:length(optim.aircraft.pointMassNames)   
        [masses_kg(end+1), massesLocX_m(end+1), massesLocY_m(end+1), massesLocZ_m(end+1)] = GetMass(optim.aircraft.pointMassNames{m}, optim, abs(ys_m(y)));
    end
        
    % Equivalent lumped mass
    [equivalentMass_kg, eqMassX_m] = GetEquivalentMass(masses_kg, massesLocX_m);
    [~,                 eqMassZ_m] = GetEquivalentMass(masses_kg, massesLocZ_m);
    [~,                 eqMassY_m] = GetEquivalentMass(masses_kg, massesLocY_m);
    
    eqWeight_N                     = equivalentMass_kg * optim.constants.g_ms2;
    
    % If mass y is equal to a boom/pod y, then attach to the closest boom/pod
    % and use the non-dimentional y as the t index. Otherwise, attach to
    % the closest lifting beam e.g. wing.
    
    if (isfield(optim.boom,'y_m') && max(abs(ys_m(y)-optim.boom.y_m)<tolerance)) || (max(abs(ys_m(y)-optim.pod.y_m)<tolerance) && any((find(cell2mat(strfind(optim.aircraft.beamNames, 'pod'))) == 1)))
        if (isfield(optim.boom,'y_m') && max(abs(ys_m(y)-optim.boom.y_m)<tolerance))
            beam = 'boom';
            x0Location_m = optim.(beam).omlSectionsX_m{find(abs(optim.(beam).y_m - ys_m(y))<tolerance)}(1);
        else
            beam = 'pod';
            %saxisX_m = interp1(optim.wing.structure.sRefY_m, optim.wing.globalx_m{1} , optim.batteries.yobo2(2)*optim.wing.bref_m/2);
            %podWingInterfaceLocation_m = max(optim.pod.sectionsX_m)*optim.boom.attachmentLocation;
            %podsectionsX_mLoc = optim.pod.sectionsX_m + saxisX_m - podWingInterfaceLocation_m ; %should be 1/4 chord not s axis
            %x0Location_m = min(podsectionsX_mLoc);
            x0Location_m = optim.(beam).x_m(find(abs(optim.(beam).y_m - ys_m(y))<tolerance));
        end
        characteristicLength_m = optim.(beam).structure.length_m(find(abs(optim.(beam).y_m - ys_m(y))<tolerance));
        tIndex    = (eqMassX_m - x0Location_m)/ characteristicLength_m;
        tReflection= 1;
        beamIndexC = optim.(beam).beamNumber(abs(ys_m(y) - optim.(beam).y_m)<tolerance);
        beamIndexR = beamIndexC;
        beamIndexL = beamIndexC + 1;
        
    else
        % Find closest lifting beam
        % Account for sweep
        for i = 1:length(optim.aircraft.beamNames)
            if isfield(optim.(optim.aircraft.beamNames{i}), 'sweep_deg')
                xBeams_m(i) = xBeams_m(i) + eqMassX_m*tand((optim.(optim.aircraft.beamNames{i}).sweep_deg));
            end
            %get beam y location, if beam expends to desired y location
            %then ybea = location otherwise its the tip
            if isfield(optim.(optim.aircraft.beamNames{i}), 'bref_m')
                if optim.(optim.aircraft.beamNames{i}).y_m == 0
                    if eqMassY_m > optim.(optim.aircraft.beamNames{i}).bref_m/2
                        yBeams_m(i) = optim.(optim.aircraft.beamNames{i}).bref_m/2;
                    else
                        yBeams_m(i) = eqMassY_m;
                    end
                else
                    if eqMassY_m > optim.(optim.aircraft.beamNames{i}).bref_m/2+optim.(optim.aircraft.beamNames{i}).y_m || eqMassY_m < optim.(optim.aircraft.beamNames{i}).y_m-optim.(optim.aircraft.beamNames{i}).bref_m/2
                        if eqMassY_m > optim.(optim.aircraft.beamNames{i}).bref_m/2+optim.(optim.aircraft.beamNames{i}).y_m
                            yBeams_m(i) = optim.(optim.aircraft.beamNames{i}).bref_m/2+optim.(optim.aircraft.beamNames{i}).y_m;
                        else
                            yBeams_m(i) = optim.(optim.aircraft.beamNames{i}).y_m-optim.(optim.aircraft.beamNames{i}).bref_m/2;
                        end
                    else
                        yBeams_m(i) = eqMassY_m;
                    end
                end
            else
                yBeams_m(i) = -Inf;
            end
        end
        [~, lifttingSurfaceIndex] = min((abs(xBeams_m - eqMassX_m)).^2+(abs(yBeams_m - eqMassY_m).^2));
        characteristicLength_m = optim.(optim.aircraft.beamNames{lifttingSurfaceIndex}).bref_m/2;
        tIndex     = (ys_m(y)-optim.(optim.aircraft.beamNames{lifttingSurfaceIndex}).y_m) / characteristicLength_m;
        tReflection= -1;
        if(max(size(optim.(optim.aircraft.beamNames{lifttingSurfaceIndex}).beamNumber)) == 1)
            beamIndexC = optim.(optim.aircraft.beamNames{lifttingSurfaceIndex}).beamNumber;
            beamIndexR = optim.(optim.aircraft.beamNames{lifttingSurfaceIndex}).beamNumber;
            beamIndexL = optim.(optim.aircraft.beamNames{lifttingSurfaceIndex}).beamNumber;
        elseif (max(size(optim.(optim.aircraft.beamNames{lifttingSurfaceIndex}).beamNumber)) == 2)
            beamIndexR = optim.(optim.aircraft.beamNames{lifttingSurfaceIndex}).beamNumber(1);
            beamIndexL = optim.(optim.aircraft.beamNames{lifttingSurfaceIndex}).beamNumber(2);
        end
    end
    
    % If some masses are further apart from the CG of the
    % equivalent mass by more than 20% of the associated beam length, treat
    % as a separate mass
    if max(abs(eqMassX_m - massesLocX_m(1, :)).*(masses_kg(1, :)~=0)) >= 0.2 * characteristicLength_m
        while max(abs(eqMassX_m - massesLocX_m(1, :)).*(masses_kg(1, :)~=0)) >= 0.2 * characteristicLength_m
            tIndex = [];
            [~, indexOutlier] = max(abs(eqMassX_m - massesLocX_m(1, :)).*(masses_kg(1, :)~=0));
            
            % Move the outlier to the following line i.e new lumped mass
            masses_kg(end+1, indexOutlier) = masses_kg(1, indexOutlier);
            masses_kg(1,     indexOutlier) = 0;
            massesLocX_m(end+1, indexOutlier) = massesLocX_m(1, indexOutlier);
            massesLocX_m(1,   indexOutlier) = 0;
            massesLocZ_m(end+1, indexOutlier) = massesLocZ_m(1, indexOutlier);
            massesLocZ_m(1,   indexOutlier) = 0;
            
            % Re-lump
            [equivalentMass_kg, eqMassX_m] = GetEquivalentMass(masses_kg(1, :), massesLocX_m(1, :));
            [~,                 eqMassZ_m] = GetEquivalentMass(masses_kg(1, :), massesLocZ_m(1, :));
            eqWeight_N                     = equivalentMass_kg * optim.constants.g_ms2;
            reT = 1;
        end
        
        %If masses were singled out, add them to the list
        [equivalentMass_kg, eqMassX_m] = GetEquivalentMass(masses_kg, massesLocX_m);
        [~                , eqMassZ_m] = GetEquivalentMass(masses_kg, massesLocZ_m);
        eqWeight_N                     = equivalentMass_kg * optim.constants.g_ms2;
        tIndex = eqMassX_m./characteristicLength_m; 
    end
    
    for i = 1:length(eqMassX_m)
        % Add to ASWING file
        if ys_m(y) == 0
            AddPointMassAswingFile(aswingIn, beamIndexC, tIndex(i), eqMassX_m(i), ys_m(y), eqMassZ_m(i), eqWeight_N(i));
        else % Reflect to the negative boom
            AddPointMassAswingFile(aswingIn, beamIndexR, tIndex(i), eqMassX_m(i), ys_m(y), eqMassZ_m(i), eqWeight_N(i)/2);
            AddPointMassAswingFile(aswingIn, beamIndexL, tReflection * tIndex(i), eqMassX_m(i), -ys_m(y), eqMassZ_m(i), eqWeight_N(i)/2);
        end
    end
end

% Close file
fprintf(aswingIn,'End\n'); 
end

% Retrieve the mass and its x-location provided it is at a provided y
% value
function [mass_kg, x_m, y_m, z_m] = GetMass(massName, optim, varargin) 

% If an extra argument is passed, then the targeted y value is given
if ~isempty(varargin)
    y_m = varargin{1};
else
    y_m = [];
end
    
% If mass name contains a '.' this means it is nested in the optim
% structure
if strfind(massName, '.')
    index    = strfind(massName, '.');
    massPath = optim.(massName(1:index-1)).(massName(index+1:end));
else
    massPath = optim.(massName);
end

if isempty(y_m)
       y_m = massPath.y_m;
end

% Retrieve mass corresponding to provided y location
IsMassAtY = @(massPath, y_m) find(massPath.y_m == y_m);
mass_kg   = massPath.mass_kg(IsMassAtY(massPath, y_m)) .* massPath.N(IsMassAtY(massPath, y_m));

if isempty(mass_kg)
    mass_kg = 0;
    x_m = 0;
    z_m = 0;
    
else
    x_m = massPath.xCG_m(IsMassAtY(massPath, y_m))*isreal(massPath.xCG_m(IsMassAtY(massPath, y_m))) + ~isreal(massPath.xCG_m(IsMassAtY(massPath, y_m)))*optim.xCG_m;
    z_m       = massPath.zCG_m(IsMassAtY(massPath, y_m));
end
end