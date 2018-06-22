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
  
function [ optim ] = PlaceSensors(optim, operatingMode, aswingIn)
% this function places sensors in the .asw file based on the location of
% the structural input quantities.

%KS is sensor number and Nb is the beam the sensor is on
fprintf(aswingIn,'#============\n');
fprintf(aswingIn,'Sensor\n');
fprintf(aswingIn,'#   KS      Nb      t      Xp      Yp      Zp\n');
KS = 1;

% Wing Sensors

if strcmp(operatingMode, 'wingloads')
    entities = {'wing'};
elseif strcmp(operatingMode, 'tailloads')
    entities = {'htail', 'vtail', 'boom'};
elseif strcmp(operatingMode, 'polar')
    entities = {};
    for b = 1:length(optim.aircraft.beamNames)
        if ~max(strcmp(optim.aircraft.beamNames{b}, {'pod', 'boom'}))
            entities{end+1} = optim.aircraft.beamNames{b};
        end
    end
end

if strcmp(operatingMode, 'wingloads') || strcmp(operatingMode, 'tailloads') || strcmp(operatingMode, 'polar')
    for j = 1:length(entities)
        
        for n = 1:length(optim.(entities{j}).N)
            
            % if multiple instantes write to both surfaces
            if max(optim.(entities{j}).globaly_m{n})-min(optim.(entities{j}).globaly_m{n}) >  max(optim.(entities{j}).globalz_m{n})-min(optim.(entities{j}).globalz_m{n})
                verticalReflectionFactor = 1;
                horizontalReflectionFactor  = -1;
            else
                verticalReflectionFactor  = -1;
                horizontalReflectionFactor  = 1;
            end
            
            if optim.(entities{j}).N(n) == 2
                
                [KS, sensorX_m, sensorY_m, sensorZ_m] = ...
                    WriteSensorsToFile(aswingIn, KS, optim.(entities{j}).beamNumber(1) , ...
                    optim.(entities{j}).structure.spanLocation, optim.(entities{j}).globalx_m{n}, ...
                    optim.(entities{j}).globaly_m{n}, optim.(entities{j}).globalz_m{n}, entities{j});
                
                optim.(entities{j}).sensorLocationsX_m{n} = sensorX_m;
                optim.(entities{j}).sensorLocationsY_m{n} = sensorY_m;
                optim.(entities{j}).sensorLocationsZ_m{n} = sensorZ_m;
                
                % if loads place sensors on negative y instance of desired surface
                if strcmp(operatingMode, 'wingloads') || strcmp(operatingMode, 'tailLoads')
                    
                    % write negative y surface
                    [KS,sensorX_m, sensorY_m, sensorZ_m] = ...
                        WriteSensorsToFile(aswingIn, KS, optim.(entities{j}).beamNumber(2) , ...
                        optim.(entities{j}).structure.spanLocation, optim.(entities{j}).globalx_m{n}, ...
                        -1 * optim.(entities{j}).globaly_m{n}, optim.(entities{j}).globalz_m{n}, entities{j});
                    
                    optim.(entities{j}).sensorLocationsX_m{n}(2) = sensorX_m;
                    optim.(entities{j}).sensorLocationsY_m{n}{2} = sensorY_m;
                    optim.(entities{j}).sensorLocationsZ_m{n}{2} = sensorZ_m;
                    
                end
                
            else
                [KS, sensorX_m, sensorY_m, sensorZ_m] = ...
                    WriteSensorsToFile(aswingIn, KS, optim.(entities{j}).beamNumber(1) , ...
                    optim.(entities{j}).structure.spanLocation, optim.(entities{j}).globalx_m{n}, ...
                    optim.(entities{j}).globaly_m{n}, optim.(entities{j}).globalz_m{n}, entities{j});
                
                % record sensor locations
                optim.(entities{j}).sensorLocationsX_m{n} = sensorX_m;
                optim.(entities{j}).sensorLocationsY_m{n} = sensorY_m;
                optim.(entities{j}).sensorLocationsZ_m{n} = sensorZ_m;
            end
        end
        
    end
    
else
    % don't write a thing
end
fprintf(aswingIn,'End\n');
end

function [KS, sensorX_m, sensorY_m, sensorZ_m] = WriteSensorsToFile(aswingIn, KS, beamNumber , t, x_m, y_m, z_m, entity)

sensorX_m = [];
sensorY_m = [];
sensorZ_m = [];

for i = 1:length(t)
    
    if max(y_m)-min(y_m) >  max(z_m)-min(z_m)
        vertical = 1;
        horizontal = -1;
    else
        vertical = -1;
        horizontal = 1;
    end
    % write right side sensors
    fprintf(aswingIn,'\t%3.4f\t%3.4f\t%3.4f\t%3.4f\t%3.4f\t%3.4f\n',...
        KS,  beamNumber, t(i), x_m(i), y_m(i), (z_m(i)-z_m(1))+z_m(1));
    
    % Increment sensor value
    KS = KS+1;
    if ~strcmp(entity, 'boom')
        % write left side sensors
        fprintf(aswingIn,'\t%3.4f\t%3.4f\t%3.4f\t%3.4f\t%3.4f\t%3.4f\n',...
            KS,  beamNumber, -1*t(i), x_m(i), y_m(1)+(y_m(end)/abs(y_m(end)))*(abs(y_m(1))-abs(y_m(i))), vertical*(z_m(i)-z_m(1))+z_m(1));
        
        % Increment sensor value
        KS = KS+1;
        
        % record sensor locations
        sensorX_m(end+1) = x_m(i); sensorX_m(end+1) = x_m(i);
        sensorY_m(end+1) = y_m(i); sensorY_m(end+1) = y_m(1)+(y_m(end)/abs(y_m(end)))*(abs(y_m(1))-abs(y_m(i))) ;
        sensorZ_m(end+1) = z_m(i); sensorZ_m(end+1) = vertical*(z_m(i)-z_m(1))+z_m(1);
        
    else
        % record sensor locations
        sensorX_m(end+1) = x_m(i);
        sensorY_m(end+1) = y_m(i);
        sensorZ_m(end+1) = z_m(i);
    end
    
end

end

