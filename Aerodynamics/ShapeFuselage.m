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
  
function optim = ShapeFuselage(optim)
% Sizes fuselage body to trailing edge of the wing based on the x_m position of
% the propulsion system. Uses fuselage body shape.
% Populates to following values in optim
%  optim.wing.z_m      scalar with wing z location     (meters)
%  optim.boom.sectionsX_m      x location of fuselage sections (meters)
%  optim.boom.sectionsY_m      y location of fuselage sections (meters)
%  optim.boom.sectionsZ_m      z location of fuselage sections (meters)
%  optim.boom.sectionsR_m      boom radius at cross sections   (meters)
 
% If there are no booms, then assume the fuselage is a pod

%% Specify Fuselage Body Shape 

if max(contains(optim.aircraft.beamNames,'boom')) == 0
    fuselage = 'pod';
else
    fuselage = 'boom';
end

% assume first index optim.(fuselage).N indicates fuselage. 
idx = 1;

% Scale up fuselage so wing beam axis sits at .55 on fuselage
scaleFactor    =  optim.wing.x_m/optim.(fuselage).attachmentLocation;
x_sections     =  optim.(fuselage).shape.X*scaleFactor;
r_sections     =  optim.(fuselage).shape.Y*scaleFactor;

% Check if fuselage minimum radius is enforced, if so scale up fuselage
if max(r_sections) < optim.(fuselage).minRadius_m(idx) 
    scaleFactor    =  optim.(fuselage).minRadius_m(idx)/max(optim.(fuselage).shape.Y);
    x_sections     =  optim.(fuselage).shape.X*scaleFactor;
    r_sections     =  optim.(fuselage).shape.Y*scaleFactor;
end

if max(r_sections) > optim.(fuselage).maxRadius_m(idx)   
    scaleFactor    =  optim.(fuselage).maxRadius_m(idx)/ max(r_sections) ;
    x_sections     =  x_sections * scaleFactor;
    r_sections     =  r_sections * scaleFactor;
end
    x_sections(1) =  0;
    y_sections	  =  x_sections*0;
    z_sections    =  x_sections*0;

% Find geometric properties
optim.(fuselage).Scross_m2(idx)          =  trapz(x_sections, r_sections)*2;             %cross sectonal area 
optim.(fuselage).xAreaCentroid_m(idx)    =  interp1(cumtrapz(x_sections, r_sections), x_sections, trapz(x_sections, r_sections)/2); % area centroid
optim.(fuselage).V_m3(idx)               =  trapz(x_sections, pi*r_sections.^2);         %boom volume
optim.(fuselage).Swet_m2(idx)            =  2*pi*trapz(x_sections, r_sections);          % wetted area
optim.(fuselage).fuselage.length_m(idx)  =  max(x_sections);
optim.(fuselage).omlSectionsX_m{idx}        =  linspace (0, max(x_sections), optim.(fuselage).structure.numStations)';    
optim.(fuselage).omlSectionsY_m{idx}        =  interp1(x_sections,y_sections,optim.(fuselage).omlSectionsX_m{idx});
optim.(fuselage).omlSectionsZ_m{idx}        =  interp1(x_sections,z_sections,optim.(fuselage).omlSectionsX_m{idx});
optim.(fuselage).omlSectionsR_m{idx}        =  interp1(x_sections,r_sections,optim.(fuselage).omlSectionsX_m{idx});
optim.(fuselage).structure.fuselageCanopySectionsR_m{idx} = optim.(fuselage).omlSectionsR_m{idx};
optim.(fuselage).structure.chord_m{idx}  =  optim.(fuselage).omlSectionsR_m{idx}(end)*2*ones(optim.(fuselage).structure.numStations,1);
if strcmp(fuselage, 'pod')
optim.pod.structure.length_m(idx) =  max(x_sections);
end

% Move wing Z to match fuselage radius
[optim.wing.z_m, maxIndex]   =  max(r_sections);

%% If Boom Exists: Specify Geometry following fuselage shape

if strcmp(fuselage, 'boom')
    
    for idx = 1:length(optim.boom.N)
    
    % create cylinder 
    x_sections = optim.boom.omlSectionsX_m{idx};
    r_sections = optim.boom.omlSectionsR_m{idx};
    [~, maxIndex]   =  max(r_sections);
    optim.boom.structure.length_m(idx) = max(optim.htail.x_m, optim.vtail.x_m);
    if optim.boom.structure.length_m > x_sections(end)
    addX = linspace(x_sections(end)+0.1*(x_sections(end)+x_sections(end-1)), optim.boom.structure.length_m(idx), 10);
    addR = ones(1,10)*r_sections(end);
    else
        warning('Tails inside fuselage canopy');
    end
        
    % Append OML with cylinder
    x_sections	  =  [x_sections; addX'];   
    r_sections	  =  [r_sections; addR'];
    y_sections	  =  x_sections*0;
    z_sections    =  x_sections*0;
    
    optim.boom.maxActualRadius_m = max(r_sections);
    optim.boom.dMaximum_m  =  max(r_sections);
    optim.boom.xofdMaximum =  x_sections(maxIndex);
   
    % Calculate wetted area of the aft cylinder
    chord = addR*2;
    sparWidth = linspace(optim.boom.structure.sparCapWidthInboard, optim.boom.structure.sparCapWidthOutboard, 10);
    
    % Angular distance from chord to intersection of curved and straight
    % portion in the top right corner. 
    theta = acos (sparWidth);

    % Area of curved sides:
    curvedLength = (pi-2*theta).*chord;

    % Area of flat sides:
    flatSideLength = 2*chord.*sin(theta);

    % Total area:
    optim.boom.aftWettedArea_m2(idx) = trapz(addX, curvedLength + flatSideLength);

    % interpolate OML onto structural grid:
    optim.boom.omlSectionsX_m{idx} =  linspace (0, max(x_sections), optim.boom.structure.numStations)';    
    optim.boom.omlSectionsY_m{idx} =  interp1(x_sections,y_sections,optim.boom.omlSectionsX_m{idx});
    optim.boom.omlSectionsZ_m{idx} =  interp1(x_sections,z_sections,optim.boom.omlSectionsX_m{idx});
    optim.boom.omlSectionsR_m{idx} =  interp1(x_sections,r_sections,optim.boom.omlSectionsX_m{idx});
    [~,aftBoomStartIdx] = min(abs(optim.boom.fuselage.length_m(idx) - optim.boom.omlSectionsX_m{idx}));
    optim.boom.structure.fuselageCanopySectionsR_m{idx} = optim.boom.omlSectionsR_m{idx}(1:aftBoomStartIdx);
    % specify constant chord for internal structure 
    optim.boom.structure.chord_m{idx} = optim.boom.omlSectionsR_m{idx}(end)*2*ones(optim.boom.structure.numStations,1);

    end
end

%% Define Non-Fuselage Pod Shapes

if max(contains(optim.aircraft.beamNames,'pod'))   
    
    if length(optim.pod.N) == 2
        idx = 2;
    else
        idx = 1;
    end
    
    scaleFactor    =  optim.pod.maxRadius_m(idx)/max(optim.pod.shape.Y) ;
    x_sections     =  optim.pod.shape.X*scaleFactor;
    r_sections     =  optim.pod.shape.Y*scaleFactor;
    x_sections(1)  =  0;
    y_sections	   =  x_sections*0;
    z_sections     =  x_sections*0;
        
    % make OML
    optim.pod.Scross_m2(idx)          = trapz(x_sections, r_sections)*2;             %cross sectonal area 
    optim.pod.xAreaCentroid_m(idx)    = interp1(cumtrapz(x_sections, r_sections), x_sections, trapz(x_sections, r_sections)/2); % area centroid
    optim.pod.V_m3(idx)               = trapz(x_sections, pi*r_sections.^2);         %boom volume
    optim.pod.Swet_m2(idx)            = 2*pi*trapz(x_sections, r_sections);          % wetted area
    optim.pod.fuselage.length_m(idx)  = max(x_sections);
    optim.pod.structure.length_m(idx) = max(x_sections);
    optim.pod.omlSectionsX_m{idx}        =  linspace (0, max(x_sections), optim.pod.structure.numStations)';    
    optim.pod.omlSectionsY_m{idx}        =  interp1(x_sections,y_sections,optim.pod.omlSectionsX_m{idx});
    optim.pod.omlSectionsZ_m{idx}        =  interp1(x_sections,z_sections,optim.pod.omlSectionsX_m{idx});
    optim.pod.omlSectionsR_m{idx}        =  interp1(x_sections,r_sections,optim.pod.omlSectionsX_m{idx});
    optim.pod.structure.chord_m{idx}  =  optim.pod.omlSectionsR_m{idx}(end)*2*ones(optim.pod.structure.numStations,1);       
    optim.pod.structure.fuselageCanopySectionsR_m{idx} = optim.pod.omlSectionsR_m{idx};
end