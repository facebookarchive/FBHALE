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

findIntersection subfunction based on Lufuno Vhengani' Lines Intersection 
algorithm with the following license:

Copyright (c) 2011, Lufuno Vhengani
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are
met:

* Redistributions of source code must retain the above copyright
notice, this list of conditions and the following disclaimer.
* Redistributions in binary form must reproduce the above copyright
notice, this list of conditions and the following disclaimer in
the documentation and/or other materials provided with the distribution
* Neither the name of the CSIR nor the names
of its contributors may be used to endorse or promote products derived
from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
POSSIBILITY OF SUCH DAMAGE.
%}
  
function optim = SetInterfaceMasses (optim)
% This function places interfaces masses based on beam intersections.

%% Find global coordinates of beams 
optim = BuildAircraftGeometry (optim);

%% Collect coordinates of individual beams
numBeams = 0;
beamNames = optim.aircraft.beamNames;

for i = 1:length(beamNames)
    for j = 1:length(optim.(beamNames{i}).globalx_m)   
      numBeams = numBeams + 1;  
      % Coords of beam pair, choose whether horizontal or vertical surface
      if abs(diff(optim.(beamNames{i}).globalz_m{j}(1:2))) > 1e-3
      beam(numBeams).Coords(1,:) = [min(optim.(beamNames{i}).globalx_m{j}) min(optim.(beamNames{i}).globalz_m{j})+optim.(beamNames{i}).y_m-optim.(beamNames{i}).z_m]; 
      beam(numBeams).Coords(2,:) = [max(optim.(beamNames{i}).globalx_m{j}) max(optim.(beamNames{i}).globalz_m{j})+optim.(beamNames{i}).y_m-optim.(beamNames{i}).z_m]; 
      else
      beam(numBeams).Coords(1,:) = [min(optim.(beamNames{i}).globalx_m{j}) min(optim.(beamNames{i}).globaly_m{j})]; 
      beam(numBeams).Coords(2,:) = [max(optim.(beamNames{i}).globalx_m{j}) max(optim.(beamNames{i}).globaly_m{j})]; 
      end
      beam(numBeams).beamID = [beamNames{i} num2str(j)];
      beam(numBeams).beamName = beamNames{i};
    end
end

%% Find intersection locations and weights and add to pointmass list

joints = nchoosek (1:numBeams, 2);

for i = 1:length(joints)
   interfaceWeightName = ['interface' beam(joints(i,1)).beamID '' beam(joints(i,2)).beamID];
   beam1Coords = round(beam(joints(i,1)).Coords, 12);
   beam2Coords = round(beam(joints(i,2)).Coords, 12);
   
   % Find intersection point
   [interfaceLocationX interfaceLocationY] = findIntersection(beam1Coords, beam2Coords); 
   
   if isnan(interfaceLocationX) || isnan(interfaceLocationY)
           N = 0;
   elseif interfaceLocationY < 1e-6
           interfaceLocationY = 0;
           N = 1;
   else
           N = 2;
   end
   
   % add to point mass list if it's a valid intersection
   if N > 0 && ~isfield(optim, interfaceWeightName)
      % Specify weight based on weight scaling
      [ChildWeight, idx] = min ([optim.(beam(joints(i,1)).beamName).structure.mass_kg optim.(beam(joints(i,2)).beamName).structure.mass_kg]);     
      % Add pointmass to the mix
      optim.aircraft.pointMassNames         = [optim.aircraft.pointMassNames interfaceWeightName];
      optim.(interfaceWeightName).mass_kg   = ChildWeight*optim.interfaceWeightScale;
      optim.(interfaceWeightName).N         = N;         
      optim.(interfaceWeightName).zCG_m     = 0;
      optim.(interfaceWeightName).xCG_m     = interfaceLocationX;
      optim.(interfaceWeightName).y_m       = interfaceLocationY;  
   end
   
end
end

function [x y] = findIntersection(L1,L2)

    x1=L1(1,1);y1=L1(1,2);x2=L1(2,1);y2=L1(2,2); 
    x3=L2(1,1);y3=L2(1,2);x4=L2(2,1);y4=L2(2,2);
    x = det([det([x1 y1;x2 y2]), (x1-x2);det([x3 y3;x4 y4]), (x3-x4) ])/det([(x1-x2),(y1-y2) ;(x3-x4),(y3-y4)]);
    y = det([det([x1 y1;x2 y2]), (y1-y2);det([x3 y3;x4 y4]), (y3-y4) ])/det([(x1-x2),(y1-y2) ;(x3-x4),(y3-y4)]);

    if ~isPointWithinLine(L1(1,1), L1(1,2), L1(2,1), L1(2,2), x, y) || ~isPointWithinLine(L2(1,1), L2(1,2), L2(2,1), L2(2,2), x, y)
    x = NaN;
    y = NaN;
    end   
end

function R = isPointWithinLine(x1, y1, x2, y2, x3, y3)
    tol = 1e3*eps;
    if x1 ~= x2 
      m   = (y2-y1) / (x2-x1);
      if m~=0
      y3e = m*x3 + y1 - m*x1;
      R   = (abs(y3 - y3e) < 1e-6);
      else
      R   =  (x3 >= min(x1,x2)-tol) & (x3 <= max(x1,x2)+tol) & (abs(y1-y3)<tol);
      end
    else
      R   = (y3 >= min(y1,y2)-tol) & (y3 <= max(y1,y2)+tol) & (abs(x1-x3)<tol);
    end
end

  