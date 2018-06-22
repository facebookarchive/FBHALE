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
  
function [ optim ] = GetSolarModuleDimensions( optim, entity )
% Finds the solar module size to maximize the total solar area achievable on
% a given surface given dimensions of the raw cells and the specified 
% spanwise and chordwise module dimensions limitations.

 n_max = ceil(optim.solar.module.maxNPanels /optim.solar.module.chordwiseMaxPanels);
 n_min = floor(optim.solar.module.chordwiseMinPanels/optim.solar.module.maxNPanels);

 m = optim.solar.module.chordwiseMaxPanels:optim.solar.module.chordwiseMinPanels;
 n = n_min:n_max;
 
 nroll = [];
 mroll = [];
 cellroll = [];
 
 for i = 1:length(m)
    cells_panel = m(i)*n;
    cellroll = [cellroll cells_panel];
    nroll = [nroll n];
    mroll = [mroll repmat(m(i),length(n),1)'];

 end
 
 m = mroll(cellroll> optim.solar.module.minNPanels & cellroll< optim.solar.module.maxNPanels);
 n = nroll(cellroll> optim.solar.module.minNPanels & cellroll< optim.solar.module.maxNPanels);
 
 chordwisePanelLength_m = m*optim.solar.length_m; 
 spanwisePanelLength_m  = n*optim.solar.width_m;
 
 area = [];
 for i = 1:length(m)
    area(end+1) =  GetMaxSolarArea(optim, chordwisePanelLength_m(i), spanwisePanelLength_m(i), entity); 
 end
     
 [~, index] = max(area);
 optim.(entity).solar.module.chordwiseLength_m = chordwisePanelLength_m(index);
 optim.(entity).solar.module.spanwiseLength_m  = spanwisePanelLength_m(index);
 
end

function [maxPossibleSolarArea] = GetMaxSolarArea(optim, m_m, n_m, entity) 
% Finds max solar area achievable on a surface.
nPanels = 0;
for i = 1 : length(optim.(entity).c_m)-1
    airfoilSuctionSideRelativeLength = sum(sqrt(abs((diff(optim.(entity).aero.airfoilADB.geom.xySuction{1}(:,1)))).^2+(abs(diff(optim.(entity).aero.airfoilADB.geom.xySuction{1}(:,2)))).^2));
        
    nPanelsLoc = GetNPanels(optim.(entity).c_m(i)*airfoilSuctionSideRelativeLength, optim.(entity).c_m(i+1)*airfoilSuctionSideRelativeLength, optim.(entity).yobo2(i+1)*optim.(entity).bref_m/2 - optim.(entity).yobo2(i)*optim.(entity).bref_m/2, m_m , n_m);
    nPanels = nPanels + nPanelsLoc; 
end

maxPossibleSolarArea =  nPanels*m_m*n_m;

end

function [nPanels] = GetNPanels(rootChord_m, tipChord_m, span_m, chordwiseLength_m, spanwiseLength_m )
% This function finds the maximum number of solar modules of a given
% dimension that can be fit onto a surface.

if rootChord_m == tipChord_m
   nPanels =  floor(rootChord_m/chordwiseLength_m)*floor(span_m/spanwiseLength_m);
else
   dChord_dSpan = abs(rootChord_m-tipChord_m)/span_m;
   nPanels = 0;
   for  i = 1:floor(span_m/spanwiseLength_m)
       localChord_m = max([rootChord_m tipChord_m])-spanwiseLength_m*dChord_dSpan*i;
       nPanelsLoc   = floor(localChord_m/chordwiseLength_m);
       nPanels = nPanels + nPanelsLoc;
   end
end

end
