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
  
% Sometimes ASW will drop outliers in both CL and CDi, try to protect
% against that by checking for large differences within a dataset.
function optim = RemoveASWOutliers(optim, surface)

% runs and aircraft CL, CDi, cl... are off.
foundOutlier = true;
outlierIndex = [];
CL           = optim.aircraft.aero.ASWINGpolar.CL;
AoA_deg      = optim.aircraft.aero.ASWINGpolar.AoA_deg;
CDi          = optim.aircraft.aero.ASWINGpolar.CDi;

while foundOutlier
   
    dCL = [0; diff(CL)];
    index = find(abs(dCL) > 5 * mean(abs(dCL)), 1, 'first');
    if ~isempty(index)
        CL(index)       = [];
        AoA_deg(index)  = [];
        CDi(index)      = [];
        outlierIndex(end+1) = index + length(outlierIndex);
    else
        foundOutlier = false;
    end
    
end

% The first time we find outliers, correct the aircraft aero quantities
if ~isempty(outlierIndex) && ~isfield(optim.aircraft.aero.ASWINGpolar, 'outlierIndex')
    
    % Interpolate over the offending points
  	optim.aircraft.aero.ASWINGpolar.CL(outlierIndex)  = interp1(AoA_deg, CL, optim.aircraft.aero.ASWINGpolar.AoA_deg(outlierIndex), 'linear', 'extrap');
  	optim.aircraft.aero.ASWINGpolar.CDi(outlierIndex) = interp1(AoA_deg, CDi, optim.aircraft.aero.ASWINGpolar.AoA_deg(outlierIndex), 'linear', 'extrap');
        
end
optim.aircraft.aero.ASWINGpolar.outlierIndex = outlierIndex;

% Then correct cl for the considered surface
if ~isempty(optim.aircraft.aero.ASWINGpolar.outlierIndex)
    
    outlierIndex = optim.aircraft.aero.ASWINGpolar.outlierIndex;
    
    for i = 1:length(optim.(surface).N)
        cl = optim.(surface).aero.ASWINGpolar.cl{i};
        AoA_deg  = optim.(surface).aero.ASWINGpolar.AoA_deg;
        cl(outlierIndex, :)   = [];
        AoA_deg(outlierIndex) = [];
        cl1                   = optim.(surface).aero.ASWINGpolar.cl{i};
        cl1(outlierIndex, :)  = interp1(AoA_deg, cl, optim.aircraft.aero.ASWINGpolar.AoA_deg(outlierIndex), 'linear', 'extrap');
        optim.(surface).aero.ASWINGpolar.cl{i} = cl1;
    end
    
end

end