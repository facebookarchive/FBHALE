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
  
function optim = AssignDesignVariables(params)
% This routine assigns all design variables to the global optim data 
% structure. 

% Unpack params
v2struct(params);

optim.aircraftName = aircraftName;
optim.designID 	= designID;
optim.MGTOW_kg	= MGTOW_kg;

optim.wing.twists_deg       = [0 twists_deg];

% wingToCIndices is an index variable over the selected airfoil database.
% At the root, 0 means lowest thickness available and 1 means maximum. 
% At the next section, 0 still means lowest thickness available and 1 means
% the same as the root. And so on and so forth....
optim.wing.toc 				= [wingToCIndices(1) wingToCIndices].*[1 1 wingToCIndices(1:2)].*[1 1 1 wingToCIndices(3)];
optim.wing.taperRatios      = [1 taperRatios];
optim.wing.sweep_deg        = sweep_deg;
optim.wing.aero.CLsizing   	= CL;
optim.wing.wingLoading_kgm2 = wingLoading_kgm2;
optim.wing.AR               = AR;
optim.wing.yobo2            = [0 yWingBreakobo2 1];
optim.batteries.yobo2       = [0 ybatteriesobo2];
optim.solar.relativeHorizontalArea  = percentHorizontalSolarPanel;
optim.solar.relativeVerticalArea     = percentVerticalSolarPanel;

optim.boom.lob = relativeBoomLength;

optim.wing.structure.UD_nPly                = [rootUD_nPly tipUD_nPly];
optim.wing.structure.boxSkin_nPly           = [rootBoxSkin_nPly tipBoxSkin_nPly];
optim.wing.structure.sparCapWidthInboard  	= sparCapWidth;
optim.wing.structure.sparCapWidthOutboard  	= sparCapWidth*sparCapWidthTaperRatio;
optim.wing.structure.pitchAxis            	= pitchAxis;
optim.wing.structure.sparCap_core_nPly      = sparCapCore_nPly;
optim.wing.structure.web_core_nPly          = webCore_nPly*[1 1];

optim.htail.VH = VH;

optim.mission.lat_deg      = latitude_deg;
optim.mission.minh_m  	   = minh_m;

optim.solar.inputSelectionIndex       = solarSelectionIndex;
optim.batteries.inputSelectionIndex   = batterySelectionIndex;
optim.techOperatingMode               = techType;

optim.propulsion.propeller.CLroot = propellerCLroot;
optim.propulsion.propeller.CLtip  = propellerCLtip;
optim.propulsion.propeller.RPMmargin = propellerRPMmargin;

end
