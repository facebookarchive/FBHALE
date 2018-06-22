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
  
function  [maxRelNormalStress, maxRelTransverseStress, maxRelShearStress, maxRelBucklingLoad, maxRelTipDeflection, maxTwist_deg, optim]  = ...
    StressesAndDeflections(optim)
% This routine runs the airframe through loads and determines peak stresses
% from coblade. 

% Get Loads from ASWING:
optim = RunASWINGLoads(optim,'wing',[1,1,1,1]);
optim = CoBladeLoadsWing(optim);

% Calculate stresses
optim.wing.structure.recordMaxValues = true;
optim = UpdateMassProperties(optim);

maxRelNormalStress     = max(optim.wing.structure.s11TFailureCriteria,optim.wing.structure.s11CFailureCriteria);
maxRelTransverseStress = max(optim.wing.structure.s22TFailureCriteria,optim.wing.structure.s22CFailureCriteria);
maxRelShearStress      = optim.wing.structure.s12SFailureCriteria;
maxRelTipDeflection    = optim.wing.structure.MaxTipDeflection;
maxTwist_deg           = optim.wing.structure.MaxTwist;
maxRelBucklingLoad     = optim.wing.structure.maxBucklingStress;

%% Print Report

optim = UpdateMassProperties (optim);
StructuralMass_kg = 0;

fprintf('================================================================== \n');
for i = 1:length(optim.aircraft.beamNames)
    
entity = optim.aircraft.beamNames{i};    
StructuralMass_kg = StructuralMass_kg + optim.(entity).structure.mass_kg*optim.(entity).N(:);

fprintf('%s Structure: \n', regexprep(lower(entity),'(\<[a-z])','${upper($1)}'));
fprintf('Structural weight: %f kg\n', optim.(entity).structure.mass_kg);
if isfield (optim.(entity).structure, 's11TFailureCriteria')
fprintf('Normal stress criteria: %f  (Failure: >1) \n',max(optim.(entity).structure.s11TFailureCriteria,optim.(entity).structure.s11CFailureCriteria));
fprintf('Transverse stress criteria: %f  (Failure: >1) \n',max(optim.(entity).structure.s22TFailureCriteria, optim.(entity).structure.s22CFailureCriteria));
fprintf('Shear stress criteria: %f  (Failure: >1) \n', optim.(entity).structure.s12SFailureCriteria);
fprintf('Buckling criteria: %f  (Failure: >1) \n', optim.(entity).structure.maxBucklingStress);
fprintf('Max. normalized tip deflection (w.r.t semispan): %f\n', optim.(entity).structure.MaxTipDeflection);
fprintf('Max. twist (deg): %f \n', optim.(entity).structure.MaxTwist);
end
fprintf('------------------------------------------------------------------ \n');
end
fprintf('Total structural beam weight: %f kg, %2.2f%% MGTOW\n', StructuralMass_kg, StructuralMass_kg/optim.MGTOW_kg*100);
fprintf('================================================================== \n');
end

