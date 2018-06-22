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
  
function optim = UpdateMassProperties(optim, varargin)
% This function computes the current mass of all components (beams and
% point masses) as well as the CG of the aircraft. The summed mass is
% computed and the difference with the assigned MGTOW is stored.
% Beam masses can be computed in two different manners:
% varargin = 'co-blade': weight computation using precomp
% varargin = 'analytical': weight computation using hand-calculations for
%                          quicker evaluation while used in an optimizer.

if isempty(varargin)
    varargin = {'co-blade'};
end

% If margin weight hasn't been calculated yet, do so. It's location is at
% the aircraft CG, which is flagged by the imaginary number i
if ~isfield(optim.margin, 'mass_kg')
    optim.margin.mass_kg  = optim.MGTOW_kg*optim.margin.weight_percent; 
    optim.margin.xCG_m      = 1i; 
    optim.margin.N          = 1;
end

% Find structural component-wise CGs
optim = ComputeMassDistribution(optim, varargin{1});

% Find solar-cell distribution
optim = GetSolarMassDistribution(optim);

% Update locations of point masses
optim = UpdateMassLocations(optim);

% Update locations of interface masses
optim = SetInterfaceMasses(optim);

% Find harness mass
optim = GetHarnessMass(optim);

% Get Point Mass CG and total mass
[PMass_kg, PMassXLocation_kgm]     = GetMasses(optim.aircraft.pointMassNames, 'point-mass', optim);

% Get Beam Mass CG and total mass
[BMass_kg, BMassXLocation_kgm]     = GetMasses(optim.aircraft.beamNames, 'beam', optim);                                              

% Weight build-up
optim.MGTOWcurrent_kg = BMass_kg + PMass_kg;
optim.batteries.weighResidual_kg = optim.MGTOW_kg - optim.MGTOWcurrent_kg;

% CG-buildup. The imaginary part of the sum(mass*X-location) is the piece
% that is supposed to be at the CG i.e mass*X-location = mass * i
massXLocation_kgm   = BMassXLocation_kgm + PMassXLocation_kgm;
massAtCG_kg         = imag(massXLocation_kgm);
massXLocation_kgm   = real(massXLocation_kgm);
optim.xCG_m         = 1/(optim.MGTOWcurrent_kg - massAtCG_kg) * (massXLocation_kgm);
                
% Compute only structural weight from beams and pods
optim.structuralMass_kg = BMass_kg ;
end
