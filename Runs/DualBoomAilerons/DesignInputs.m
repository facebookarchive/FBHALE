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
  
% This routine assigns all the design inputs for the design closure of a
% given configuration.

function [optim] = DesignInputs(optim)

% Configuration
optim.boom.N        = 2;
optim.wing.N        = 1;
optim.htail.N       = 2;
optim.vtail.N       = 2;
optim.pod.N         = 1;
optim.propulsion.N  = [2 2];

% If no boom is specified, remove any variables populated by the framework
if isfield(optim, 'boom') && ~isfield(optim.boom, 'N')
    optim = rmfield(optim, 'boom');
    optim.boom.N = 0;
    if isfield(optim, 'htail')
        optim = rmfield(optim, 'htail');
    end
    if isfield(optim, 'vtail')
        optim = rmfield(optim, 'vtail');
    end
end

%Names of all beams that make up the aircraft 
optim.aircraft.beamNames        = {'wing', 'htail', 'vtail', 'boom', 'pod'};
 optim.aircraft.joints           = {[7 1], [7 3], [7 5], [1 8]};
 optim.aircraft.beamNumbers      = {[1], [2 3], [4 5], [6 7], [8]};
%optim.aircraft.groundBeam       = 4;

%Specify the names of all point masses that need to rolled into the CG calc
%and into the aswing files. 
optim.aircraft.pointMassNames   = {'batteries','avionics', 'payload', ...
    'margin', 'propulsion', 'harness', 'landingGear', ...
    'htail.solar', 'vtail.solar', 'wing.solar'};

%Specify beam numbers of Aswing files. For reflected beams the nuber is
%given for the right instance first then the left.
optim.boom.beamNumber    = [6 7];
optim.wing.beamNumber    = 1;
optim.htail.beamNumber   = [2 3];
optim.vtail.beamNumber   = [4 5];
optim.pod.beamNumber     = 8;

% Initialize interface weights
optim.interfaceWeightScale = 0.15;

% Set boom numbers and names for cnBeta Aircraft
optim.boom.CnBetaAircraftBoomNumber = 2;
optim.aircraft.CnBetaAircraftBeamNames = {'wing', 'boom'};


% System Level Margins
optim.margin.weight_percent          = 0; % percent of MGTOW reserved for margin. Weight added to landingGear and placed at CG. 
optim.margin.power_percent           = 0; % percent of power draw reserved for margin.
optim.margin.drag_percent            = 0; % margin added to calculated drag

% Assign parameters related to the OML, Aerodynamics, Structures and loads,
% CONOPS, subsystems performances and others
optim = AerodynamicsAndOML(optim);
optim = Structures(optim);
optim = CONOPs(optim);
optim = Subsystems(optim);
optim = MiscParameters(optim);

end
