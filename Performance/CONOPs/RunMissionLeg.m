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
  
% This function executes the mission. The subfunction forward time step 
% contains the type of mission logic.

function [mission, aircraft, freeStream] = RunMissionLeg(mission, aircraft, freeStream, environment)

% Initialize leg time, add to state list
mission.dt_s            = mission.defaultDt_s;
mission.leg.t_s         = 0;

% Add first state: time
aircraft.states.integration(1).var	= 'mission.leg.t_s';
aircraft.states.integration(1).der	= '1';

% Initialize leg and overall mission quantities logging
mission = Logging(mission, 'initialize', aircraft, freeStream);

% If no climb rate was specified, assume zero
if ~isfield(freeStream ,'gamma_rad')
    freeStream.gamma_rad = 0;
end
if ~isfield(freeStream ,'dVdt_ms2')
    freeStream.dVdt_ms2 = 0;
end

% Compute min power CL vs altitude if not already specified
if ~isfield(aircraft.aero.minPower, 'h_m')
    h_m = unique([0 linspace(mission.minh_m, mission.maxh_m, 10)]);
    [rho_kgm3, dynamicViscosity_Nsm2]  =  GetAtmosphereProperties(h_m);
    K = 1/2*rho_kgm3.*(dynamicViscosity_Nsm2./(rho_kgm3 * aircraft.aero.Cref_m)).^2 * aircraft.aero.Sref_m2;
    for i = 1:length(h_m)
        F = @(x) aircraft.massProperties.MGTOW_N -  K(i)*x.^2.*interp1(aircraft.aero.minPower.Re, aircraft.aero.minPower.CL, x);
        %If max reynolds number out of range saturate at max reynolds
        %number values
        if min(F(aircraft.aero.minPower.Re))>0
            Re_minPower(i) = max(aircraft.aero.minPower.Re);
        else     
            Re_minPower(i) = interp1(F(aircraft.aero.minPower.Re), aircraft.aero.minPower.Re, 0);
        end
    end
    aircraft.aero.minPower.h_m    = interp1(Re_minPower, h_m', aircraft.aero.minPower.Re, 'linear', 'extrap');
end

[aircraft, freeStream] = InitializeStates(mission, aircraft, freeStream);

% While the exit criterion is not satisfied continue
while ~mission.leg.legExit.criterion( eval(mission.leg.legExit.variable) )
    
    % Based on the current altitude stored in mission.leg.h_m and look-up
    % density and specificViscosity
    [freeStream.rho_kgm3, freeStream.dynamicViscosity_Nsm2] = GetAtmosphereProperties(aircraft.states.h_m);
    
    % Backup airplane and conditions before moving forward
    aircraft0   = aircraft;
    freeStream0 = freeStream;
    exitVar0    = eval(mission.leg.legExit.variable);
    
    % Try to move forward
    rejectTimeStep = false;
    [aircraft, freeStream, mission] = PredictorCorrector(aircraft, freeStream, mission, environment);
    
    % If criterion would be satisfied within tolerance then exit. 
    % If not then split the time step.
    if mission.leg.legExit.criterion( eval(mission.leg.legExit.variable) ) % criterion is satisfied
        exitVar = eval(mission.leg.legExit.variable);
        error   = abs(exitVar - mission.leg.legExit.value)/abs(mission.leg.legExit.value);
        
        if error > mission.leg.legExit.tolerance
            mission.dt_s = mission.dt_s * (mission.leg.legExit.value - exitVar0) / (exitVar - exitVar0);
            
            % Since the time step is rejected, bring back the old data
            rejectTimeStep = true;
            freeStream     = freeStream0;
            aircraft       = aircraft0;
        end
    end
    
    if mission.dt_s < 1e-2 % Bullet proofing for low time steps
        rejectTimeStep = false;
    end
    
    % States integration and logging
    if ~rejectTimeStep
         
        % Integrate
        [aircraft, freeStream, mission] = StateIntegration(aircraft, freeStream, mission);
       
        % Additional leg-logging
        mission = Logging(mission, 'appendTimeStep', aircraft, freeStream);
    end
    
end

% End of leg, promote end values to the mission structure
mission = Logging(mission, 'appendLeg', aircraft, freeStream);

% Ensure that stale data is not used subsequently
freeStream = rmfield(freeStream, 'gamma_rad');

end
