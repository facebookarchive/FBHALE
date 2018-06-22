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
  
function [aircraft, freeStream, mission] = FlightMechanicsExplicit(aircraft, freeStream, mission)
% This function trims the aircraft based on the specified degrees of
% freedom, aero and propulsion models. It leverages explicit expressions
% for the flight mechanics equations assuming 2DOF.
% The equations are specified in the wind axis under the form:
%       F_propulsion = F_aero + Weight
% The system is solved using variables coherently with the choice of model.
% For instance, in a 2DOF system, 
%       F_propulsion = [Thrust * cos(installationAngle); Thrust * sin(installationAngle)];
%       F_aero       = [Drag; Lift];
%       Weight       = [W_N * sin(climbAngle); W_N * cos(climbAngle)];

% Bullet proofing
if ~isfield(aircraft.aero, 'DOF')
    error('No DOF specified in the aero structure');
end

% Keep track of aircraft trim variables to avoid solving from scratch
persistent X
if isempty(X)
    X.aeroMode = '';
    X.SLF      = true;
    X.VTAS_ms   = 0;
end


% If trim is for SLF and the past trim was the same SLF then avoid trimming
% again.
trim = true;
h_m  = GetLoggedVariable(mission, 'leg', 'h_m');
if length(h_m) >= 2
    if freeStream.gamma_rad == 0 && freeStream.dVdt_ms2 == 0 && X.SLF
        % Check that the same aero mode was selected. If it was 'specifiedV',
        % check that it was the same speed. If it was 'minPower' then there is
        % no additional checking necessary.
        if strcmp(X.aeroMode, aircraft.aero.mode)
            if isfield(aircraft.states, 'hdotRequested_ms') % If we're keeping the same altitude, no reason to re-trim
                if aircraft.states.hdotRequested_ms ~= 0
                    trim = false;
                end
            end
            if strcmp(X.aeroMode, 'specifiedV')             % If the specified airspeed and altitude hasn't changed, no reason to re-trim
                if freeStream.VTAS_ms == X.VTAS_ms
                    if h_m(end) == h_m(end-1)
                        trim = false;
                    end
                end
            end
            if strcmp(mission.groundTrackPath, 'orbit')     % If we're orbitting & the ground figure isn't a circle, then re-trim
                if ~strcmp(mission.groundTrackFigure, 'circle')
                    trim = true;
                end
            end
            if isfield(freeStream, 'windHeading_rad')       % If wind heading was provided, then re-trim
                trim = true;
            end
        end 
    end
end
        
if trim || ~isfield(aircraft.propulsion.propulsor(1).propeller, 'thrust_N')
    % Finding freeStream speed based on climb rate and wind speed
    if strcmp(mission.leg.mode, 'minV')
        freeStream.VTAS_ms = freeStream.windSpeed_ms/cos(freeStream.gamma_rad);
    end
    
    switch aircraft.aero.DOF
        case 2
            [F, Faero, Fpropulsion, Fweight, Faccel, X] = FlightMechanics2DOF(aircraft, freeStream, mission, X);
    end
        
    % Solve explicitly
    if strcmp(X.tags{1}, 'freeStream.VTAS_ms')
        VTAS0_ms   = sqrt( 2 * aircraft.massProperties.MGTOW_N / (aircraft.aero.Sref_m2*aircraft.aero.CLcruise*freeStream.rho_kgm3));
        Aero_N     = Faero(VTAS0_ms);
        W_N        = Fweight();
        VTAS_ms    = sqrt(abs(W_N(2)) / (Aero_N(2)/VTAS0_ms^2) );
        Aero_N     = Faero(VTAS_ms);
        T_N        = Aero_N(1) + W_N(1);
        
        X.values   = [VTAS_ms; T_N];
        X.initial  = X.values;
        aircraft.aero.CL = abs(W_N(2))*2/(freeStream.rho_kgm3 * aircraft.aero.Sref_m2*VTAS_ms^2);
    else
        CL0        = aircraft.aero.CLcruise;
        W_N        = Fweight();
        Aero_N     = Faero(CL0);
        CL         = -W_N(2) / (Aero_N(2)/CL0);
        Aero_N     = Faero(CL);
        accel_N    = Faccel();
        T_N        = Aero_N(1) + W_N(1) - accel_N(1);
        X.values   = [CL; T_N];
        X.initial  = X.values;
    end
    
end

    % Assign
    for t = 1:length(X.tags)
        eval([X.tags{t} '= ' num2str(X.values(t)) ';']);
    end
    
    % Update climb angle if hdot was specified
    if isfield(aircraft.states, 'hdotRequested_ms')
        freeStream.gamma_rad = asin(aircraft.states.hdotRequested_ms/freeStream.VTAS_ms);
    end
    
    % Record mode for next time step
    X.aeroMode = aircraft.aero.mode;
    X.VTAS_ms     = freeStream.VTAS_ms;
    if freeStream.gamma_rad == 0 && freeStream.dVdt_ms2 == 0
        X.SLF = true;
    else
        X.SLF = false;
    end
%end


% Aircraft kinematics based on mission specification
[aircraft, freeStream] = FlightKinematics(mission, aircraft, freeStream);

end

