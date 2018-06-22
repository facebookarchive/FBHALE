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
  
% This routine creates the function that will be used for trimming the
% aircraft. Forces are expressed in the wind axes.
function [F, Faero, Fpropulsion, Fweight, Faccel, X] = FlightMechanics2DOF(aircraft, freeStream, mission, X)

%% Build external aero function
if strcmp(aircraft.aero.description, 'polar')
    
    % If V is specified solve for CL
    if strcmp(aircraft.aero.mode, 'specifiedV')
        Faero = @(X) getfield(Polar(setfield(aircraft.aero, 'CL', X(1)), freeStream, aircraft, mission), 'forces_N');
        
        % Update tags only if non-existent or necessary
        try
            if ~strcmp(X.tags{1}, 'aircraft.aero.CL')
                X.tags{1} = 'aircraft.aero.CL';
                X.initial(1)  = .8;
            end
        catch
            X.tags{1}     = 'aircraft.aero.CL';
            X.initial(1)  = .8;
        end
        
        % Record angle of attack 
        aircraft.aero = Polar((setfield(aircraft.aero, 'CL', X.initial(1))), freeStream, aircraft, mission);
      
    % If min Power is specified solve for V as CL is known
    else
        Faero = @(X) getfield(Polar(aircraft.aero, setfield(freeStream, 'VTAS_ms', X(1)),aircraft,mission), 'forces_N');
       
        % Update tags only if non-existent or necessary
        if isfield(X, 'tags')
            if ~strcmp(X.tags{1}, 'freeStream.VTAS_ms')
                X.tags{1} = 'freeStream.VTAS_ms';
                X.initial(1)  = sqrt(2*aircraft.massProperties.MGTOW_N/ (freeStream.rho_kgm3 * aircraft.aero.Sref_m2 ));
            end
        else
            X.tags{1} = 'freeStream.VTAS_ms';
            X.initial(1)  = sqrt(2*aircraft.massProperties.MGTOW_N/ (freeStream.rho_kgm3 * aircraft.aero.Sref_m2 ));
        end
        
        % Record angle of attack
        aircraft.aero = Polar(aircraft.aero, setfield(freeStream, 'VTAS_ms', X.initial(1)), aircraft,mission);
    end
end

if strcmp(aircraft.aero.description, 'LoD')
    Faero = @(X) getfield(LoD(setfield(aircraft.aero, 'lift_N', X(1))), 'forces');
    
    if ~isfield(X, 'tags')
        X.tags{1}     = 'aero.lift_N';
        X.initial(1)  = aircraft.massProperties.MGTOW_N*cos(freeStream.gamma_rad);
    end
end

%% Build propulsion function
if length(aircraft.propulsion.propulsor) == 1 % No need for any sort of mix i.e assuming equal RPM for all propellers
    aircraft.propulsion.activePropulsor = 1;
    aP = 1;
end
if strcmp(aircraft.propulsion.propulsor(aP).propeller.description, 'constantEfficiency')
    Fpropulsion = @(X) X(2) * [cos(aircraft.aero.alpha_rad); sin(aircraft.aero.alpha_rad)];
    
    if length(X.tags) == 1
        X.tags{2} = ['aircraft.propulsion.propulsor(' num2str(aP) ').propeller.thrust_N'];
        X.initial(2) = aircraft.massProperties.MGTOW_N/35/aircraft.propulsion.propulsor(aP).N; % Assumin L/D of 35 as a guess
    end
end
if strcmp(aircraft.propulsion.propulsor(aP).propeller.description, 'variableEfficiency')
    Fpropulsion = @(X) X(2) * [cos(aircraft.aero.alpha_rad); sin(aircraft.aero.alpha_rad)];
    
    if length(X.tags) == 1
        X.tags{2} = ['aircraft.propulsion.propulsor(' num2str(aP) ').propeller.thrust_N'];
        X.initial(2) = aircraft.massProperties.MGTOW_N/35/aircraft.propulsion.propulsor(aP).N; % Assumin L/D of 35 as a guess
    end
end

if strcmp(aircraft.propulsion.propulsor(aP).propeller.description, 'propellerMap')
    Fpropulsion = @(X) GetThrust(aircraft.propulsion, freeStream, X(1), X(2)) * ...
                            [cos(aircraft.aero.alpha_rad); -sin(aircraft.aero.alpha_rad)];
    
    if length(X.tags) == 1
        X.tags{2} = ['aircraft.propulsion.propulsor(' num2str(aP) ').propeller.rpm'];
        X.initial(2) = 600;
    end
end

%% Build weight function

if ~isfield(mission, 'pointing')
    Weff_N = aircraft.massProperties.MGTOW_N;
elseif strcmp(mission.pointing, 'orbit')
    % Effective weight due to the centripetal force
    Weff_N = aircraft.massProperties.MGTOW_N / cos(atan(mission.leg.groundSpeed_ms.^2./(mission.trajectory.radius_m*mission.g_ms2)));
else
    Weff_N = aircraft.massProperties.MGTOW_N;
end

% Climb angle
if isfield(freeStream, 'hdotRequested_ms')
    if ~isfield(freeStream, 'VTAS_ms')
        gamma_rad = 0;
    else
        gamma_rad = asin(aircraft.states.hdotRequested_ms/freeStream.VTAS_ms);
    end
else
    gamma_rad = freeStream.gamma_rad;
end
        
% Combined weight function
Fweight = @(X) Weff_N * [sin(gamma_rad); -cos(gamma_rad)];
        
%% Add acceleration if provided by mission integration

if isfield(freeStream, 'dVdt_ms2')
    Faccel = @(X) -aircraft.massProperties.MGTOW_kg * freeStream.dVdt_ms2 * [1; 0];
else
    Faccel = @(X) [0; 0];
end

%% Consolidating for zero finding
F = @(X) sum( ( Faccel(X) + Fweight(X) + Faero(X) - Fpropulsion(X) ).^2 ) ;

end

function thrust_N =  GetThrust(propulsion, freeStream, VTAS_ms, rpm)

% Active propulsor
aP = propulsion.activePropulsor;

% Set free-stream speed and rpm
freeStream.VTAS_ms = VTAS_ms;
propulsion.propulsor(aP).propeller.rpm = rpm;
propulsion = Propeller(propulsion, freeStream);

% Record thrust
thrust_N = propulsion.propulsor(aP).propeller.thrust_N * propulsion.propulsor(aP).N;

end
