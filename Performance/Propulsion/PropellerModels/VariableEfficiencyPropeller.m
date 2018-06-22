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
  
function propulsion = VariableEfficiencyPropeller(propulsion, freeStream)
% This routine leverages the propeller performance maps as computed by the
% MDO sizing framework in 'SizeAndCharacterizePropeller'. The formulation
% allows for quick calculations of thrust, torque, and RPM based on 
% required thrust for the mission and climb rate and within maximum RPM
% torque limits. If constraints are violated, the maximum achievable thrust
% within the constraints is output.

% Selected propulsor
aP = propulsion.activePropulsor;

% Bullet proofing
if ~isfield(propulsion.propulsor(aP).propeller, 'thrust_N')
    error('No required thrust was specified for the constant efficiency propeller model.');
end
if ~isfield(propulsion.propulsor(aP).propeller, 'map')
    error('No map specified for the variable efficiency propeller model.');
end
if ~isfield(freeStream, 'VTAS_ms')
    error('No freestream speed was specified. Cannot look-up propeller performance.');
end

% Required thrust per propeller
N      = sum(propulsion.propulsor(aP).N);
Treq_N = propulsion.propulsor(aP).propeller.thrust_N / N;
   

if Treq_N < 2 % If thrust is quasi zero find advance ratio and blade pitch for minimum extra power
    
    J        = @(rpm) freeStream.VTAS_ms./(rpm*propulsion.propulsor(aP).propeller.d_m)*60;
    Pshaft_W = @(rpm) interp1f(propulsion.propulsor(aP).propeller.map.JCT0, propulsion.propulsor(aP).propeller.map.CPCT0, J(rpm)) * freeStream.rho_kgm3 .* (rpm/60).^3 * propulsion.propulsor(aP).propeller.d_m^5;

    RPMmin = min(freeStream.VTAS_ms./(propulsion.propulsor(aP).propeller.map.JCT0*propulsion.propulsor(aP).propeller.d_m)*60);
  	RPMmax = max(freeStream.VTAS_ms./(propulsion.propulsor(aP).propeller.map.JCT0*propulsion.propulsor(aP).propeller.d_m)*60);
        
    RPMs = linspace(RPMmin, RPMmax, 100);
    Ps_W = Pshaft_W(RPMs);
    [Ps_W, r] = min(Ps_W);
    RPM      = RPMs(r);
    
    % Propeller state
    propulsion.propulsor(aP).propeller.shaftPower_W = Ps_W * N;
    propulsion.propulsor(aP).propeller.theta_deg    = interp1f(propulsion.propulsor(aP).propeller.map.JCT0, propulsion.propulsor(aP).propeller.map.pitch_deg, J(RPM));
    propulsion.propulsor(aP).propeller.efficiency   = 0;
    propulsion.propulsor(aP).propeller.RPM          = RPM;
    propulsion.propulsor(aP).propeller.thrustActual_N = 0;
    propulsion.propulsor(aP).propeller.thrustResidual_N = - Treq_N*N;
else
    
    pitch_deg = propulsion.propulsor(aP).propeller.map.pitch_deg;
    CT  = propulsion.propulsor(aP).propeller.map.CT;
    CP  = propulsion.propulsor(aP).propeller.map.CP;
    CQ  = propulsion.propulsor(aP).propeller.map.CQ;
    J   = propulsion.propulsor(aP).propeller.map.J;
    eta = propulsion.propulsor(aP).propeller.map.eta;
    
    % CTv constraint = CT./J^2
    CTvReq  = Treq_N / (freeStream.VTAS_ms^2 * freeStream.rho_kgm3 * propulsion.propulsor(aP).propeller.d_m^2);
    Jctv = ScaleTimeBulletProof(propulsion.propulsor(aP).propeller.invertedMaps.JCTv', propulsion.propulsor(aP).propeller.invertedMaps.CTv, CTvReq);
    
    % CQv constraint = CQ./J^2
    CQMaxv = propulsion.propulsor(aP).propeller.Qmax_Nm/(freeStream.VTAS_ms^2 * freeStream.rho_kgm3 * propulsion.propulsor(aP).propeller.d_m^3);
    Jcqv = ScaleTimeBulletProof(propulsion.propulsor(aP).propeller.invertedMaps.JCQv', propulsion.propulsor(aP).propeller.invertedMaps.CQv, CQMaxv);
    
    % CPv constraint  CP./J^3
    CPMaxv = propulsion.propulsor(aP).propeller.Pmax_W/(freeStream.VTAS_ms^3 * freeStream.rho_kgm3 * propulsion.propulsor(aP).propeller.d_m^2);
    Jcpv = ScaleTimeBulletProof(propulsion.propulsor(aP).propeller.invertedMaps.JCPv', propulsion.propulsor(aP).propeller.invertedMaps.CPv, CPMaxv);
    
    % Max RPM constraint
    JmaxRPM = freeStream.VTAS_ms./(propulsion.propulsor(aP).propeller.RPMmax*propulsion.propulsor(aP).propeller.d_m)*60;
    
    % Find feasible space : area under the power and torque curves
    Jmax = max(max(Jcpv', Jcqv'), JmaxRPM*ones(size(Jcpv))');
    
    % Check if a part of the requested thrust lies under the
    % power-torque constraint curve
    feasibleJ = Jctv' >= Jmax;
    canDeliverThrust = max(feasibleJ);
    
    if canDeliverThrust % Maximize efficiency
        CPv = interp2(pitch_deg*pi/180, J, CP, propulsion.propulsor(aP).propeller.invertedMaps.pitch_deg(feasibleJ)'*pi/180, Jctv(feasibleJ)')./Jctv(feasibleJ)'.^3;
        [~, indexMinPower] = min(CPv);
        theta_deg = propulsion.propulsor(aP).propeller.invertedMaps.pitch_deg(feasibleJ);
        theta_deg = theta_deg(indexMinPower);
        lambda    = Jctv(feasibleJ)';
        lambda    = lambda(indexMinPower);
        
    else % Maximize thrust based on constraints
        CTv = interp2(pitch_deg*pi/180, J, CT, propulsion.propulsor(aP).propeller.invertedMaps.pitch_deg'*pi/180, Jmax)./Jmax.^2;
        [~, indexMaxThrust] = max(CTv);
        theta_deg = propulsion.propulsor(aP).propeller.invertedMaps.pitch_deg(indexMaxThrust);
        lambda    = Jmax(indexMaxThrust);
        
    end
    RPM = freeStream.VTAS_ms./(lambda*propulsion.propulsor(aP).propeller.d_m)*60;
    T_N = @(rpm, theta) interp2f(pitch_deg*pi/180, J, CT, theta*pi/180, freeStream.VTAS_ms./(rpm*propulsion.propulsor(aP).propeller.d_m)*60)' * freeStream.rho_kgm3 .* (rpm/60).^2*(propulsion.propulsor(aP).propeller.d_m)^4;
    Q_Nm = @(rpm, theta) interp2f(pitch_deg*pi/180, J, CQ, theta*pi/180, freeStream.VTAS_ms./(rpm*propulsion.propulsor(aP).propeller.d_m)*60)' * freeStream.rho_kgm3 .* (rpm/60).^2*propulsion.propulsor(aP).propeller.d_m^5;
    P_W = @(rpm, theta) Q_Nm(rpm, theta) .* rpm*2*pi/60;
    
    % Propeller state
    propulsion.propulsor(aP).propeller.theta_deg      = theta_deg;
    propulsion.propulsor(aP).propeller.thrustActual_N = T_N(RPM, theta_deg)*N;
    propulsion.propulsor(aP).propeller.shaftPower_W = P_W(RPM, theta_deg)*N;
    propulsion.propulsor(aP).propeller.efficiency   = interp2f(pitch_deg*pi/180, J, eta, theta_deg*pi/180, freeStream.VTAS_ms./(RPM*propulsion.propulsor(aP).propeller.d_m)*60);
    propulsion.propulsor(aP).propeller.torque_Nm    = Q_Nm(RPM, theta_deg);
    propulsion.propulsor(aP).propeller.RPM          = RPM;
    if canDeliverThrust
        propulsion.propulsor(aP).propeller.thrustResidual_N = 0;
    else
        propulsion.propulsor(aP).propeller.thrustResidual_N = propulsion.propulsor(aP).propeller.thrustActual_N - Treq_N*N;
    end
end
end