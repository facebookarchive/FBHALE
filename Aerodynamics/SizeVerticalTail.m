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
  
function optim = SizeVerticalTail(optim)
% This function finds the vertical tail area Sv by driving the worst case
% sideslip + aileron deflection to zero yaw. It also checks for a minimum
% CnB requirement (set by the user).

% Initial guess
optim.vtail.x_m = optim.htail.x_m + optim.htail.c_m(1) + optim.vtail.c_m(1)/4;

% Through the sizing, the area depends on the position and vice-versa.
% Set-up re-usable functions to solve the coupled system
b_m  = @(xv, Sv) sqrt(optim.vtail.AR * Sv);
cR_m = @(xv, Sv) Sv ./ (b_m(xv, Sv) * (2 - optim.vtail.taperRatio));
x_m  = @(xv, Sv) optim.htail.x_m + optim.htail.c_m(1) + cR_m(xv, Sv)/4;
    
% Derivatives for Jacobian
dxdxv = @(xv, Sv) 0;
dxdSv = @(xv, Sv) 1/8./sqrt(Sv*optim.vtail.AR)/(2 - optim.vtail.taperRatio);

% 1. Size per the worst case yaw

    % Find worst case sideslip: approach speed
    V_ms                        = sqrt(optim.MGTOW_kg * optim.constants.g_ms2 * 2/(optim.aircraft.Sref_m2 * optim.constants.rhoSL_kgm3 * optim.wing.aero.CLmax));
    optim.vtail.aero.V_ms       = V_ms;
    beta_rad                    = atan(optim.vtail.maxVy_ms/V_ms);
    optim.vtail.aero.beta_rad   = beta_rad;

    % Worst case yaw due to sideslip and boom
    optim = WorstCaseYaw(optim);

    % Get CLmax: 90 % of the maximum airfoil CL
    optim.vtail.aero.CLMax = .9 * mean(optim.vtail.aero.aswingLoadsAirfoilADB.CLmax);

    % Update Vtail size. Make function to solve 2-2 system
    S_m2 = @(xv, Sv) optim.aircraft.Sref_m2 * optim.aircraft.bref_m / (xv - optim.xCG_m) * optim.vtail.aero.CnMax / optim.vtail.aero.CLMax / sum(optim.vtail.N);
    dSdxv = @(xv, Sv) -optim.aircraft.Sref_m2 * optim.aircraft.bref_m * optim.vtail.aero.CnMax / optim.vtail.aero.CLMax / sum(optim.vtail.N)./(xv - optim.xCG_m).^2;
    dSdSv = @(xv, Sv) 0;
    
    %  Solve the 2-2 system
    F = @(X) VtailPositionSize(X, S_m2, x_m, dSdxv, dSdSv, dxdSv, dxdxv);
    J = @(X) VtailPositionSize(X, S_m2, x_m, dSdxv, dSdSv, dxdSv, dxdxv,'J');
    X0 = [optim.vtail.x_m; optim.vtail.Sref_m2];
    X = Newton(F, X0, 'J', J, 'relax', 1);
    optim.vtail.x_m  = X(1);
    optim.vtail.Sref_m2 = X(2);
    
% 2. Check CnBeta is above minimum value

    % The boom deflection can seriously dampen the directional stability
    k = mean(optim.vtail.aero.aswingLoadsAirfoilADB.dCLda)/(2*pi); % ratio of airfoil clalpha / 2pi - conservative
    optim.vtail.CLalpha = 2 * pi * optim.vtail.AR / (2 + sqrt( optim.vtail.AR^2 / k ^ 2 + 4 ));
    CYbvtailBA          = optim.vtail.CLalpha; % neglect sidewash
    CnBetavtail         = @(xv, Sv) CYbvtailBA * (x_m(xv, Sv) - optim.xCG_m)/optim.aircraft.bref_m * Sv/optim.aircraft.Sref_m2;
    CnBeta              = @(xv, Sv) sum(optim.boom.N) * CnBetavtail(xv, Sv) + optim.vtail.aero.CnBetaAdditional;
    
    isSizedByCnBeta = false;
    if CnBeta(optim.vtail.x_m, optim.vtail.Sref_m2) < optim.vtail.CnBetaMin
        S_m2 = @(xv, Sv) optim.aircraft.bref_m * optim.aircraft.Sref_m2 / CYbvtailBA / sum(optim.vtail.N) * (optim.vtail.CnBetaMin - optim.vtail.aero.CnBetaAdditional) ...
                ./(xv - optim.xCG_m);
        dSdSv = @(xv, Sv) 0;
        dSdxv = @(xv, Sv) -optim.aircraft.bref_m * optim.aircraft.Sref_m2 / CYbvtailBA / sum(optim.vtail.N) * (optim.vtail.CnBetaMin - optim.vtail.aero.CnBetaAdditional) ...
                ./(xv - optim.xCG_m).^2;
        
      	F = @(X) VtailPositionSize(X, S_m2, x_m, dSdxv, dSdSv, dxdSv, dxdxv);
       	J = @(X) VtailPositionSize(X, S_m2, x_m, dSdxv, dSdSv, dxdSv, dxdxv, 'J');
    
        X0 = [optim.vtail.x_m; optim.vtail.Sref_m2];
        X = Newton(F, X0, 'J', J);
        optim.vtail.x_m  = X(1); 
        optim.vtail.Sref_m2 = X(2);
        isSizedByCnBeta = true;
    end
    
% Record chords and span
optim.vtail.bref_m  = b_m(X(1), X(2));
optim.vtail.c_m     = cR_m(X(1), X(2)) * [1 optim.vtail.taperRatio];
    
% Store CnBetaVtail for output matching gammaB
optim.vtail.CnBetavtail = CnBetavtail;

end

% X(1): xV_m, X(2): Sv_m2. 
% Y(1): dxV_m
% Y(2): dSv_m2
function Y = VtailPositionSize(X, S_m2, x_m, dSdxv, dSdSv, dxdSv, dxdxv, varargin)

% First error is on the position, second is on the size
Y(1) = X(1) - x_m(X(1), X(2));
Y(2) = X(2) - S_m2(X(1), X(2));

% Jacobian
J(1, 1) = 1 - dxdxv(X(1), X(2));
J(2, 2) = 1 - dSdSv(X(1), X(2));
J(2, 1) = -dSdxv(X(1), X(2));
J(1, 2) = -dxdSv(X(1), X(2));

if nargin > 7
    Y = J;
end

end

% This routine finds the worst case yaw due to the wing, boom, and
% their interference.
function optim = WorstCaseYaw(optim)

% Boom/Fuselage destabilizing effect. Assume slender bendy theory: 
% CnBeta = -2V/(Sref*bref), 
%  with V the equivalent volume of the fuselage (same as volume if axisymmetric)
CnBetaBoom = sum(optim.boom.N .* optim.boom.V_m3) / (optim.wing.bref_m * optim.wing.Sref_m2);

% Boom/Fuselage yaw. Assume flat plate
CDfuse = 0.9;
CYBoom = sum(optim.boom.N) * CDfuse * optim.boom.Scross_m2 / optim.aircraft.Sref_m2 * cos(optim.vtail.aero.beta_rad)^2;
CnBoom = CnBetaBoom * optim.vtail.aero.beta_rad;

% Yaw due to sideslip due to induced drag at envelope bounds, provided at
% the deflected CG
CnBetaDihedral_VD = abs(optim.wing.aero.CnBeta_VD(1));
CnBetaDihedral_VC = abs(optim.wing.aero.CnBeta_VC(1));
CnDihedral        = CnBetaDihedral_VD * optim.vtail.aero.beta_rad;

% Yaw due to sideslip and aileron deflection
% At landing with max Beta, max CS. Beta is negative at max aileron
% deflection
CnBetaLanding_VS    = optim.wing.aero.CnBetaDeltaMax_VS(1); 
CnLandingMaxBeta_VS = optim.wing.aero.CnDeltaMax_VS(1) + CnBetaLanding_VS * (-optim.vtail.aero.beta_rad);
CnBetaLanding_VD    = optim.wing.aero.CnBetaDeltaMax_VD(1);
CnLandingMaxBeta_VD = optim.wing.aero.CnDeltaMax_VD(1) + CnBetaLanding_VD * (-optim.vtail.aero.beta_rad);
CnMaxDelta_VS       = optim.wing.aero.CnDeltaMax_VS(1);
CnMaxDelta_VD       = optim.wing.aero.CnDeltaMax_VD(1);

% Engine out case initial guess
if ~isfield(optim.propulsion, 'CnMotorOut')
    
    % Two propeller configurations see a large increase in tail size
    % due to motor out case. Prepare for upcoming disturbance.
    if sum(optim.propulsion.N) == 2
        % Assume L/D of 30. Then double hit for drag on the other side
        CL    = 1;
        V_ms  = sqrt(2 * optim.MGTOW_kg * optim.constants.g_ms2 / (optim.constants.rhoSL_kgm3 * optim.wing.Sref_m2 * CL) );
        D_N   = optim.MGTOW_kg * optim.constants.g_ms2 / 30;
        hdot_ms = optim.mission.minh_m / (60 * optim.mission.tclimb_min) / 2; % Half climb rate required
        T_N     = 3/2 * D_N + optim.MGTOW_kg * optim.constants.g_ms2 * hdot_ms / V_ms; % Margin for drag of motor out + full climb rate
        YawMotorOut_Nm      = T_N * abs(optim.propulsion.y_m( optim.propulsion.y_m~=0 ));
        CnMotorOut          = YawMotorOut_Nm /optim.aircraft.bref_m / (1/2 * optim.constants.rhoSL_kgm3 * optim.aircraft.Sref_m2 * V_ms^2) * 1/2;
    elseif optim.propulsion.N > 2
        CnMotorOut          = 0;
    else
        CnMotorOut          = 0;
    end
    
else
    CnMotorOut          = optim.propulsion.CnMotorOut;
end
       
% Worst case between trim, full CS, full beta
CnWorstCase = max( [abs(CnLandingMaxBeta_VS), abs(CnLandingMaxBeta_VD), ...     % Landing with max sideslip and max CS deflection
                    abs(CnDihedral), ...                                    	% Max sideslip at cruise
                    abs(CnMaxDelta_VS), abs(CnMaxDelta_VD), ...                 % Max CS deflection at cruise
                    abs(CnMotorOut)] );
                
if CnWorstCase < 0
    CnWorstCase = 0;
end

% Store
optim.vtail.aero.CnMax = abs(CnBoom) + CnWorstCase;
optim.vtail.aero.CYMax = CYBoom;
optim.vtail.aero.CnLanding = max([abs(CnLandingMaxBeta_VS) abs(CnLandingMaxBeta_VD)]);
optim.vtail.aero.CnCruise  = max([abs(CnMaxDelta_VS) abs(CnMaxDelta_VD) abs(CnDihedral)]);
optim.vtail.aero.CnBetaAdditional = - (CnBetaBoom + max([abs(CnBetaDihedral_VC) abs(CnBetaDihedral_VD) abs(CnBetaLanding_VS) abs(CnBetaLanding_VD)]));

end
