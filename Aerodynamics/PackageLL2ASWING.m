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
  
function optim = PackageLL2ASWING(optim, wingPolar)
% This routine packages the output of the non-linear lifting line tool to 
% emulate an ASWING output and allow corresponding drag calculations with
% 'LiftAndDragCorrection.m'.
% Positions and angles are given at beam nodes. Derived quantities
% (cl, c, flift) given in between (see ASWING manual). Nodes 
% positions are assimilated as bound vortex locations.

  	optim.wing.aero.ASWINGpolar.AoA_deg = wingPolar.AoA_deg;
    optim.wing.aero.ASWINGpolar.controlSurface.deflection_deg = wingPolar.AoA_deg;    
    
    % t-index in lifting line is given along the quarter chord vs against
    % feathering axis in ASWING. Convert.
    tcp = interp1f(optim.wing.aero.sectionPolars.tQC, optim.wing.aero.sectionPolars.t, wingPolar.t{end});
    
    % Positions
    t    = tcp;
    tMid = t(1:end-1) + diff(t)/2;
    optim.wing.aero.ASWINGpolar.t        = {t};
    optim.wing.aero.ASWINGpolar.deltax_m = {zeros(1, length(t))};
    optim.wing.aero.ASWINGpolar.x_m      = {interp1f(optim.wing.aero.sectionPolars.t, optim.wing.aero.sectionPolars.x_m, t)};
    optim.wing.aero.ASWINGpolar.y_m      = {interp1f(optim.wing.aero.sectionPolars.t, optim.wing.aero.sectionPolars.y_m, t)};
    optim.wing.aero.ASWINGpolar.deltaz_m = {zeros(1, length(t))};
    optim.wing.aero.ASWINGpolar.z_m      = {interp1f(optim.wing.aero.sectionPolars.t, optim.wing.aero.sectionPolars.z_m, t)};
    
	% Euler Angles
    dxdt = diff(optim.wing.aero.ASWINGpolar.x_m{1})./diff(t);
	dydt = diff(optim.wing.aero.ASWINGpolar.y_m{1})./diff(t);
    dzdt = diff(optim.wing.aero.ASWINGpolar.z_m{1})./diff(t);
    
    rad2deg = 180/pi;
    optim.wing.aero.ASWINGpolar.phi_deg 	= atan(dzdt ./ dydt)*rad2deg;
    optim.wing.aero.ASWINGpolar.psi_deg 	= atan(-dxdt./(dydt.*cosd(optim.wing.aero.ASWINGpolar.phi_deg) + ...
                                                           dzdt.*sind(optim.wing.aero.ASWINGpolar.phi_deg))) * rad2deg;
    optim.wing.aero.ASWINGpolar.phi_deg     = {interp1(tMid, optim.wing.aero.ASWINGpolar.phi_deg, t, 'linear', 'extrap')};
  	optim.wing.aero.ASWINGpolar.psi_deg     = {interp1(tMid, optim.wing.aero.ASWINGpolar.psi_deg, t, 'linear', 'extrap')};
    optim.wing.aero.ASWINGpolar.theta_deg   = {0*interp1f(optim.wing.aero.sectionPolars.t, optim.wing.aero.sectionPolars.twists_deg, t)};
    
    % Derived quantities: given at mid points. Last value is zero
    optim.wing.aero.ASWINGpolar.cl  = {[interp1(wingPolar.tcp{end}, wingPolar.cl{end},  tMid, 'linear', 'extrap') 0]};
    optim.wing.aero.ASWINGpolar.c_m = {[interp1(optim.wing.aero.sectionPolars.t, optim.wing.aero.sectionPolars.c_m, tMid, 'linear', 'extrap') 0]};
    
    % Lift element
    ds_m = sqrt(diff(optim.wing.aero.ASWINGpolar.x_m{1}).^2 + diff(optim.wing.aero.ASWINGpolar.y_m{1}).^2 ...
                                                                  + diff(optim.wing.aero.ASWINGpolar.z_m{1}).^2);
    V_ms  = sqrt(2*optim.MGTOW_kg*optim.constants.g_ms2/(optim.constants.rhoSL_kgm3*optim.wing.Sref_m2*wingPolar.CL));
    q_Pa  = 1/2*optim.constants.rhoSL_kgm3*V_ms^2;
    optim.wing.aero.ASWINGpolar.f_lift_N_m = {[wingPolar.f_lift_m2{end}.*q_Pa./ds_m*2 0]};
    
    % Add aircraft quantities
    optim.aircraft.aero.ASWINGpolar.AoA_deg = wingPolar.AoA_deg;
    optim.aircraft.aero.ASWINGpolar.CL      = wingPolar.CL;
end