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
  
function optim = SizeWinglets(optim)
% Winglets are used for flying wings configurations to provide a minimum
% CnB.
% Currently, the extent is fixed conservatively.

% Position & current vertical area
xV_m = max(optim.wing.aero.sectionPolars.xQC_m);

if ~isfield(optim.wing.winglet, 'zoc0')
    optim.wing.winglet.zoc0 = optim.wing.winglet.zoc;
end
Sv0_m2 = optim.wing.c_m(end)^2*optim.wing.winglet.zoc0;

% Size winglets for CnBeta to be above minimum value

% Prototype sizing:
% Turned off for now
%     % Compute CnBeta due to other factors: wing deflection, booms, pods
%     optim = CnBetaOthers(optim);
%         
%     % CnB due to wing deflection at various speeds
%     CnB = max(optim.wing.aero.CnBeta_VD(1), optim.wing.aero.CnBeta_VC(1));
% 
%     % The boom deflection can seriously dampen the directional stability
%     CYbvBA       = mean(optim.wing.aero.aswingPerfAirfoilADB.dCLda(end)); % assumes quasi 2D as wing-extension
%     
% 	% CnB due to current-sized winglets
%     CnBwinglets0 = -2 * CYbvBA * (xV_m - optim.xCG_m)/optim.wing.bref_m * Sv0_m2 / optim.wing.Sref_m2;
%     
%     % Size winglets
% 	Sv_m2 = -(optim.aircraft.aero.CnBetaMin - (CnB - CnBwinglets0))./(CYbvBA * (xV_m - optim.xCG_m)/optim.wing.bref_m ) * optim.wing.Sref_m2;
%     
%     % Record chords and span
%     optim.wing.winglet.zoc = Sv_m2/max(optim.wing.c_m(end))^2;
%     if optim.wing.winglet.zoc < 0
%         optim.winglet.zoc = 0;
%     end

% Leave as the initial value i.e don't update.

% Update aero data at the tip
optim = PopulateSectionPolars(optim, 'wing');
optim.wing.aero.aswingLoadsAirfoilADB.t = optim.wing.aero.sectionPolars.t;
optim.wing.aero.aswingPerfAirfoilADB.t  = optim.wing.aero.sectionPolars.t;
end


% This routine finds the worst case yaw due to the wing, boom, and
% their interference.
function optim = CnBetaOthers(optim)

% Boom/Pod destabilizing effect. Assume slender bendy theory: 
% CnBeta = -2V/(Sref*bref), 
%  with V the equivalent volume of the fuselage (same as volume if axisymmetric)
if max(contains(optim.aircraft.beamNames, 'boom'))
    CnBetaBoom = sum(optim.boom.N) * optim.boom.V_m3 / (optim.wing.bref_m * optim.wing.Sref_m2);
else
	CnBetaBoom  = 0;
end
if max(contains(optim.aircraft.beamNames, 'pod'))
    CnBetaPod  = sum(optim.pod.N) * optim.pod.V_m3 / (optim.wing.bref_m * optim.wing.Sref_m2);
else
    CnBetaPod  = 0;
end

% CnB due to wing deflection at various speeds
CnBetaDihedral_VD = optim.wing.aero.CnBeta_VD(1);
CnBetaDihedral_VC = optim.wing.aero.CnBeta_VC(1);

% Store
optim.aircraft.aero.CnBetaAdditional = max(CnBetaBoom + CnBetaPod + [CnBetaDihedral_VC CnBetaDihedral_VD]);

end
