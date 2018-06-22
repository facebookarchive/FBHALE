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
  
function structData = ExecCoBlade(structData, idx, SIM, ANLS, OPT, ENV, BLADE, WEB, AF, MATS, Coord)
% This routine runs coblade and packages results into an aswing-friendly
% format. 

%% Build Laminate:
numStiffeners     = structData.numStiffeners;
nSegsTopBot       = ones(BLADE.NUM_SEC,1)*numStiffeners*2+1;
w_cap_inb         = structData.sparCapWidthInboard;
w_cap_oub         = structData.sparCapWidthOutboard;
UD_nPly           = structData.UD_nPly(:);
boxSkin_nPly      = structData.boxSkin_nPly(:);
sparCap_core_nPly = structData.sparCap_core_nPly(:);
web_core_nPly     = structData.web_core_nPly(:);
RemoveStationsStressCalc = structData.RemoveStationsStressCalc;

% concatenate all the design variables into a single column vector, for
% possible sub-optimization within the outer MDO loop. 

xo = [w_cap_inb; ...            % scalar: (1 by 1)
    w_cap_oub; ...   	        % scalar: (1 by 1)
    UD_nPly; ...   	            % vector: (2 by 1)
    sparCap_core_nPly; ...      % vector: (2 by 1)
    boxSkin_nPly; ...           % vector: (2 by 1)
    web_core_nPly];             % vector: (2 by 1)

[WEB SECNODES LamData] = defineLaminateData(structData.name, xo, structData.TopUDScale, structData.BotUDScale, structData.UDScaleStations, structData.numStiffeners,...
OPT, BLADE, WEB, MATS, nSegsTopBot);

%% Run Co-Blade:

[Panel StrProps AppLoads ResLoads Disp NormS ShearS Buckle MidPlane LaminaSS] ...
    = structAnalysis(SIM, ANLS, ENV, BLADE, WEB, MATS, LamData,...
      AF, SECNODES, Coord, structData.ResLoads);

%% Process/Collect Outputs:

% Calculate critical stress criteria's
[s11TFailureCriteria,  s11CFailureCriteria,  s22TFailureCriteria,  s22CFailureCriteria,  s12SFailureCriteria, maxBucklingStress] = ...
    failureCriteria(BLADE, WEB, LaminaSS, Buckle, structData.FOS, RemoveStationsStressCalc);

StrProps.s11TFailureCriteria = s11TFailureCriteria;
StrProps.s11CFailureCriteria = s11CFailureCriteria;
StrProps.s22TFailureCriteria = s22TFailureCriteria;
StrProps.s22CFailureCriteria = s22CFailureCriteria;
StrProps.s12SFailureCriteria = s12SFailureCriteria;
StrProps.maxBucklingStress   = maxBucklingStress;

% Arrange outputs in ASWING format
structData.spanLocation      = structData.spanLocation(:);
structData.zSec              = StrProps.zSec;
structData.chord_m           = structData.chord_m(:);
structData.twist_deg         = structData.twist_deg;
structData.Panel             = Panel;
structData.LamData           = LamData;
structData.MATS              = MATS;
structData.StrProps          = StrProps;

if isfield(structData,'twistAero_deg')
warning('twistAero_deg not used!');
end

%Calculate secondary mass structural weight:
massPerLength_kgm = structData.ribMassPerChord_kgm*structData.NribsPerSpan_m.*structData.chord_m{idx}...
    + structData.skinWeightPerArea_kgm2.*structData.chord_m{idx}; 

% Add fairing masses if needed for fuselage
massPerLength_kgm(1:length(structData.fuselageCanopySectionsR_m{idx})) = massPerLength_kgm(1:length(structData.fuselageCanopySectionsR_m{idx}))+...
    2*pi*structData.fuselageCanopySectionsR_m{idx}*structData.fairingWeightPerArea_kgm2;

%Scale secondary with uncertainty multipliers:
miscStructMass_den = massPerLength_kgm*(1+structData.uncertaintyAnalysis.secondaryStruct);

%Update CG with secondary mass structData:
miscStructMass_CG  = 0.5 - structData.pitchAxis;
structData.Ccg_m{idx} = (StrProps.mass_den.*StrProps.x_cm +...
    miscStructMass_den(:).*miscStructMass_CG.*structData.chord_m{idx}(:))./...
    (StrProps.mass_den(:) + miscStructMass_den(:));
StrProps.mass_den = StrProps.mass_den(:) + miscStructMass_den(:);

structData.EIcc_Nm2{idx}                    = StrProps.EIx_tc;
structData.EInn_Nm2{idx}                    = StrProps.EIy_tc;
structData.GJ_Nm2{idx}                      = StrProps.tor_stff;
structData.EA_N{idx}                        = StrProps.axial_stff;
structData.EIcn_Nm2{idx}                    = -StrProps.EIxy_tc;
structData.EIsn_Nm2{idx}                    = zeros(structData.numStations,1);
structData.EIcs_Nm2{idx}                    = zeros(structData.numStations,1);
structData.Nea_m{idx}                       = StrProps.y_sc;
structData.Cea_m{idx}                       = StrProps.x_sc;
structData.Nta_m{idx}                       = StrProps.y_tc;
structData.Cta_m{idx}                       = StrProps.x_tc;
structData.Ncg_m{idx}                       = StrProps.y_cm;
structData.mg_N_m{idx}                      = StrProps.mass_den*9.81;
structData.mg_N_m2{idx}                     = StrProps.mass_den*9.81./structData.chord_m{idx}(:);
structData.mgcc_Nm{idx}                     = StrProps.mIx_tc*9.81;
structData.mgnn_Nm{idx}                     = (StrProps.mIy_tc + (miscStructMass_den.*miscStructMass_CG.*structData.chord_m{idx}(:)).^2)*9.81;
structData.Xax{idx}                         = structData.pitchAxis*ones(structData.numStations,1);
structData.mass_kg(idx)                     = trapz(StrProps.zSec,StrProps.mass_den)*structData.numInstances;
structData.secondaryStructuralMass_kg(idx)  = trapz(StrProps.zSec, miscStructMass_den)*structData.numInstances;
structData.primaryStructuralMass_kg(idx)    = structData.mass_kg(idx) - structData.secondaryStructuralMass_kg(idx);

if structData.recordMaxValues
    structData.MaxTipDeflection           = StrProps.MaxTipDeflection;
    structData.MaxTwist                   = StrProps.MaxTwist;
    structData.maxBucklingStress          = StrProps.maxBucklingStress;
    structData.s11TFailureCriteria        = StrProps.s11TFailureCriteria;
    structData.s11CFailureCriteria        = StrProps.s11CFailureCriteria;
    structData.s22TFailureCriteria        = StrProps.s22TFailureCriteria;
    structData.s22CFailureCriteria        = StrProps.s22CFailureCriteria;
    structData.s12SFailureCriteria        = StrProps.s12SFailureCriteria;
end

% Spanwise CG:
structData.yCG_m(idx) = ...
    trapz(StrProps.zSec,structData.sRefY_m{idx}.*(StrProps.mass_den(:)))/trapz(StrProps.zSec,(StrProps.mass_den(:)));

% Chordwise CG:
structData.xCG_m(idx) = ...
    StructureCGCalc (structData.t{idx}, structData.sRefX_m{idx}, structData.sRefY_m{idx}, structData.sRefZ_m{idx}, structData.twist_deg{idx},...
    structData.Ncg_m{idx}, structData.Ccg_m{idx}, structData.mg_N_m{idx});

% Calculate Approx. Natural Frequencies:
m_mean_mg_N_m               = mean(structData.mg_N_m{idx}/9.81);
EIcc_mean_Nm2               = mean(structData.EIcc_Nm2{idx});
EInn_mean_Nm2               = mean(structData.EInn_Nm2{idx});
L_m                         = structData.length_m(idx);

structData.omega_cc_rad_s       = 1.875^2*sqrt(EIcc_mean_Nm2/(m_mean_mg_N_m*L_m^4));
structData.omega_nn_rad_s       = 1.875^2*sqrt(EInn_mean_Nm2/(m_mean_mg_N_m*L_m^4));
structData.minNaturalFreq_rad_s = min(structData.omega_cc_rad_s,structData.omega_nn_rad_s);


end
function xCG_m = StructureCGCalc (tidx, x_m, y_m, z_m, twist_deg, Ncg_m, Ccg_m, mg_N_m)
% This routine computes cg of a structural member given distributed mass
% properties. 

% Geometry
t   = linspace(0, max(tidx), 200);
x_m = interp1(tidx, x_m, t);
y_m = interp1(tidx, y_m, t);
z_m = interp1(tidx, z_m, t);
theta_deg = interp1(tidx, twist_deg, t);

% Local Mass distribution
Ncg_m = interp1(tidx, Ncg_m, t);
Ccg_m = interp1(tidx, Ccg_m, t);
mg_N_m  = interp1(tidx, mg_N_m, t);

% Euler Angles
dxdt = diff(x_m)./diff(t);
dydt = diff(y_m)./diff(t);
dzdt = diff(z_m)./diff(t);

rad2deg     = 180/pi;
phi_deg 	= atan(dzdt ./ dydt)*rad2deg;
psi_deg 	= atan(-dxdt./(dydt.*cosd(phi_deg) + dzdt.*sind(phi_deg))) * rad2deg;
tMid        = t(1:end-1) + diff(t)/2;
phi_deg     = interp1(tMid, phi_deg, t, 'linear', 'extrap');
psi_deg     = interp1(tMid, psi_deg, t, 'linear', 'extrap');

 % Local beam c-s-n system based on undeformed shape (sweep, twist)
for s = 1:length(t)
        
    Rtwist = [  cosd(theta_deg(s)) 	0   -sind(theta_deg(s));
                0                           1   0;
                sind(theta_deg(s))	0   cosd(theta_deg(s))];
    
    Ryaw = [cosd(psi_deg(s)) 	sind(psi_deg(s))	0;
            -sind(psi_deg(s)) 	cosd(psi_deg(s))	0;
            0                           0                           1];
    
    Rdihedral = [1	0                               0;
                0	cosd(phi_deg(s))     sind(phi_deg(s));
                0 	-sind(phi_deg(s))	cosd(phi_deg(s))];
    
    % Tensor matrix
    T = Rtwist * Ryaw * Rdihedral;
    
    % Local coordinate system
    ec(:, s) = T(1, :)';
    es(:, s) = T(2, :)';
    en(:, s) = T(3, :)';
    
    % Local cg location
    CG_m(:, s) = [x_m(s); y_m(s); z_m(s)] + Ncg_m(s)*en(:, s) + Ccg_m(s)*ec(:, s);
    
    % x * Mass of the section
    xMG(s) = CG_m(1, s) * mg_N_m(s);
end

% Expected CG
xCG_m = trapz(t, xMG)/trapz(t, mg_N_m);
end
