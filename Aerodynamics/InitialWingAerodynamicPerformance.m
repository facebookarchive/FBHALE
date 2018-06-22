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
  
function optim = InitialWingAerodynamicPerformance(optim)
% This routine uses a non-linear lifting line to capture the performance of
% the main aerodynamic surface: the wing. The main outputs are 1) a first
% guess at the wing root twist for cruise conditions, and 2) CLmax, CLmin
% for V-n diagram generation

	% Cruise Re
    % SL Re
    VCruiseEAS_ms = sqrt(optim.MGTOW_kg*optim.constants.g_ms2 /(1/2*optim.constants.rhoSL_kgm3*optim.wing.Sref_m2*optim.wing.aero.CLsizing));
    Re = VCruiseEAS_ms * optim.wing.cref_m / (optim.constants.muSL_Nsm2/optim.constants.rhoSL_kgm3);
   
    % Assemble the CL polynomial curves across the span
    optim = PopulateSectionPolars(optim, 'wing', {'cl', 'cd', 'cm'});
    [Xq, Yq, Zq] = meshgrid(optim.wing.aero.sectionPolars.delta_deg, optim.wing.aero.sectionPolars.Re, optim.wing.aero.sectionPolars.AoA_deg);
    indexZeroDef = find(optim.wing.aero.sectionPolars.delta_deg == 0);
    
    for s = 1:length(optim.wing.aero.sectionPolars.y_m)
        
        % Sectional data at aircraft Re
        Relocal = Re*optim.wing.aero.sectionPolars.c_m(s)/optim.wing.cref_m;
        if Relocal >= max(optim.wing.aero.sectionPolars.Re)
            Relocal = max(optim.wing.aero.sectionPolars.Re);
        end
        
        % Sectional data at aircraft Re
        cl = squeeze(optim.wing.aero.sectionPolars.cl(s, indexZeroDef, :, :));
        cl = permute(cl, [2 1 3]);
        cl = squeeze(interp2(optim.wing.aero.sectionPolars.Re, optim.wing.aero.sectionPolars.AoA_deg, cl, Relocal, optim.wing.aero.sectionPolars.AoA_deg));
        
        % To improve convergence, we remove potential outliers in the input
        % sectional polars due to ill-converged cases at high AoAs
        cla = abs(diff(cl)./diff(optim.wing.aero.airfoilADB.dim5.vector'));
        cla = ([cla(1); cla] + [cla; cla(end)])/2;
        flag = cla > 1.5 * 2*pi * pi/180; % 50% greater than theoretical inviscid value
        alfa_deg = optim.wing.aero.airfoilADB.dim5.vector(~flag);
        
        % Assign
     	optim.wing.aero.sectionPolars.cls(s, :) = interp1(alfa_deg, cl(~flag), optim.wing.aero.airfoilADB.dim5.vector, 'linear', 'extrap');
    end
    optim.wing.aero.sectionPolars.alphas_deg  = optim.wing.aero.airfoilADB.dim5.vector;
        
    % Sweep AoAs searching for CLmax. Use a copy of optim to avoid
    % polluting the actual one with temporary values
    optim0   = optim;
    AoAs_deg = -15:2:20;
    CL       = zeros(1, length(AoAs_deg));
    CDi      = zeros(1, length(AoAs_deg));
    isConverged = true;
    for a = 1:length(AoAs_deg)
      	if ~exist('wingPolar') || ~isConverged
            wingPolar = [];
             wingPolar2 = [];
        end
        [wingPolar, optim0.wing.aero.sectionPolars] = LiftingLine(optim.wing.aero.sectionPolars.xQC_m, optim.wing.aero.sectionPolars.yQC_m, optim.wing.aero.sectionPolars.zQC_m, optim.wing.aero.sectionPolars.c_m, optim.wing.aero.sectionPolars.twists_deg, AoAs_deg(a), optim.wing.aero.sectionPolars, wingPolar);
        
        % Protect against unconverged results
        isConverged = wingPolar.isConverged(end);
        if isConverged
            CL(a) = wingPolar.CL(end);
            CDi(a)= wingPolar.CDi(end);
        else
            CL(a) = NaN;
            CDi(a)= NaN;
        end
        if isnan(CL(a)) && a>3 % If we failed three times in a row, exit
            if isnan(CL(a-1)) & isnan(CL(a-2))
                CL(a:end) = NaN;
                CDi(a:end) = NaN;
                break
            end
        end
    end
    
    % Remove the last unconverged points and interpolate to remove 
    % unconverged values
 	CDi = CDi(1:find(~isnan(CDi), 1, 'last'));
    CL = CL(1:find(~isnan(CL), 1, 'last'));
    AoAs_deg = AoAs_deg(1:length(CL));
    CL(isnan(CL)) = interp1(AoAs_deg(~isnan(CL)), CL(~isnan(CL)), AoAs_deg(isnan(CL)), 'linear', 'extrap');
    CDi(isnan(CDi)) = interp1(AoAs_deg(~isnan(CDi)), CDi(~isnan(CDi)), AoAs_deg(isnan(CDi)), 'linear', 'extrap');
    optim.wing.aero.CLmax = max(CL);
    optim.wing.aero.CLmin = min(CL);
        
    % Make use of above data to update the wing root twist for cruise
    % conditions
    AoAcruise_deg = interp1f(CL, AoAs_deg, optim.wing.aero.CLcruise);
    optim.wing.aero.sectionPolars.twists_deg = optim.wing.aero.sectionPolars.twists_deg + AoAcruise_deg;
    
    % If a winglet is defined, ensure the tip twist is zero
    if isfield(optim.wing, 'winglet')
        if optim.wing.winglet.zoc > 0
            optim.wing.aero.sectionPolars.twists_deg(1:2) = 0;
            optim.wing.aero.sectionPolars.twists_deg(end-1:end) = 0;
        end
    end
    
    optim.wing.twists_deg = optim.wing.twists_deg + AoAcruise_deg;
    
    % Since wing twist has changed, re-compute normal vectors for solar
    % panelling
    optim.wing.solar = rmfield(optim.wing.solar, 'triangulation');
    
    % Populate output of lifting line as ASWING output at an appropriate CL
    AoAs_deg = AoAs_deg - AoAcruise_deg;
	pCL = polyfit(AoAs_deg(CL > .4 & CL < 1), CL(CL > .4 & CL < 1), 1);
    pCLcruise = polyfit(AoAs_deg(abs(CL - optim.wing.aero.CLcruise) < .15), CL(abs(CL - optim.wing.aero.CLcruise) < .15), 1);
    optim.aircraft.aero.alphaCruise_deg = 0;
    wingPolar = []; % Resetting polar stack to avoid convergence issue
    [wingPolar, optim0.wing.aero.sectionPolars] = LiftingLine(optim.wing.aero.sectionPolars.xQC_m, optim.wing.aero.sectionPolars.yQC_m, optim.wing.aero.sectionPolars.zQC_m, optim.wing.aero.sectionPolars.c_m, optim.wing.aero.sectionPolars.twists_deg, optim.aircraft.aero.alphaCruise_deg, optim.wing.aero.sectionPolars, wingPolar);
    
    % Fake ASWING output for drag and moment calculation
    optim0 = PackageLL2ASWING(optim, wingPolar);
    optim0 = LiftAndProfileDragCorrection(optim0, 'wing');
    
    % Find CLalpha and alpha0 through a simple polyfit
    optim.wing.aero.CMcruise   	= interp1f(optim0.wing.aero.polar.Re, optim0.wing.aero.polar.Cm{1}, Re);
    optim.aircraft.aero.CLcruise = optim.wing.aero.CLcruise;
    e                           = .97; % First guess
    optim.wing.aero.CDcruise   	= interp1f(optim0.wing.aero.polar.Re, optim0.wing.aero.polar.CDp{1}, Re) + optim.wing.aero.CLcruise^2/(pi*e*optim.wing.AR);
    optim.wing.aero.CLalpha   	= pCL(1) * 180/pi;
    optim.wing.aero.CLcruiseAlpha = pCLcruise(1) * 180/pi;
    optim.wing.aero.alpha0_deg	= -pCL(2)/pCL(1);
    
    % Initial assessment of the tail
    if isfield(optim, 'htail')
        optim.htail.aero.CMcruise = 0;
        optim.htail.aero.CDcruise = 0;
    end
    
end