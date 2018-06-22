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
  
function optim = LiftAndProfileDragCorrection(optim, surface)
% This function extracts the induced flow field from ASWING calculations
% and uses it to look-up actual sectional performance from the airfoil db.
% Lift, profile and pressure drags, are reconstructed (see AIAA papers for
% details).

% Clear matrices in case dimensions changed
optim.(surface).aero.polar.CDpp = {[]};
optim.(surface).aero.polar.CDf  = {[]};
optim.(surface).aero.polar.CDp  = {[]};
optim.(surface).aero.polar.CL   = {[]};
optim.(surface).aero.polar.Cm   = {[]};
 
% Bullet proof against bad aswing output. Sometimes ASWING misses a couple
if isfield(optim.aircraft, 'aero')
    if length(optim.aircraft.aero.ASWINGpolar.CL)>1
        optim = RemoveASWOutliers(optim, surface);
    end
end

for i = 1:length(optim.(surface).N)

    % Gather polars vs span, deflection, Re, alpha from
    optim = PopulateSectionPolars(optim, surface, {'cl', 'cd', 'cdp', 'cm'});
    polar = optim.(surface).aero.sectionPolars;
    ADB   = optim.(surface).aero.airfoilADB;
    
    % Record how many control surfaces are on the surface itself
    if isfield(optim.(surface), 'controlSurface')
        Nc = length(optim.(surface).controlSurface);
    else
        Nc = 0;
    end

    % Harvest ASWING output. Create substructure that will contain the
    % processed values.
    qtities = {'deltax_m', 'y_m', 'deltaz_m', 'phi_deg', 'theta_deg', 'psi_deg', 'c_m', 'cl', 'f_lift_N_m'};
    
    % Coordinates and angles are provided at beam control points. Remaining
    % quantities are given at mid points. Discard last dummy value and
    % interpolate back at control points
    ASWING.t    = optim.(surface).aero.ASWINGpolar.t{i};
    if max(ASWING.t) > max(optim.(surface).aero.sectionPolars.t)
        ASWING.t    = ASWING.t/(max(ASWING.t)+eps) * max(optim.(surface).aero.sectionPolars.t);
    end
    ASWING.tMid = ASWING.t(1:end-1) + diff(ASWING.t)/2;
       
    % Figure out aero data and twist at beam t indices
    t = optim.(surface).aero.sectionPolars.t;
    ToCs = interp1f(optim.(surface).aero.sectionPolars.t, optim.(surface).aero.sectionPolars.toc, ASWING.t);
    twists_deg = interp1f(optim.(surface).aero.sectionPolars.t, optim.(surface).aero.sectionPolars.twists_deg, ASWING.t);

    % Interpolate polars along the span t span locations
    siz = size(polar.cl);
    for q = {'cl', 'cd', 'cdp', 'cm'}
        polar.([q{1} '0']) = ScaleTimeBulletProof(cat(1, polar.(q{1})(end:-1:2, :, :, :), polar.(q{1})), t, ASWING.t);
        polar.([q{1} '0']) = reshape(polar.([q{1} '0']), [length(ASWING.t) siz(2:end)]);
    end
    
    % Iterate along ASWING polar alphas
    for a = 1:length(optim.(surface).aero.ASWINGpolar.AoA_deg)
        
        % Free-stream direction
        eta = [cosd(optim.(surface).aero.ASWINGpolar.AoA_deg(a)); 0; sind(optim.(surface).aero.ASWINGpolar.AoA_deg(a))];
        
        % Retrieve ASWING quantities and carry appropriate interpolation
        for q = 1:length(qtities)
            if ~strcmp(qtities{q}, {'deltax_m', 'y_m', 'deltaz_m'})
                ASWING.(qtities{q}){i} = interp1(ASWING.tMid, optim.(surface).aero.ASWINGpolar.(qtities{q}){i}(a, 1:end-1), ASWING.t, 'linear', 'extrap');
            else
                ASWING.(qtities{q}){i} = optim.(surface).aero.ASWINGpolar.(qtities{q}){i}(a, :);
            end
        end
        
        % Ensure that tip loading is zero
        ASWING.cl{i}([1 end]) = 0;
        ASWING.f_lift_N_m{i}([1 end]) = 0;
        
        % Polar was run at SL conditions. Non-dimentionalize quantities.
        V_ms(a) = sqrt(2*optim.MGTOWcurrent_kg*optim.constants.g_ms2/(optim.constants.rhoSL_kgm3*optim.wing.Sref_m2*optim.aircraft.aero.ASWINGpolar.CL(a)));
        q_Pa(a) = 1/2*optim.constants.rhoSL_kgm3*V_ms(a)^2;
        ASWING.f_lift_m{i} = ASWING.f_lift_N_m{i}/q_Pa(a);
        
        % Compute distance from tip to tip along es
        ASWING.x_m{i} = interp1f(optim.(surface).aero.sectionPolars.t, optim.(surface).aero.sectionPolars.x_m, ASWING.t) + ASWING.deltax_m{i};
        ASWING.z_m{i} = interp1f(optim.(surface).aero.sectionPolars.t, optim.(surface).aero.sectionPolars.z_m, ASWING.t) + ASWING.deltaz_m{i};
        es0 = [diff(ASWING.x_m{i}); diff(ASWING.y_m{i}); diff(ASWING.z_m{i})]';
        ds  = sqrt(sum(es0.^2, 2))';
        s_m(a, :) = [0 cumsum(ds)];
        
        % Bullet proof against unconverged results
        if max(isnan(optim.(surface).aero.ASWINGpolar.cl{i}(a, :)))
            optim.(surface).aero.polar.CDpp{i}(:, a) = NaN * ones(length(optim.(surface).aero.airfoilADB.dim4.vector), 1);
            optim.(surface).aero.polar.CDf{i}(:, a)  = NaN * ones(length(optim.(surface).aero.airfoilADB.dim4.vector), 1);
            optim.(surface).aero.polar.CDp{i}(:, a)  = NaN * ones(length(optim.(surface).aero.airfoilADB.dim4.vector), 1);
            optim.(surface).aero.polar.CL{i}(:, a)   = NaN * ones(length(optim.(surface).aero.airfoilADB.dim4.vector), 1);
            optim.(surface).aero.polar.Cm{i}(:, a)   = NaN * ones(length(optim.(surface).aero.airfoilADB.dim4.vector), 1);
        else
            for cs = 1:Nc
                
                % Build deflection vector for polar interpolations
                csRightFlag = interp1f(optim.(surface).aero.sectionPolars.t, optim.(surface).aero.sectionPolars.FlagRightDeflection{cs}, ASWING.t);
                csRightFlag(csRightFlag<1) = 0;
                csRightFlag(find(isnan(csRightFlag))) = 0;
                csLeftFlag  = interp1f(optim.(surface).aero.sectionPolars.t, optim.(surface).aero.sectionPolars.FlagLeftDeflection{cs}, ASWING.t);
                csLeftFlag(csLeftFlag<1) = 0;
                csLeftFlag(find(isnan(csLeftFlag))) = 0;
                
                % If the whole surface deflects, left and right surfaces
                % will overlap At exactly zero. In this case, make the
                % deflection flag = 1/2 on both surfaces at zero s.t.
                % defl_right(0)+defl_left(0) = 1 instead of two
                flagOverLap = csLeftFlag + csRightFlag == 2;
                csLeftFlag(flagOverLap) = 1/2;
                csRightFlag(flagOverLap) = 1/2;
                 
                if length(optim.(surface).aero.ASWINGpolar.controlSurface(cs).deflection_deg(:, a)) > 1
                    defRight_deg = optim.(surface).aero.ASWINGpolar.controlSurface(cs).deflection_deg(1, a);
                    defLeft_deg  = optim.(surface).aero.ASWINGpolar.controlSurface(cs).deflection_deg(2, a);
                else
                    defRight_deg = optim.(surface).aero.ASWINGpolar.controlSurface(cs).deflection_deg(1, a);
                    defLeft_deg  = optim.(surface).aero.ASWINGpolar.controlSurface(cs).deflection_deg(1, a);
                end
                deflections_deg(:, cs) = csLeftFlag * defLeft_deg + csRightFlag * defRight_deg;
                
            end
            
            % If there are no control surfaces, set deflections to 0
            if Nc == 0
                deflections_deg= zeros(length(ASWING.t), 1);
            end
            
            % To interpolate along deflection, upgrade it to the first
            % dimension
            for secQ = {'cl', 'cd', 'cdp', 'cm'}
                
                polar.([secQ{1} '1'])  = permute(polar.([secQ{1} '0']), [2 1 3 4]);
                siz                    = size(polar.([secQ{1} '1']));
                polar.([secQ{1} '1'])  = ScaleTimeBulletProof(polar.([secQ{1} '1']), ADB.dim3.vector, sum(deflections_deg, 2));
                polar.([secQ{1} '1'])  = reshape(polar.([secQ{1} '1']), [size(deflections_deg, 1) siz(2:end)]);
                
                % Only diagonal is of interest
                [m,n,p,q] = size(polar.([secQ{1} '1']));
                E = eye(m);
                e = logical(reshape(E, m*n, 1));
                polar.([secQ{1} '2']) = reshape(polar.([secQ{1} '1']), m*n, p, q);
                polar.([secQ{1} '2']) = polar.([secQ{1} '2'])(e, :, :);
                polar.(secQ{1})       = reshape(polar.([secQ{1} '2']), m, p, q);
                
                % polar.q{1} has the dimentions of span * Re * AoA
                % Interpolate cd based on cls
                polar.(secQ{1}) = permute(polar.(secQ{1}), [3 1 2]);
            end
            
            % Record the variation of the sectional polars with Re and AoA        
            for s = 1:m % Span location
                
                % Find the control surface index at the current station
                [~, csIndex] = max(abs(deflections_deg(s, :)));
                
                % Reconstruct the local c-s-n beam coordinate system
                Rtwist = [  cosd(ASWING.theta_deg{i}(s) + twists_deg(s)) 	0   -sind(ASWING.theta_deg{i}(s)+ twists_deg(s));
                    0                                     	1   0;
                    sind(ASWING.theta_deg{i}(s) + twists_deg(s))	0   cosd(ASWING.theta_deg{i}(s)+ twists_deg(s))];
                
                Ryaw = [cosd(ASWING.psi_deg{i}(s))     sind(ASWING.psi_deg{i}(s))	0;
                    -sind(ASWING.psi_deg{i}(s))	cosd(ASWING.psi_deg{i}(s))	0;
                    0                           0                       1];
                
                Rdihedral = [1	0                           0;
                    0	cosd(ASWING.phi_deg{i}(s))     sind(ASWING.phi_deg{i}(s));
                    0 	-sind(ASWING.phi_deg{i}(s))	cosd(ASWING.phi_deg{i}(s))];
                
                % Tensor matrix
                T = Rtwist * Ryaw * Rdihedral;
                
                % Local coordinate system
                ec(:, s) = T(1, :)';
                es(:, s) = T(2, :)';
                en(:, s) = T(3, :)';
                                
                % Estimation of the span oriented flow field Vs = (V.Es)Es
                % We assume that it's constant across streamlines and therefore
                % Vs = (Vinf.Es)Es
                VsoVinf(s) = dot(eta, es(:, s));
                
                % Find the local aswing aero properties (from the aswing adb)
                % to back out the actual angle of attack
                alpha0 = interp1f(optim.(surface).aero.aswingPerfAirfoilADB.t, optim.(surface).aero.aswingPerfAirfoilADB.alpha,  ASWING.t(s));
                dCLda  = interp1f(optim.(surface).aero.aswingPerfAirfoilADB.t, optim.(surface).aero.aswingPerfAirfoilADB.dCLda,  ASWING.t(s));
                dCLdF  = interp1f(optim.(surface).aero.aswingPerfAirfoilADB.t, optim.(surface).aero.aswingPerfAirfoilADB.dCLdF{csIndex}, ASWING.t(s));
                
                % Reconstruct the full angle of attack (includes net
                % twist due to deflection)
                alphas_deg = (ASWING.cl{i}(s) - dCLdF * deflections_deg(s) * pi/180 ) ...
                    / dCLda * 180/pi - alpha0;
                
                alphas_deg(alphas_deg < min(polar.AoA_deg)) = min(polar.AoA_deg);
                alphas_deg(alphas_deg > max(polar.AoA_deg)) = max(polar.AoA_deg);
                
                % Reconstruct local flow field from ASWING recorded
                % lift force and cl:
                % f_aero = rho Vperp Gamma, cl = 2*Gamma/(rho*Vperp)
                if s == 1 || s == length(ASWING.t) || ASWING.cl{i}(s) == 0
                    VperpNormoVinf(s) = norm(eta - VsoVinf(s)*es(:, s));
                else
                    VperpNormoVinf(s) = sqrt(abs(ASWING.f_lift_m{i}(s) ./(ASWING.c_m{i}(s).* ASWING.cl{i}(s))))';
                end
                VperpoVinf(:, s)  = VperpNormoVinf(s) * (cosd(alphas_deg)*ec(:, s) ...
                    + sind(alphas_deg)*en(:, s));
                VoVinf(:, s)  = VsoVinf(s)*es(:, s) + VperpoVinf(:, s);
                VNormoVinf(s) = norm(VoVinf(:, s));
                
                % For CL, CDp integration: record local Vperp
                % component along the lift (zeta) and free stream
                % (zeta) directions:
                % (Vperp x es).(eta and zeta)
                VperoVinfXes          = cross(VperpoVinf(:, s), es(:, s));
                zeta                  = cross(eta, [0; 1; 0]);
                VperpoVinfXesDzeta(s) = dot(cross(VperpoVinf(:, s), es(:, s)), zeta);
                VoVinfDeta(s)         = dot(VoVinf(:, s), eta);
                VperpoVinfDeta        = dot(VperpoVinf(:, s), eta);
                esDey(s)              = dot(es(:, s), [0 1 0]);
               
                % For CM integration. Reference point is root feathering
                % axis i.e [0, 0, 0] at the root
                x0_m(s) = optim.(surface).structure.Xax{i}(1)*ASWING.c_m{i}(s);
                xRoot_m = interp1f(ASWING.t, ASWING.x_m{i}, 0);
                zRoot_m = interp1f(ASWING.t, ASWING.z_m{i}, 0);
                r0_m    = [ASWING.x_m{i}(s); ASWING.y_m{i}(s); ASWING.z_m{i}(s)] - [xRoot_m; 0; zRoot_m];
                rQC_m   = (ASWING.c_m{i}(s)/4 - x0_m(s))*ec(:, s);
                DeltarXliftDey_m(s) = dot(cross(r0_m + rQC_m, VperoVinfXes), [0; 1; 0]);
                
                for r = 1:p % Reynolds number
                    
                    for secQ = {'cl', 'cd', 'cdp', 'cm'}
                        
                        polar.([secQ{1} '3']) = polar.(secQ{1})(:, s, r);
                        
                        % Make sure that the interpolation will go smoothly by removing
                        % any potential NaNs or non monotonic values in cl
                        polar.([secQ{1} '3'])(isnan(polar.cl3)) = [];
                        
                        % Lookup of corrected sectional coefficients based on
                        % perpendicular angle of attack and local Re
                        polar.([secQ{1} '4'])(r, s) = interp1f(polar.AoA_deg', polar.([secQ{1} '3']), alphas_deg);
                    end
                end
            end
                        
            % Store c-s-n coordinate system for further deflected CG
            % calculations
            optim.(surface).aero.ASWINGpolar.ec{i}(a, :, :) = ec;
            optim.(surface).aero.ASWINGpolar.es{i}(a, :, :) = es;
            optim.(surface).aero.ASWINGpolar.en{i}(a, :, :) = en;
                        
            % For a given planform and reference chord Reynolds, each section
            % operates at a different Re
            Re      = optim.(surface).aero.airfoilADB.dim4.vector;
            AoA_deg = optim.(surface).aero.ASWINGpolar.AoA_deg;
            
            for re = 1:length(Re)
                
                % Distribution of Re along the span. The boundary layer
                % develops and grows over the entire chord, not just along
                % the span normal projection: c = cPerp/cos(psi_deg)
                ReSpan = Re(re) * ASWING.c_m{i}./cosd(ASWING.psi_deg{i}) /optim.(surface).c_m(1);
                
                % Some of the Reynolds numbers may go much below what was recorded
                % as the minimum Re in the airfoil db. Saturate these values.
                ReSpan(ReSpan<=min(Re)) = min(Re);
                ReSpan(ReSpan>=max(Re)) = max(Re);
                
                % Derive sectional distribution
                selector = find(reshape(eye(length(ReSpan)), length(ReSpan)^2, 1));
                for secQ = {'cl', 'cd', 'cdp', 'cm'}
                    polar.([secQ{1} 'Span']) = ScaleTimeBulletProof(polar.([secQ{1} '4']), Re, ReSpan);
                    polar.([secQ{1} 'Span']) = reshape(polar.([secQ{1} 'Span']), length(polar.([secQ{1} 'Span']))^2, 1);
                    polar.([secQ{1} 'Span']) = polar.([secQ{1} 'Span'])(selector)';
                end
                
                % Store cl distribution for tip stall check
                optim.(surface).aero.ASWINGpolar.clCorrected(a, re, :) = polar.clSpan;
                
                % Integrate
                S_m2 = trapz(s_m(a, :), ASWING.c_m{i});
               
                % Forces
                optim.(surface).aero.polar.CL{i}(re, a)  = trapz(s_m(a, :), ASWING.c_m{i}'.*polar.clSpan'.*VperpNormoVinf'.*VperpoVinfXesDzeta')/S_m2;
                optim.(surface).aero.polar.CDpp{i}(re, a)= trapz(s_m(a, :), ASWING.c_m{i}'.*polar.cdpSpan'.*VperpNormoVinf'.*VperpoVinfDeta')/S_m2;
                optim.(surface).aero.polar.CDf{i}(re, a) = trapz(s_m(a, :), ASWING.c_m{i}'.*(polar.cdSpan-polar.cdpSpan)'.*VNormoVinf'.*VoVinfDeta')/S_m2;
                optim.(surface).aero.polar.CDp{i}(re, a) = optim.(surface).aero.polar.CDpp{i}(re, a) + optim.(surface).aero.polar.CDf{i}(re, a);
                                
                % Moment due to lift. Corrected moment + moment due to
                % lift. Reference point: root quarter chord. Non-dimentionalized by root chord
                optim.(surface).aero.polar.Cm{i}(re, a)  = 1/(S_m2*optim.(surface).c_m(1)) * (trapz(s_m(a, :), ASWING.c_m{i}'.^2 .* polar.cmSpan' .* VperpNormoVinf'.^2 .* esDey') + ...
                                                                                             trapz(s_m(a, :), ASWING.c_m{i}'.*polar.clSpan'.*VperpNormoVinf'.*DeltarXliftDey_m'));
            end
        end
        
        % Record AoA, Re
        optim.(surface).aero.polar.Re = Re;
        optim.(surface).aero.polar.AoA_deg = AoA_deg;
    end
    
    % Bullet proof against crazy ASWING output. Sometimes the deflections are
    % unphysical, resulting in NaNs. Remove and interpolate along AoAs.
    fields = {'CDpp', 'CDf', 'CDp', 'CL', 'Cm'};
    
    for f = 1:length(fields)
        [indexIIssues, indexJIssues] = find(isnan(optim.(surface).aero.polar.(fields{f}){i}));
        uniqueIindices = unique(indexIIssues);
        
        for j = 1:length(uniqueIindices)
            
            jIndices = indexJIssues( indexIIssues == uniqueIindices(j) );
            AoA_deg1 = AoA_deg;
            AoA_deg1(jIndices) = [];
            Matrix = optim.(surface).aero.polar.(fields{f}){i}(uniqueIindices(j), :);
            Matrix(jIndices) = [];
            try
            optim.(surface).aero.polar.(fields{f}){i}(uniqueIindices(j), jIndices) = interp1(AoA_deg1, Matrix, AoA_deg(indexJIssues(j)), 'linear', 'extrap');
            catch
                1
            end
        end
    end
end
end