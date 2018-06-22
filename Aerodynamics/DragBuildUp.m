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
  
% This function sums all the drag sources and stores the consolidated
% polars vs Re at the root level of the optim structure:
% optim.polar.AoA_deg    is a (Nx1) vector,
% optim.polar.Re         is a (Mx1) vector,
% optim.polar.CL and optim.aero.CD are both (MxN) matrices.
function optim = DragBuildUp(optim)

% Reference Re, AoA
polar.Re = optim.wing.aero.polar.Re * optim.aircraft.cref_m/optim.wing.c_m(1);
polar.AoA_deg = optim.aircraft.aero.ASWINGpolar.AoA_deg;

% Combine lifting surface polars into one with respect to the overall Re
polar.CL	= 0;
polar.CDp  	= 0;

for b = 1:length(optim.aircraft.beamNames)
    if ~max(strcmp(optim.aircraft.beamNames{b}, {'pod', 'boom'}))
        Re	= polar.Re * optim.(optim.aircraft.beamNames{b}).c_m(1) / optim.aircraft.cref_m;
        
        for i = 1:length(optim.(optim.aircraft.beamNames{b}).N)
            CL	= interp1(optim.(optim.aircraft.beamNames{b}).aero.polar.Re, optim.(optim.aircraft.beamNames{b}).aero.polar.CL{i}, Re, 'nearest', 'extrap'); 
            CDp	= interp1(optim.(optim.aircraft.beamNames{b}).aero.polar.Re, optim.(optim.aircraft.beamNames{b}).aero.polar.CDp{i}, Re, 'nearest', 'extrap');

            % Add to polar
            polar.CL  = polar.CL  + CL *optim.(optim.aircraft.beamNames{b}).Sref_m2/optim.aircraft.Sref_m2*optim.(optim.aircraft.beamNames{b}).N(i);
            polar.CDp = polar.CDp + CDp*optim.(optim.aircraft.beamNames{b}).Sref_m2/optim.aircraft.Sref_m2*optim.(optim.aircraft.beamNames{b}).N(i);
        end    
    end
end

% Induced drag
e         = optim.aircraft.aero.ASWINGpolar.CL'.^2./(pi * optim.wing.AR * optim.aircraft.aero.ASWINGpolar.CDi');
polar.CDi = polar.CL.^2./(pi * ones(length(polar.Re), 1) * e * optim.wing.AR);

% For post-processing only
polar.extraCDpPercentage = polar.CDp;

% Additional drag sources vs Re

% For the booms, use a friction factor with Form Factor (FF)
if max(contains(optim.aircraft.beamNames, 'boom'))
    
        % Aft piece
        SWetBoom_m2     = optim.boom.aftWettedArea_m2; % for the boom, the chord is the diameter
        lod             = (optim.vtail.x_m - optim.boom.fuselage.length_m) ./ optim.boom.structure.chord_m{:}(end);
        FFBoom          = 1 + 2.2 * (1./lod).^(1.5) + 3.8 * (1./lod).^3; % Torenbeek formula
        ReBoom          = [(optim.vtail.x_m - optim.boom.fuselage.length_m ) / optim.aircraft.cref_m]' * polar.Re;
        CFBoom          = 0.455 ./ log10(ReBoom).^(2.58); % Prandtl empirical formula
        CDBoom          = (FFBoom .* SWetBoom_m2)' * CFBoom/ optim.aircraft.Sref_m2;

        for b = 1:length(optim.boom.N)
            polar.CDp       = polar.CDp + optim.boom.N(b) * CDBoom(b, :)' * ones(1, length(polar.AoA_deg));
        end
        
        % Fore fairing
        SBoomFairingRef_m2 = (optim.boom.V_m3).^(2/3);
        polar.CDp          = polar.CDp + sum(optim.boom.N .* optim.boom.CDFairing .* SBoomFairingRef_m2) / optim.aircraft.Sref_m2 * ones(length(polar.Re), length(polar.AoA_deg));
    end
    
	% Pod fairing
    SPodFairingRef_m2 = (optim.pod.V_m3).^(2/3);
    polar.CDp         = polar.CDp + sum(optim.pod.N .* optim.pod.CDFairing .* SPodFairingRef_m2) /optim.aircraft.Sref_m2 * ones(length(polar.Re), length(polar.AoA_deg));
    
    % Payload drag
    polar.CDp	= polar.CDp + optim.payload.CD * optim.payload.Sref_m2 / optim.aircraft.Sref_m2;
    
    % Propulsion
    polar.CDp   = polar.CDp + sum(optim.propulsion.N) * (optim.propulsion.CD * optim.propulsion.Sref_m2 / optim.aircraft.Sref_m2);
    
% Allowance of 10% for interference drag
polar.CDp   = 1.10 * polar.CDp;

% Drag with margin
polar.CD   = (polar.CDp + polar.CDi) * (1 + optim.margin.drag_percent);

% Assign
optim.aircraft.aero.polar = polar;

end
