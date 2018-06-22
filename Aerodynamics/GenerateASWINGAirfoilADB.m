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
  
function ASWINGAirfoilADB = GenerateASWINGAirfoilADB(airfoilADB, csoc, ToCs, Re)
% This routine looks up each sectional aerodynamic quantity required by 
% ASWING at a given thickness to chord, Reynolds number, and relative 
% control surface length to chord ratio.

% Look for the appropriate index for the control surface length - assumes
% the ADB was built with it in.
for cs = 1:length(csoc)
    [~, c0(cs)] = min( abs(airfoilADB.dim2.vector - csoc(cs)));
end
d0      = find(airfoilADB.dim3.vector == 0); % no deflection index

% Initialize output
ASWINGAirfoilADB.toc = airfoilADB.dim1.vector';

% If another thickness was added but is not in the reduced quantities,
% reduce the thickness vector
if length(airfoilADB.dim1.vector) > size(airfoilADB.ASWINGReducedQuantities.clmax, 1)
    airfoilADB.dim1.vector = airfoilADB.dim1.vector(1: size(airfoilADB.ASWINGReducedQuantities.clmax, 1));
end

% Saturate on Re if too high at SL
Re = min(Re, max(airfoilADB.dim4.vector));

% Saturate if Re is too small for the prescribed DB
Re = max(Re, min(airfoilADB.dim4.vector));

for t = 1:length(Re)
    
    if length(airfoilADB.dim1.vector) > 1
    
    % Angle of attack from linear portion
    %alpha = interp2f(airfoilADB.dim4.vector, airfoilADB.dim5.vector, squeeze(airfoilADB.ASWINGReducedQuantities.alphaLin_deg(t, c0(1), d0, :, :))', Re(t), 0);
    alpha = interpn(airfoilADB.dim1.vector, airfoilADB.dim4.vector, airfoilADB.dim5.vector, squeeze(airfoilADB.ASWINGReducedQuantities.alphaLin_deg(:, c0(1), d0, :, :)), ToCs(t), Re(t), 0);
     
    % Select single values based on input Re, alpha, etc..
	ASWINGAirfoilADB.Cdp(t, 1)   = interpn(airfoilADB.dim1.vector, airfoilADB.dim4.vector, airfoilADB.dim5.vector, squeeze(airfoilADB.table4.values(:, c0(1), d0, :, :)), ToCs(t), Re(t), alpha);
    ASWINGAirfoilADB.Cdf(t, 1)   = interpn(airfoilADB.dim1.vector, airfoilADB.dim4.vector, airfoilADB.dim5.vector, squeeze(airfoilADB.table2.values(:, c0(1), d0, :, :)), ToCs(t), Re(t), alpha) - ASWINGAirfoilADB.Cdp(t);
    ASWINGAirfoilADB.alpha(t, 1) = interpn(airfoilADB.dim1.vector, airfoilADB.dim4.vector, airfoilADB.dim5.vector, squeeze(airfoilADB.ASWINGReducedQuantities.alpha0_deg(:, c0(1), d0, :, :)), ToCs(t), Re(t), alpha);
    ASWINGAirfoilADB.Cm(t, 1)    = interpn(airfoilADB.dim1.vector, airfoilADB.dim4.vector, airfoilADB.dim5.vector, squeeze(airfoilADB.table3.values(:, c0(1), d0, :, :)), ToCs(t), Re(t), alpha);
    ASWINGAirfoilADB.CLmax(t, 1) = interpn(airfoilADB.dim1.vector, airfoilADB.dim4.vector, airfoilADB.dim5.vector, squeeze(airfoilADB.ASWINGReducedQuantities.clmax(:, c0(1), d0, :, :)), ToCs(t), Re(t), alpha);
    ASWINGAirfoilADB.CLmin(t, 1) = interpn(airfoilADB.dim1.vector, airfoilADB.dim4.vector, airfoilADB.dim5.vector, squeeze(airfoilADB.ASWINGReducedQuantities.clmin(:, c0(1), d0, :, :)), ToCs(t), Re(t), alpha);
    ASWINGAirfoilADB.dCLda(t, 1) = interpn(airfoilADB.dim1.vector, airfoilADB.dim4.vector, airfoilADB.dim5.vector, squeeze(airfoilADB.ASWINGReducedQuantities.clalpha(:, c0(1), d0, :, :)), ToCs(t), Re(t), alpha);
    
    
    for cf = 1:length(csoc)
        ASWINGAirfoilADB.dCLdF{cf}(t) = interpn(airfoilADB.dim1.vector, airfoilADB.dim4.vector, airfoilADB.dim5.vector, squeeze(airfoilADB.ASWINGReducedQuantities.dcldf1(:, c0(cf), d0, :, :)), ToCs(t), Re(t), alpha);
        ASWINGAirfoilADB.dCDdF{cf}(t) = interpn(airfoilADB.dim1.vector, airfoilADB.dim4.vector, airfoilADB.dim5.vector, squeeze(airfoilADB.ASWINGReducedQuantities.dcddf1(:, c0(cf), d0, :, :)), ToCs(t), Re(t), alpha);
        ASWINGAirfoilADB.dCMdF{cf}(t) = interpn(airfoilADB.dim1.vector, airfoilADB.dim4.vector, airfoilADB.dim5.vector, squeeze(airfoilADB.ASWINGReducedQuantities.dcmdf1(:, c0(cf), d0, :, :)), ToCs(t), Re(t), alpha);
    end
    
    else
        alpha = interp2f(airfoilADB.dim4.vector, airfoilADB.dim5.vector, squeeze(airfoilADB.ASWINGReducedQuantities.alphaLin_deg(1, c0(1), d0, :, :))', Re(t), 0);
        ASWINGAirfoilADB.Cdp(t, 1)   = interp2f(airfoilADB.dim4.vector, airfoilADB.dim5.vector, squeeze(airfoilADB.table4.values(1, c0(1), d0, :, :))', Re(t), alpha);
        ASWINGAirfoilADB.Cdf(t, 1)   = interp2f(airfoilADB.dim4.vector, airfoilADB.dim5.vector, squeeze(airfoilADB.table2.values(1, c0(1), d0, :, :))', Re(t), alpha) - ASWINGAirfoilADB.Cdp(t);
        ASWINGAirfoilADB.alpha(t, 1) = interp2f(airfoilADB.dim4.vector, airfoilADB.dim5.vector, squeeze(airfoilADB.ASWINGReducedQuantities.alpha0_deg(1, c0(1), d0, :, :))', Re(t), alpha);
        ASWINGAirfoilADB.Cm(t, 1)    = interp2f(airfoilADB.dim4.vector, airfoilADB.dim5.vector, squeeze(airfoilADB.table3.values(1, c0(1), d0, :, :))', Re(t), alpha);
        ASWINGAirfoilADB.CLmax(t, 1) = interp2f(airfoilADB.dim4.vector, airfoilADB.dim5.vector, squeeze(airfoilADB.ASWINGReducedQuantities.clmax(1, c0(1), d0, :, :))', Re(t), alpha);
        ASWINGAirfoilADB.CLmin(t, 1) = interp2f(airfoilADB.dim4.vector, airfoilADB.dim5.vector, squeeze(airfoilADB.ASWINGReducedQuantities.clmin(1, c0(1), d0, :, :))', Re(t), alpha);
        ASWINGAirfoilADB.dCLda(t, 1) = interp2f(airfoilADB.dim4.vector, airfoilADB.dim5.vector, squeeze(airfoilADB.ASWINGReducedQuantities.clalpha(1, c0(1), d0, :, :))', Re(t), alpha);
        
        for cf = 1:length(csoc)
            ASWINGAirfoilADB.dCLdF{cf}(t) = interp2f(airfoilADB.dim4.vector, airfoilADB.dim5.vector, squeeze(airfoilADB.ASWINGReducedQuantities.dcldf1(1, c0(cf), d0, :, :))', Re(t), alpha);
            ASWINGAirfoilADB.dCDdF{cf}(t) = interp2f(airfoilADB.dim4.vector, airfoilADB.dim5.vector, squeeze(airfoilADB.ASWINGReducedQuantities.dcddf1(1, c0(cf), d0, :, :))', Re(t), alpha);
            ASWINGAirfoilADB.dCMdF{cf}(t) = interp2f(airfoilADB.dim4.vector, airfoilADB.dim5.vector, squeeze(airfoilADB.ASWINGReducedQuantities.dcmdf1(1, c0(cf), d0, :, :))', Re(t), alpha);
        end
    end
end

end
