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
  
function [ optim ] = WriteVtail(  optim, aswingIn, useStructure, relfectionY_m, beamNumber, aeroDataBaseName , entityName, index )
% Write vtail(s) to .asw file.

% Select the appropriate aeroDataBase 
if strcmp(aeroDataBaseName, 'loadsAeroDataBase')
    aeroDataBase = optim.vtail.aero.aswingLoadsAirfoilADB;
elseif strcmp(aeroDataBaseName, 'Performace')
    aeroDataBase = optim.vtail.aero.aswingPerfAirfoilADB;
elseif strcmp(aeroDataBaseName, 'AeroCoefficients')
    aeroDataBase = optim.vtail.aero.aswingPerfAirfoilADB;
    aeroDataBase.dCLda = zeros(size(aeroDataBase.dCLda))+.01;
else
    error('Aero Database not specified');
end
 
fprintf(aswingIn,'\n');
 
fprintf(aswingIn,'#============\n');
fprintf(aswingIn,['Beam ' num2str(beamNumber) '\n']);
fprintf(aswingIn,[entityName '\n']);
fprintf(aswingIn,'#\n');
  
%vtail Aero
fprintf(aswingIn,'\n');
fprintf(aswingIn,'   t        chord   x       y       z       twist     Xax\n');
l = length(optim.vtail.structure.t{index})+1;
 
for i = 1:length(optim.vtail.structure.t{index}) 
    fprintf(aswingIn,'\t%3.4f\t%3.4f\t%3.4f\t%3.4f\t%3.4f\t%3.4f\t%3.4f\n', ...
        -1*optim.vtail.structure.t{index}(l-i), optim.vtail.structure.chord_m{index}(l-i), optim.vtail.globalx_m{index}(l-i), relfectionY_m * optim.vtail.globaly_m{index}(i), ...
        optim.vtail.z_m - (optim.vtail.globalz_m{index}(l-i)-optim.vtail.z_m), optim.vtail.structure.twist_deg{index}(l-i), optim.vtail.structure.Xax{index}(l-i));
end
for i = 1:length(optim.vtail.structure.spanLocation) 
    fprintf(aswingIn,'\t%3.4f\t%3.4f\t%3.4f\t%3.4f\t%3.4f\t%3.4f\t%3.4f\n', ...
        optim.vtail.structure.spanLocation(i), optim.vtail.structure.chord_m{index}(i), optim.vtail.globalx_m{index}(i) , relfectionY_m*optim.vtail.globaly_m{index}(i), ...
        optim.vtail.z_m + (optim.vtail.globalz_m{index}(i)-optim.vtail.z_m), optim.vtail.structure.twist_deg{index}(i), optim.vtail.structure.Xax{index}(i));
end
 
%specify aero properties
fprintf(aswingIn,'\n');
fprintf(aswingIn,'   t        alpha   Cm      Cdf   Cdp    CLmax    CLmin   dCLda\n');
for i = 1:length(optim.vtail.yAirfoilsobo2) 
            fprintf(aswingIn,'\t%3.4f\t%3.4f\t%3.4f\t%3.4f\t%3.4f\t%3.4f\t%3.4f\t%3.4f\n', -1*optim.vtail.yAirfoilsobo2(length(optim.vtail.yAirfoilsobo2)+1-i),...
            interp1f(aeroDataBase.t, aeroDataBase.alpha', optim.vtail.yAirfoilsobo2(i)) ,  ...
            interp1f(aeroDataBase.t, aeroDataBase.Cm', optim.vtail.yAirfoilsobo2(i)) ,  ...
            interp1f(aeroDataBase.t, aeroDataBase.Cdf', optim.vtail.yAirfoilsobo2(i)) ,  ...
            interp1f(aeroDataBase.t, aeroDataBase.Cdp', optim.vtail.yAirfoilsobo2(i)) ,  ...
            interp1f(aeroDataBase.t, aeroDataBase.CLmax', optim.vtail.yAirfoilsobo2(i)) ,  ...
            interp1f(aeroDataBase.t, aeroDataBase.CLmin', optim.vtail.yAirfoilsobo2(i)) ,  ...
            interp1f(aeroDataBase.t, aeroDataBase.dCLda', optim.vtail.yAirfoilsobo2(i)) );   
end
for i = 1:length(optim.vtail.yAirfoilsobo2) 
            fprintf(aswingIn,'\t%3.4f\t%3.4f\t%3.4f\t%3.4f\t%3.4f\t%3.4f\t%3.4f\t%3.4f\n', optim.vtail.yAirfoilsobo2(i),...
            interp1f(aeroDataBase.t, aeroDataBase.alpha', optim.vtail.yAirfoilsobo2(i)) ,  ...
            interp1f(aeroDataBase.t, aeroDataBase.Cm', optim.vtail.yAirfoilsobo2(i)) ,  ...
            interp1f(aeroDataBase.t, aeroDataBase.Cdf', optim.vtail.yAirfoilsobo2(i)) ,  ...
            interp1f(aeroDataBase.t, aeroDataBase.Cdp', optim.vtail.yAirfoilsobo2(i)) ,  ...
            interp1f(aeroDataBase.t, aeroDataBase.CLmax', optim.vtail.yAirfoilsobo2(i)) ,  ...
            interp1f(aeroDataBase.t, aeroDataBase.CLmin', optim.vtail.yAirfoilsobo2(i)) ,  ...
            interp1f(aeroDataBase.t, aeroDataBase.dCLda', optim.vtail.yAirfoilsobo2(i)) ); 
end
fprintf(aswingIn,'\n');
 
% Vertical tail control surfaces
optim = WriteEmpennageControlSurface(optim, aswingIn, aeroDataBase, 'vtail', relfectionY_m);
 
%vtail structural
%use optim.vtail.structure.spanLocation
% Add vtail section structural properties
if useStructure(3) == 0
    fprintf(aswingIn,'\n');
    fprintf(aswingIn,'      t      mg      mgnn   mgcc     Ccg     Ncg     Cea   Nea     Cta     Nta\n');
     
    for i=1:length(optim.vtail.structure.t{index})
        fprintf(aswingIn,'\t%3.4f\t%3.4f\t%3.4f\t%3.4f\t%3.4f\t%3.4f\t%3.4f\t%3.4f\t%3.4f\t%3.4f\n',...
            -1*optim.vtail.structure.t{index}(l-i),  optim.vtail.structure.mg_N_m{index}(l-i),  optim.vtail.structure.mgnn_Nm{index}(l-i), ...
            optim.vtail.structure.mgcc_Nm{index}(l-i),optim.vtail.structure.Ccg_m{index}(l-i),optim.vtail.structure.Ncg_m{index}(l-i), ...
            optim.vtail.structure.Cea_m{index}(l-i), optim.vtail.structure.Nea_m{index}(l-i), optim.vtail.structure.Cta_m{index}(l-i), ...
            optim.vtail.structure.Nta_m{index}(l-i));
    end
    for i=1:length(optim.vtail.structure.t{index})
        fprintf(aswingIn,'\t%3.4f\t%3.4f\t%3.4f\t%3.4f\t%3.4f\t%3.4f\t%3.4f\t%3.4f\t%3.4f\t%3.4f\n',...
            optim.vtail.structure.t{index}(i),  optim.vtail.structure.mg_N_m{index}(i),  optim.vtail.structure.mgnn_Nm{index}(i), ...
            optim.vtail.structure.mgcc_Nm{index}(i),optim.vtail.structure.Ccg_m{index}(i),optim.vtail.structure.Ncg_m{index}(i), ...
            optim.vtail.structure.Cea_m{index}(i), optim.vtail.structure.Nea_m{index}(i), optim.vtail.structure.Cta_m{index}(i), ...
            optim.vtail.structure.Nta_m{index}(i));
    end
    fprintf(aswingIn,'\n');
     
elseif useStructure(3) == 1 
    fprintf(aswingIn,'\n');
    fprintf(aswingIn,'      t      mg      mgnn   mgcc   EIcc    EInn    GJ        EIcs   EIsn    EIcn    EA\n');
     
    for i=1:length(optim.vtail.structure.t{index})
        fprintf(aswingIn,'\t%3.4f\t%3.4f\t%3.4f\t%3.4f\t%3.4f\t%3.4f\t%3.4f\t%3.4f\t%3.4f\t%3.4f\t%3.4f\n',...
            -1*optim.vtail.structure.t{index}(l-i),  optim.vtail.structure.mg_N_m{index}(l-i),  optim.vtail.structure.mgnn_Nm{index}(l-i), ...
            optim.vtail.structure.mgcc_Nm{index}(l-i), optim.vtail.structure.EIcc_Nm2{index}(l-i), optim.vtail.structure.EInn_Nm2{index}(l-i), ...
            optim.vtail.structure.GJ_Nm2{index}(l-i), optim.vtail.structure.EIcs_Nm2{index}(l-i), optim.vtail.structure.EIsn_Nm2{index}(l-i), ...
            optim.vtail.structure.EIcn_Nm2{index}(l-i), optim.vtail.structure.EA_N{index}(l-i));
    end
    for i=1:length(optim.vtail.structure.t{index})
        fprintf(aswingIn,'\t%3.4f\t%3.4f\t%3.4f\t%3.4f\t%3.4f\t%3.4f\t%3.4f\t%3.4f\t%3.4f\t%3.4f\t%3.4f\n',...
            optim.vtail.structure.t{index}(i),  optim.vtail.structure.mg_N_m{index}(i),  optim.vtail.structure.mgnn_Nm{index}(i), ...
            optim.vtail.structure.mgcc_Nm{index}(i), optim.vtail.structure.EIcc_Nm2{index}(i), optim.vtail.structure.EInn_Nm2{index}(i), ...
            optim.vtail.structure.GJ_Nm2{index}(i), optim.vtail.structure.EIcs_Nm2{index}(i), optim.vtail.structure.EIsn_Nm2{index}(i), ...
            optim.vtail.structure.EIcn_Nm2{index}(i), optim.vtail.structure.EA_N{index}(i));
    end
     
    fprintf(aswingIn,'\n');
    fprintf(aswingIn,'    t       Ccg     Ncg     Cea   Nea     Cta     Nta\n');
     
    for i=1:length(optim.vtail.structure.t{index})
        fprintf(aswingIn,'\t%3.4f\t%3.4f\t%3.4f\t%3.4f\t%3.4f\t%3.4f\t%3.4f\n',...
            -1*optim.vtail.structure.t{index}(l-i), optim.vtail.structure.Ccg_m{index}(l-i),optim.vtail.structure.Ncg_m{index}(l-i), ...
            optim.vtail.structure.Cea_m{index}(l-i), optim.vtail.structure.Nea_m{index}(l-i), optim.vtail.structure.Cta_m{index}(l-i), ...
            optim.vtail.structure.Nta_m{index}(l-i));
    end
    for i=1:length(optim.vtail.structure.t{index})
        fprintf(aswingIn,'\t%3.4f\t%3.4f\t%3.4f\t%3.4f\t%3.4f\t%3.4f\t%3.4f\n',...
            optim.vtail.structure.t{index}(i), optim.vtail.structure.Ccg_m{index}(i),optim.vtail.structure.Ncg_m{index}(i), ...
            optim.vtail.structure.Cea_m{index}(i), optim.vtail.structure.Nea_m{index}(i), optim.vtail.structure.Cta_m{index}(i), ...
            optim.vtail.structure.Nta_m{index}(i));
    end
end
 
fprintf(aswingIn,'End\n');
 
end
