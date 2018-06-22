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
  
function [ optim ] = WriteHtail(  optim, aswingIn, useStructure, reflectionY_m, beamNumber, aeroDataBaseName , entityName, index )
%Writes htail(s) to .asw file.

% Select the appropriate aeroDataBase 
if strcmp(aeroDataBaseName, 'loadsAeroDataBase')
    aeroDataBase = optim.htail.aero.aswingLoadsAirfoilADB;
elseif strcmp(aeroDataBaseName, 'Performace')
    aeroDataBase = optim.htail.aero.aswingPerfAirfoilADB;
elseif strcmp(aeroDataBaseName, 'AeroCoefficients') 
    aeroDataBase = optim.htail.aero.aswingPerfAirfoilADB;
    aeroDataBase.dCLda = zeros(size(aeroDataBase.dCLda))+.01;
else
    error('Aero Database not specified');
end

fprintf(aswingIn,'\n');
fprintf(aswingIn,'#============\n');
fprintf(aswingIn,['Beam ' num2str(beamNumber) '\n']);
fprintf(aswingIn,[entityName '\n']);
fprintf(aswingIn,'#\n');

%htail Aero
fprintf(aswingIn,'\n');
fprintf(aswingIn,'   t        chord   x       y       z       twist     Xax\n');
for i = 1:length(optim.htail.structure.t{index}) 
   fprintf(aswingIn,'\t%3.4f\t%3.4f\t%3.4f\t%3.4f\t%3.4f\t%3.4f\t%3.4f\n', ...
        optim.htail.structure.t{index}(i), optim.htail.structure.chord_m{index}(i), optim.htail.globalx_m{index}(i), reflectionY_m*optim.htail.globaly_m{index}(1)+(optim.htail.globaly_m{index}(i)-optim.htail.globaly_m{index}(1)), ...
        optim.htail.globalz_m{index}(i), optim.htail.structure.twist_deg{index}(i), optim.htail.structure.Xax{index}(i));
end

%specify aero properties
fprintf(aswingIn,'\n');
fprintf(aswingIn,'   t        alpha   Cm      Cdf   Cdp    CLmax    CLmin   dCLda\n');
for i = 1:length(optim.htail.yAirfoilsobo2)        
        fprintf(aswingIn,'\t%3.4f\t%3.4f\t%3.4f\t%3.4f\t%3.4f\t%3.4f\t%3.4f\t%3.4f\n', optim.htail.yAirfoilsobo2(i),...
            interp1f(aeroDataBase.t, aeroDataBase.alpha', optim.htail.yAirfoilsobo2(i)) ,  ...
            interp1f(aeroDataBase.t, aeroDataBase.Cm', optim.htail.yAirfoilsobo2(i)) ,  ...
            interp1f(aeroDataBase.t, aeroDataBase.Cdf', optim.htail.yAirfoilsobo2(i)) ,  ...
            interp1f(aeroDataBase.t, aeroDataBase.Cdp', optim.htail.yAirfoilsobo2(i)) ,  ...
            interp1f(aeroDataBase.t, aeroDataBase.CLmax', optim.htail.yAirfoilsobo2(i)) ,  ...
            interp1f(aeroDataBase.t, aeroDataBase.CLmin', optim.htail.yAirfoilsobo2(i)) ,  ...
            interp1f(aeroDataBase.t, aeroDataBase.dCLda', optim.htail.yAirfoilsobo2(i)));
end

% Horizontal tail control surfaces
optim = WriteEmpennageControlSurface(optim, aswingIn, aeroDataBase, 'htail', reflectionY_m);

% tail strucxtural
% use optim.tail.structure.t
% Add tail section structural properties

fprintf(aswingIn,'\n');
fprintf(aswingIn,'      t      mg      mgnn   mgcc   Ccg     Ncg     Cea   Nea     Cta     Nta\n');
for i=1:length(optim.htail.structure.t{index})
    fprintf(aswingIn,'\t%3.4f\t%3.4f\t%3.4f\t%3.4f\t%3.4f\t%3.4f\t%3.4f\t%3.4f\t%3.4f\t%3.4f\n',...
        optim.htail.structure.t{index}(i),  optim.htail.structure.mg_N_m{index}(i),  optim.htail.structure.mgnn_Nm{index}(i), ...
        optim.htail.structure.mgcc_Nm{index}(i),optim.htail.structure.Ccg_m{index}(i),optim.htail.structure.Ncg_m{index}(i), ...
            optim.htail.structure.Cea_m{index}(i), optim.htail.structure.Nea_m{index}(i), optim.htail.structure.Cta_m{index}(i), ...
            optim.htail.structure.Nta_m{index}(i));
end
fprintf(aswingIn,'\n');

% Print structal properties if requested    
if useStructure(2) == 1
    fprintf(aswingIn,'\n');
    fprintf(aswingIn,'  t      mg      mgnn   mgcc    EIcc    EInn    GJ        EIcs   EIsn    EIcn    EA\n');
    for i=1:length(optim.htail.structure.t{index})
        fprintf(aswingIn,'\t%3.4f\t%3.4f\t%3.4f\t%3.4f\t%3.4f\t%3.4f\t%3.4f\t%3.4f\t%3.4f\t%3.4f\t%3.4f\n',...
            optim.htail.structure.t{index}(i),  optim.htail.structure.mg_N_m{index}(i),  optim.htail.structure.mgnn_Nm{index}(i), ...
            optim.htail.structure.mgcc_Nm{index}(i), optim.htail.structure.EIcc_Nm2{index}(i), optim.htail.structure.EInn_Nm2{index}(i), ...
            optim.htail.structure.GJ_Nm2{index}(i), optim.htail.structure.EIcs_Nm2{index}(i), optim.htail.structure.EIsn_Nm2{index}(i), ...
            optim.htail.structure.EIcn_Nm2{index}(i), optim.htail.structure.EA_N{index}(i));
    end
    
    fprintf(aswingIn,'\n');
    fprintf(aswingIn,'    t       Ccg     Ncg     Cea   Nea     Cta     Nta\n');
    for i=1:length(optim.htail.structure.t{index})
        fprintf(aswingIn,'\t%3.4f\t%3.4f\t%3.4f\t%3.4f\t%3.4f\t%3.4f\t%3.4f\n',...
            optim.htail.structure.t{index}(i), optim.htail.structure.Ccg_m{index}(i),optim.htail.structure.Ncg_m{index}(i), ...
            optim.htail.structure.Cea_m{index}(i), optim.htail.structure.Nea_m{index}(i), optim.htail.structure.Cta_m{index}(i), ...
            optim.htail.structure.Nta_m{index}(i));
    end
end

fprintf(aswingIn,'End\n');

end

