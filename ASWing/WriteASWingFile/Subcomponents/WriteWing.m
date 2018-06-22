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
  
function [ optim] = WriteWing( optim, aswingIn, useStructure, aeroDataBaseName, beamNumber, entityName, index )
% writes wing to .asw file.
%% Select the appropriate aeroDataBase
if strcmp(aeroDataBaseName, 'loadsAeroDataBase')
    aeroDataBase = optim.wing.aero.aswingLoadsAirfoilADB;
elseif strcmp(aeroDataBaseName, 'Performace')
    aeroDataBase = optim.wing.aero.aswingPerfAirfoilADB;
elseif strcmp(aeroDataBaseName, 'AeroCoefficients')
    aeroDataBase = optim.wing.aero.aswingPerfAirfoilADB;
    
    % Use CL limits (they are removed from the performance db)
    aeroDataBase.CLmax = optim.wing.aero.aswingLoadsAirfoilADB.CLmax;
    aeroDataBase.CLmin = optim.wing.aero.aswingLoadsAirfoilADB.CLmin;
else
    error('Aero Database not specified');
end

%% Wing 
% Write Header
fprintf(aswingIn,'#============\n');
fprintf(aswingIn,['Beam ' num2str(beamNumber) '\n']);
fprintf(aswingIn,[entityName '\n']);
fprintf(aswingIn,'#\n');

% Wing Aero
fprintf(aswingIn,'\n');
fprintf(aswingIn,'   t    x       y       z     Xax\n');

% Check for break point if so find properties at break point else just
% print properties from database
for i = 1:length(optim.wing.structure.t{index})
    if abs(optim.wing.structure.t{index}(i)- optim.wing.yobo2(2))<1e-6
        t_discontinuity = optim.wing.yobo2(2);
        for p = 1:2
            fprintf(aswingIn,'\t%3.4f\t%3.4f\t%3.4f\t%3.4f\t%3.4f\n', ...
               t_discontinuity, optim.wing.globalx_m{index}(i), optim.wing.globaly_m{index}(i), optim.wing.globalz_m{index}(i),  ... 
                optim.wing.structure.Xax{index}(i));
        end
    elseif i<length(optim.wing.structure.t{index}) && optim.wing.globalz_m{index}(i+1)~=optim.wing.globalz_m{index}(i)
            for p = 1:2
                fprintf(aswingIn,'\t%3.4f\t%3.4f\t%3.4f\t%3.4f\t%3.4f\n', ...
                    optim.wing.structure.t{index}(i),  optim.wing.globalx_m{index}(i) , optim.wing.globaly_m{index}(i), ...
                    optim.wing.globalz_m{index}(i), optim.wing.structure.Xax{index}(i));
            end  
    else
        fprintf(aswingIn,'\t%3.4f\t%3.4f\t%3.4f\t%3.4f\t%3.4f\n', ...
            optim.wing.structure.t{index}(i),  optim.wing.globalx_m{index}(i) , optim.wing.globaly_m{index}(i), ...
            optim.wing.globalz_m{index}(i), optim.wing.structure.Xax{index}(i));
    end

end

%specify aero properties
fprintf(aswingIn,'\n');
fprintf(aswingIn,'   t      chord      twist    alpha   Cm      Cdf    Cdp   CLmax    CLmin   dCLda\n');
aeroSectionsHalfSpanT = optim.wing.aero.sectionPolars.tAirfoils(optim.wing.aero.sectionPolars.tAirfoils>=0);
aeroSectionsZ_m       = interp1(optim.wing.aero.sectionPolars.t, optim.wing.aero.sectionPolars.z_m, aeroSectionsHalfSpanT);
for i = 1:length(aeroSectionsHalfSpanT)
    % Enforce discontinuity at break points
    if ismember(aeroSectionsHalfSpanT(i), optim.wing.yobo2(2:end-1))
        for p = 1:2
            fprintf(aswingIn,'\t%3.4f\t%3.4f\t%3.4f\t%3.4f\t%3.4f\t%3.4f\t%3.4f\t%3.4f\t%3.4f\t%3.4f\n', aeroSectionsHalfSpanT(i),...
                interp1f(optim.wing.aero.sectionPolars.t, optim.wing.aero.sectionPolars.c_m, aeroSectionsHalfSpanT(i)),...
                interp1f(optim.wing.aero.sectionPolars.t, optim.wing.aero.sectionPolars.twists_deg, aeroSectionsHalfSpanT(i)), ...
                interp1f(aeroDataBase.t, aeroDataBase.alpha, aeroSectionsHalfSpanT(i)), ...
                interp1f(aeroDataBase.t, aeroDataBase.Cm, aeroSectionsHalfSpanT(i)), ...
                interp1f(aeroDataBase.t, aeroDataBase.Cdf, aeroSectionsHalfSpanT(i)), ...
                interp1f(aeroDataBase.t, aeroDataBase.Cdp, aeroSectionsHalfSpanT(i)), ...
                interp1f(aeroDataBase.t, aeroDataBase.CLmax, aeroSectionsHalfSpanT(i)), ...
                interp1f(aeroDataBase.t, aeroDataBase.CLmin, aeroSectionsHalfSpanT(i)), ...
                interp1f(aeroDataBase.t, aeroDataBase.dCLda, aeroSectionsHalfSpanT(i)));
        end
        
    elseif i>1 && (abs(aeroSectionsZ_m(i-1)-aeroSectionsZ_m(i))>1e8*eps)    
        for p = 1:2
            fprintf(aswingIn,'\t%3.4f\t%3.4f\t%3.4f\t%3.4f\t%3.4f\t%3.4f\t%3.4f\t%3.4f\t%3.4f\t%3.4f\n', aeroSectionsHalfSpanT(i-2+p),...
                interp1f(optim.wing.aero.sectionPolars.t, optim.wing.aero.sectionPolars.c_m, aeroSectionsHalfSpanT(i)),...
                interp1f(optim.wing.aero.sectionPolars.t, optim.wing.aero.sectionPolars.twists_deg, aeroSectionsHalfSpanT(i)), ...
                interp1f(aeroDataBase.t, aeroDataBase.alpha, aeroSectionsHalfSpanT(i)), ...
                interp1f(aeroDataBase.t, aeroDataBase.Cm, aeroSectionsHalfSpanT(i)), ...
                interp1f(aeroDataBase.t, aeroDataBase.Cdf, aeroSectionsHalfSpanT(i)), ...
                interp1f(aeroDataBase.t, aeroDataBase.Cdp, aeroSectionsHalfSpanT(i)), ...
                interp1f(aeroDataBase.t, aeroDataBase.CLmax, aeroSectionsHalfSpanT(i)), ...
                interp1f(aeroDataBase.t, aeroDataBase.CLmin, aeroSectionsHalfSpanT(i)), ...
                interp1f(aeroDataBase.t, aeroDataBase.dCLda, aeroSectionsHalfSpanT(i)));
        end
    else
        fprintf(aswingIn,'\t%3.4f\t%3.4f\t%3.4f\t%3.4f\t%3.4f\t%3.4f\t%3.4f\t%3.4f\t%3.4f\t%3.4f\n', aeroSectionsHalfSpanT(i),...
            interp1f(optim.wing.aero.sectionPolars.t, optim.wing.aero.sectionPolars.c_m, aeroSectionsHalfSpanT(i)),...
            interp1f(optim.wing.aero.sectionPolars.t, optim.wing.aero.sectionPolars.twists_deg, aeroSectionsHalfSpanT(i)), ...
            interp1f(aeroDataBase.t, aeroDataBase.alpha', aeroSectionsHalfSpanT(i)), ...
            interp1f(aeroDataBase.t, aeroDataBase.Cm', aeroSectionsHalfSpanT(i)), ...
            interp1f(aeroDataBase.t, aeroDataBase.Cdf', aeroSectionsHalfSpanT(i)), ...
            interp1f(aeroDataBase.t, aeroDataBase.Cdp', aeroSectionsHalfSpanT(i)), ...
            interp1f(aeroDataBase.t, aeroDataBase.CLmax', aeroSectionsHalfSpanT(i)), ...
            interp1f(aeroDataBase.t, aeroDataBase.CLmin', aeroSectionsHalfSpanT(i)), ...
            interp1f(aeroDataBase.t, aeroDataBase.dCLda', aeroSectionsHalfSpanT(i)));
    end
end

% Loop through all control surfaces on wing and write to file
if isfield(optim.wing,'controlSurface')
    nControlSurfaces = length(optim.wing.controlSurface);
else
    nControlSurfaces = 0;
end

for i = 1:nControlSurfaces
    tMax = max(optim.wing.controlSurface(i).yobo2);
    tMin = min(optim.wing.controlSurface(i).yobo2);
    
    controlSurfaceMapping = find(optim.wing.controlSurface(i).schedule);
    for n = 1:length(controlSurfaceMapping)
        % If the surface is used for roll set negativeSurfaceFactor to -1
        % so the positiive and negative surfacees move oppostie directions
        if controlSurfaceMapping(n) == 1
            negativeSurcafeFactor = -1;
            gearRatio = optim.wing.controlSurface(i).gearRatio;
        else
            negativeSurcafeFactor = 1;
            gearRatio = 1;
        end
        
        fprintf(aswingIn,'\n');
        fprintf(aswingIn,['  t         dCLdF' num2str(controlSurfaceMapping(n)) '  dCDdF' num2str(controlSurfaceMapping(n)) '  dCMdF' num2str(controlSurfaceMapping(n)) '\n']);
        fprintf(aswingIn,['* 1 ' num2str(pi/180) ' ' num2str(pi/180) ' ' num2str(pi/180) '\n']);
        if length(aeroDataBase.dCLdF{i})>1
                   fprintf(aswingIn,'\t%3.4f\t%3.4f\t%3.4f\t%3.4f\n', -1*max(aeroSectionsHalfSpanT), 0 , 0, 0);
            fprintf(aswingIn,'\t%3.4f\t%3.4f\t%3.4f\t%3.4f\n', -1*tMax, 0 , 0, 0);
            fprintf(aswingIn,'\t%3.4f\t%3.4f\t%3.4f\t%3.4f\n', -1*tMax,...
                negativeSurcafeFactor*interp1f(aeroDataBase.t, aeroDataBase.dCLdF{i}, tMax), ...
                interp1f(aeroDataBase.t, aeroDataBase.dCDdF{i}, tMax), ...
                negativeSurcafeFactor*interp1f(aeroDataBase.t, aeroDataBase.dCMdF{i}, tMax));
            fprintf(aswingIn,'\t%3.4f\t%3.4f\t%3.4f\t%3.4f\n', -1*tMin,...
                negativeSurcafeFactor*interp1f(aeroDataBase.t, aeroDataBase.dCLdF{i}, tMin), ...
                interp1f(aeroDataBase.t, aeroDataBase.dCDdF{i}, tMin), ...
                negativeSurcafeFactor*interp1f(aeroDataBase.t, aeroDataBase.dCMdF{i}, tMin));
            fprintf(aswingIn,'\t%3.4f\t%3.4f\t%3.4f\t%3.4f\n', -1*tMin, 0 , 0, 0);
            fprintf(aswingIn,'\t%3.4f\t%3.4f\t%3.4f\t%3.4f\n', tMin, 0 , 0, 0);
            fprintf(aswingIn,'\t%3.4f\t%3.4f\t%3.4f\t%3.4f\n', tMin,...
                1/gearRatio*interp1f(aeroDataBase.t, aeroDataBase.dCLdF{i}, tMin), ...
                1/gearRatio*interp1f(aeroDataBase.t, aeroDataBase.dCDdF{i}, tMin), ...
                1/gearRatio*interp1f(aeroDataBase.t, aeroDataBase.dCMdF{i}, tMin));
            fprintf(aswingIn,'\t%3.4f\t%3.4f\t%3.4f\t%3.4f\n', tMax,...
                1/gearRatio*interp1f(aeroDataBase.t, aeroDataBase.dCLdF{i}, tMax), ...
                1/gearRatio*interp1f(aeroDataBase.t, aeroDataBase.dCDdF{i}, tMax), ...
                1/gearRatio*interp1f(aeroDataBase.t, aeroDataBase.dCMdF{i}, tMax));
            fprintf(aswingIn,'\t%3.4f\t%3.4f\t%3.4f\t%3.4f\n', tMax, 0 , 0, 0);
            fprintf(aswingIn,'\t%3.4f\t%3.4f\t%3.4f\t%3.4f\n', max(aeroSectionsHalfSpanT), 0 , 0, 0);
            
        else
            fprintf(aswingIn,'\t%3.4f\t%3.4f\t%3.4f\t%3.4f\n', -1*max(aeroSectionsHalfSpanT), 0 , 0, 0);
            fprintf(aswingIn,'\t%3.4f\t%3.4f\t%3.4f\t%3.4f\n', -1*tMax, 0 , 0, 0);
            fprintf(aswingIn,'\t%3.4f\t%3.4f\t%3.4f\t%3.4f\n', -1*tMax,...
                negativeSurcafeFactor*aeroDataBase.dCLdF{i},  ...
                aeroDataBase.dCDdF{i},     ...
                negativeSurcafeFactor*aeroDataBase.dCMdF{i});
            fprintf(aswingIn,'\t%3.4f\t%3.4f\t%3.4f\t%3.4f\n', -1*tMin,...
                negativeSurcafeFactor*aeroDataBase.dCLdF{i}, ...
                aeroDataBase.dCDdF{i}, ...
                negativeSurcafeFactor*aeroDataBase.dCMdF{i});
            fprintf(aswingIn,'\t%3.4f\t%3.4f\t%3.4f\t%3.4f\n', -1*tMin, 0 , 0, 0);
            fprintf(aswingIn,'\t%3.4f\t%3.4f\t%3.4f\t%3.4f\n', tMin, 0 , 0, 0);
            fprintf(aswingIn,'\t%3.4f\t%3.4f\t%3.4f\t%3.4f\n', tMin,...
                1/gearRatio*aeroDataBase.dCLdF{i}, ...
                1/gearRatio*aeroDataBase.dCDdF{i}, ...
                1/gearRatio*aeroDataBase.dCMdF{i});
            fprintf(aswingIn,'\t%3.4f\t%3.4f\t%3.4f\t%3.4f\n', tMax,...
                1/gearRatio*aeroDataBase.dCLdF{i}, ...
                1/gearRatio*aeroDataBase.dCDdF{i}, ...
                1/gearRatio*aeroDataBase.dCMdF{i});
            fprintf(aswingIn,'\t%3.4f\t%3.4f\t%3.4f\t%3.4f\n', tMax, 0 , 0, 0);
            fprintf(aswingIn,'\t%3.4f\t%3.4f\t%3.4f\t%3.4f\n', max(aeroSectionsHalfSpanT), 0 , 0, 0);
        end
    end
end

%Wing structural
%use optim.wing.structure.spanLocation
% Add wing section structural properties

fprintf(aswingIn,'\n');
fprintf(aswingIn,'      t      mg      mgnn   mgcc   Ccg     Ncg     Cea   Nea     Cta     Nta\n');
for i=1:length(optim.wing.structure.t{index})
    fprintf(aswingIn,'\t%3.4f\t%3.4f\t%3.4f\t%3.4f\t%3.4f\t%3.4f\t%3.4f\t%3.4f\t%3.4f\t%3.4f\n',...
        optim.wing.structure.t{index}(i),  optim.wing.structure.mg_N_m{index}(i),  optim.wing.structure.mgnn_Nm{index}(i), ...
        optim.wing.structure.mgcc_Nm{index}(i),optim.wing.structure.Ccg_m{index}(i),optim.wing.structure.Ncg_m{index}(i), ...
        optim.wing.structure.Cea_m{index}(i), optim.wing.structure.Nea_m{index}(i), optim.wing.structure.Cta_m{index}(i), ...
        optim.wing.structure.Nta_m{index}(i));
end
fprintf(aswingIn,'\n');

if useStructure(1) == 1
    
    fprintf(aswingIn,'      t      mg      mgnn   mgcc   EIcc    EInn    GJ        EIcs   EIsn    EIcn    EA\n');
    for i=1:length(optim.wing.structure.t{index})
        fprintf(aswingIn,'\t%3.4f\t%3.4f\t%3.4f\t%3.4f\t%3.4f\t%3.4f\t%3.4f\t%3.4f\t%3.4f\t%3.4f\t%3.4f\n',...
            optim.wing.structure.t{index}(i),  optim.wing.structure.mg_N_m{index}(i),  optim.wing.structure.mgnn_Nm{index}(i), ...
            optim.wing.structure.mgcc_Nm{index}(i), optim.wing.structure.EIcc_Nm2{index}(i), optim.wing.structure.EInn_Nm2{index}(i), ...
            optim.wing.structure.GJ_Nm2{index}(i), optim.wing.structure.EIcs_Nm2{index}(i), optim.wing.structure.EIsn_Nm2{index}(i), ...
            optim.wing.structure.EIcn_Nm2{index}(i), optim.wing.structure.EA_N{index}(i));
    end
    
    fprintf(aswingIn,'\n');
    fprintf(aswingIn,'    t       Ccg     Ncg     Cea   Nea     Cta     Nta\n');
    for i=1:length(optim.wing.structure.t{index})
        fprintf(aswingIn,'\t%3.4f\t%3.4f\t%3.4f\t%3.4f\t%3.4f\t%3.4f\t%3.4f\n',...
            optim.wing.structure.t{index}(i), optim.wing.structure.Ccg_m{index}(i),optim.wing.structure.Ncg_m{index}(i), ...
            optim.wing.structure.Cea_m{index}(i), optim.wing.structure.Nea_m{index}(i), optim.wing.structure.Cta_m{index}(i), ...
            optim.wing.structure.Nta_m{index}(i));
    end
    fprintf(aswingIn,'\n');
end

fprintf(aswingIn,'End\n');

end

