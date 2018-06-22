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
  
function [ optim ] = WriteEmpennageControlSurface(optim, aswingIn, aeroDataBase, entity, reflectionY_m)
%This function writes empennage control surfaces to the .asw file. It uses
%only the tmax of the control surface and assumes the surface spans -tmax
%to tmax and both the negative and positive t sides of the surface move together.

if isfield(optim.(entity),'controlSurface')
    nControlSurfaces = length(optim.(entity).controlSurface);
else
    nControlSurfaces = 0;
end

for i = 1:nControlSurfaces
    tMax = max(optim.(entity).controlSurface(i).yobo2);
    controlSurfaceMapping = find(optim.(entity).controlSurface(i).schedule);
    for n = 1:length(controlSurfaceMapping)
        if controlSurfaceMapping(n) == 1 && reflectionY_m == -1 % check if surface is used for roll
            deflectionDirection = -1;
        else
            deflectionDirection = 1;
        end
        
        %Write Header
        fprintf(aswingIn,'\n');
        fprintf(aswingIn,['  t         dCLdF' num2str(controlSurfaceMapping(n)) ...
            '  dCDdF' num2str(controlSurfaceMapping(n)) '  dCMdF' num2str(controlSurfaceMapping(n)) '\n']);
        fprintf(aswingIn,['* 1 ' num2str(pi/180) ' ' num2str(pi/180) ' ' num2str(pi/180) '\n']); % convert radians to degrees
        
        % If surface is is full span
        if tMax == 1
            
            fprintf(aswingIn,'\t%3.4f\t%3.4f\t%3.4f\t%3.4f\n', -1*tMax,...
                deflectionDirection * interp1f(aeroDataBase.t, aeroDataBase.dCLdF{i}, tMax), ...
                interp1f(aeroDataBase.t, aeroDataBase.dCDdF{i}, tMax), ...
                deflectionDirection * interp1f(aeroDataBase.t, aeroDataBase.dCMdF{i}, tMax ));
            fprintf(aswingIn,'\t%3.4f\t%3.4f\t%3.4f\t%3.4f\n', tMax,...
                deflectionDirection * interp1f(aeroDataBase.t, aeroDataBase.dCLdF{i}, tMax), ...
                interp1f(aeroDataBase.t, aeroDataBase.dCDdF{i}, tMax), ...
                deflectionDirection * interp1f(aeroDataBase.t, aeroDataBase.dCMdF{i}, tMax ));
            
        else % if elevator is less than full span enforce discontinuity
            
            if length(aeroDataBase.t)>1
                fprintf(aswingIn,'\t%3.4f\t%3.4f\t%3.4f\t%3.4f\n', -1, 0 , 0, 0);
                fprintf(aswingIn,'\t%3.4f\t%3.4f\t%3.4f\t%3.4f\n', -1*tMax, 0 , 0, 0);
                fprintf(aswingIn,'\t%3.4f\t%3.4f\t%3.4f\t%3.4f\n', -1*tMax,...
                    deflectionDirection * interp1f(aeroDataBase.t, aeroDataBase.dCLdF{i}, tMax), ...
                    interp1f(aeroDataBase.t, aeroDataBase.dCDdF{i}, tMax), ...
                    deflectionDirection * interp1f(aeroDataBase.t, aeroDataBase.dCMdF{i}, tMax ));
                fprintf(aswingIn,'\t%3.4f\t%3.4f\t%3.4f\t%3.4f\n', tMax,...
                    deflectionDirection * interp1f(aeroDataBase.t, aeroDataBase.dCLdF{i}, tMax), ...
                    interp1f(aeroDataBase.t, aeroDataBase.dCDdF{i}, tMax), ...
                    deflectionDirection * interp1f(aeroDataBase.t, aeroDataBase.dCMdF{i}, tMax ));
                fprintf(aswingIn,'\t%3.4f\t%3.4f\t%3.4f\t%3.4f\n', tMax, 0 , 0, 0);
                fprintf(aswingIn,'\t%3.4f\t%3.4f\t%3.4f\t%3.4f\n', 1, 0 , 0, 0);
            else
                fprintf(aswingIn,'\t%3.4f\t%3.4f\t%3.4f\t%3.4f\n', -1, 0 , 0, 0);
                fprintf(aswingIn,'\t%3.4f\t%3.4f\t%3.4f\t%3.4f\n', -1*tMax, 0 , 0, 0);
                fprintf(aswingIn,'\t%3.4f\t%3.4f\t%3.4f\t%3.4f\n', -1*tMax,...
                    deflectionDirection * aeroDataBase.dCLdF{i}, ...
                    aeroDataBase.dCDdF{i},  ...
                    deflectionDirection * aeroDataBase.dCMdF{i} );
                fprintf(aswingIn,'\t%3.4f\t%3.4f\t%3.4f\t%3.4f\n', tMax,...
                    deflectionDirection * aeroDataBase.dCLdF{i}, ...
                    aeroDataBase.dCDdF{i}, ...
                    deflectionDirection * aeroDataBase.dCMdF{i} );
                fprintf(aswingIn,'\t%3.4f\t%3.4f\t%3.4f\t%3.4f\n', tMax, 0 , 0, 0);
                fprintf(aswingIn,'\t%3.4f\t%3.4f\t%3.4f\t%3.4f\n', 1, 0 , 0, 0);
            end
        end
    end
end
end

