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

% This function specifies configuration specific joint location sand ground
% beam specification for flying wing aircraft configuration.

function [] = JointSpecification(optim, aswingIn)

% Pods to Wing
fprintf(aswingIn,'\t%3.4f\t%3.4f\t%3.4f\t%3.4f\n',    1,   2,  0/(optim.wing.bref_m/2), ...
     ((optim.wing.globalx_m{1}(1)+optim.wing.structure.Cea_m{1}(1))-optim.pod.globalx_m{1}(1))/max(optim.pod.omlSectionsX_m{1}));
    fprintf(aswingIn,'\t%3.4f\t%3.4f\t%3.4f\t%3.4f\n',    1,   3, ...
        (optim.batteries.y_m(2)/(optim.wing.bref_m/2)), .5 );
    fprintf(aswingIn,'\t%3.4f\t%3.4f\t%3.4f\t%3.4f\n',    1,   4, ...
        (-1*optim.batteries.y_m(2)/(optim.wing.bref_m/2)), .5 );
 fprintf(aswingIn,'End\n');
% Specify ground location
fprintf(aswingIn,'#============\n');
fprintf(aswingIn,'Ground\n');
fprintf(aswingIn,'#  Nbeam  t     Kground\n');
fprintf(aswingIn,'    1      0.0   0\n');
    
end

