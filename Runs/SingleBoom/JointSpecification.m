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
% beam specification for the Single boom aircraft configuration.

function [] = JointSpecification(optim, aswingIn)

% Boom to Wing
fprintf(aswingIn,'\t%3.4f\t%3.4f\t%3.4f\t%3.4f\n',   1,  4, ...
    optim.boom.y_m/(optim.wing.bref_m/2), ...
    ((optim.wing.globalx_m{1}(1)+optim.wing.structure.Cea_m{1}(1))-optim.boom.globalx_m{1}(1))/max(optim.boom.omlSectionsX_m{1}));

% Boom to htail and boom to vtail
fprintf(aswingIn,'\t%3.4f\t%3.4f\t%3.4f\t%3.4f\n',    4,   2,    (optim.htail.globalx_m{1}(1)+optim.htail.structure.Cea_m{1}(1))/max(optim.boom.omlSectionsX_m{1}),            0);
    fprintf(aswingIn,'\t%3.4f\t%3.4f\t%3.4f\t%3.4f\n',    4,   3,    1,            -1*(1-((optim.vtail.bref_m/2)-optim.vtail.z_m)/(optim.vtail.bref_m/2)));

% Pods to wing
fprintf(aswingIn,'\t%3.4f\t%3.4f\t%3.4f\t%3.4f\n',    1,   5,    optim.batteries.y_m(2)/(optim.wing.bref_m/2),            .55);
    fprintf(aswingIn,'\t%3.4f\t%3.4f\t%3.4f\t%3.4f\n',    1,   6,    -1*optim.batteries.y_m(2)/(optim.wing.bref_m/2),            .55);

fprintf(aswingIn,'End\n');

% Specify ground location
fprintf(aswingIn,'#============\n');
fprintf(aswingIn,'Ground\n');
fprintf(aswingIn,'#  Nbeam  t     Kground\n');
fprintf(aswingIn,'    1      0.0   0\n');
    
end