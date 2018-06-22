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
  
function optim = CoBlade(optim, entity)
% The script takes in an input structure describing beam properties (both
% global and distributed) and uses coblade to generate distributed stiffnesses
% and masses in ASWING format.


for idx = 1:length(optim.(entity).N)
% Perform interpolation (for most recent twists or chords):
optim = UpdateGeometry (optim, entity, idx);

% Setup Coblade:
[optim SIM ANLS OPT ENV BLADE WEB OUT MATS AF Coord] = ...
    SetupCoBlade (optim, entity, idx);

% Execute Coblade:
optim.(entity).structure = ...
    ExecCoBlade(optim.(entity).structure, idx, SIM, ANLS, OPT, ENV, BLADE, WEB, AF, MATS, Coord);
end

