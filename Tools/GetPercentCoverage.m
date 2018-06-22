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
  
function [ percentPass ] = GetPercentCoverage(latBand, V_ms, x50milibar)
% finds the percent of stations that fall within latitude and velocity
% input
%Oputput: percentPass(scaler): percent of stations that fall witin
%specified requirements
passLat = x50milibar(abs(x50milibar(:,2))<=latBand,:);
passV   = passLat(abs(passLat(:,3))<=V_ms,:);

percentPass = length(passV)/length(x50milibar);

end

