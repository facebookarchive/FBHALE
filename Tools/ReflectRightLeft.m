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
  
function [ output ] = RelectRightLeft(input, flipsign )
% This function reflects the input vector or matrix and orders the
% outputmatrix [orig1 relect1 orig2 reflect2]. If flip sign is true then
% the reflection is multiplied by -1.

if flipsign == 1
    reflection = -1.*input;
else
    reflection = input;
end

output(:,1:2:length(input)*2) = input;
output(:,2:2:length(input)*2) = reflection;


end

