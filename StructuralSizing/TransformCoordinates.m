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
  

function [x_m,y_m,z_m] = TransformCoordinates (x_m,y_m,z_m,dx_m,dy_m,dz_m,thetax_deg,thetaz_deg)
% Transforms coordinates from local frames to global using translation and
% rotation. 
% Inputs: 
% dx_m: Location of the quarter-chord of the lifting surface from origin. 
% thetax_deg: rotation of the beam axis about X in deg. For horizontal surfaces
% use thetax = 0, for vertical use thetax = 90. 
% Outputs:
% Coordinates in global frame.

% Eg: To transform vertical tail local coordinates to global use:
% [x,y,z] = transformCoordinates(optim.wing.structure.sRefX_m,...
%     optim.wing.structure.sRefY_m, optim.wing.structure.sRefZ_m, dx, 90,0)

% Eg: To transform boom local coordinates to global use:
% [x,y,z] = transformCoordinates(optim.wing.structure.sRefX_m,...
%     optim.wing.structure.sRefY_m, optim.wing.structure.sRefZ_m, dx, 0,-90)

   vec = [x_m(:)';y_m(:)';z_m(:)'];
   
  % RotationX:
  rotx = [1 0 0;0 cos(thetax_deg/180*pi) -sin(thetax_deg/180*pi);...
      0 sin(thetax_deg/180*pi) cos(thetax_deg/180*pi)];

  % RotationZ:
  rotz = [cos(thetaz_deg/180*pi) -sin(thetaz_deg/180*pi) 0;...
      sin(thetaz_deg/180*pi) cos(thetaz_deg/180*pi) 0;...
      0 0 1];
  
  vec = rotz*rotx*vec;
  
  % Translation:
  vec(1,:) = vec(1,:) + dx_m;
  vec(2,:) = vec(2,:) + dy_m;
  vec(3,:) = vec(3,:) + dz_m;
  
  x_m = vec(1,:);
  y_m = vec(2,:);
  z_m = vec(3,:);
  
  
  
  
  
  
