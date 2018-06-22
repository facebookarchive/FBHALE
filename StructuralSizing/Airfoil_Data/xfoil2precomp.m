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
  
% Convert xfoil airfoil coordinate data files (selig) to precomp format.
% Make sure trailing edge is at (1,0) and leading-edge is at (0,0). 

function  xfoil2precomp (airfoilFile)

% load data:
data = readtable(airfoilFile);
[~,name] = fileparts(airfoilFile);
x = data.Var1; y = data.Var2;

% determine LE and split coordinates:
[~,iLE] = min(x);
surf1 = sortrows([x(1:iLE),y(1:iLE)]);
surf2 = sortrows([x(iLE:end),y(iLE:end)]);
data = unique([surf1;flipud(surf2)],'rows','stable');
data(1,:) = [0,0];

x = data(:,1); y = data(:,2);
[x, idx] = unique(x,'rows','stable');
y = y(idx);
x(end) = 0; y(end) = 0;

[~,iTE] = max(x);
xinp = linspace(0,1,100);

% split into top and bottom surfaces:
y1 = interp1(x(1:iTE),y(1:iTE),xinp);
y2 = interp1(x(iTE:end),y(iTE:end),xinp);

% calculate spar heights, wing box area etc.:
h = abs(y1-y2);
polyOrder = 9;
p = polyfit(xinp(5:end-5), h(5:end-5), polyOrder);

% calculate non-dimensional geometrical parameters for structural sizing 
fid = fopen(sprintf('%s.pcf',name), 'w');
fprintf(fid,'%d\n',length(data));
 fprintf(fid,'%f %f %f %f %f %f %f %f %f %f\n\n\n',p');
for i=1:length(data)
fprintf(fid,'%f %f\n',data(i,:));
end
fclose(fid);
