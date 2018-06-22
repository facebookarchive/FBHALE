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
  
function CleanUp (optim)

% Close any files that are still open
fclose ('all');

% Clean up temp files:
if isdeployed
    
delete('*.asw');
delete('*.set');
delete('*.txt');
delete('*.dat');
delete('*.pcf');
delete('*.temp');
delete('*.e00');
delete('xrotorlog');
delete('AQ3.prop');
delete('nul');

else
    
% Clean up ASWing Folder
delete([optim.ASWINGIODir filesep '*.txt'])
delete([optim.ASWINGIODir filesep '*.asw'])
delete([optim.ASWINGIODir filesep '*.e00'])
delete([optim.ASWINGIODir filesep '*.set'])
delete([optim.ASWINGIODir filesep 'Polar' filesep '*.txt'])

% Clean up structures
delete([optim.CoBladeIODir filesep '*.temp']);
cd (optim.BoxShapeDir);

exceptions = {'circle'};
files = [dir('*.dat');dir('*.pcf')];
filenames = {files.name};
for k = 1:numel(filenames)
  [~, name] = fileparts(filenames{k});
  if ~ismember(name,exceptions)
  delete(filenames{k});
  end
end

%test
cd(optim.rootDir);
end