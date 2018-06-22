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

function isReady = CheckSystem()
% This function checks that the current platform is ready to run the MDO
% sizing framework.
if contains(computer, 'MAC')
    isMac = true;
else
    isMac = false;
end

% Check for aero-codes
codes = {'aswing', 'xfoil', 'xrotor'};
isCodeInPath = true * ones(length(codes), 1);

for c = 1:length(codes)
   
    commandOutput = system([codes{c} ' << quit > ' GetNullDevice() ' 2>&1']);
    if commandOutput == 127
        isCodeInPath(c) = false;
        disp([codes{c} ' is not in your path...']);
    end        
    
end

% If a code was not in the path, offer solution
[~, missingCode] = min(isCodeInPath);
if min(isCodeInPath) == 0
    isReady = false;
    disp(' ');
    if isMac
        disp([ 'To add the missing code to your path, use the ' ...
            '''getenv'' & ''setenv'' functions e.g. ']);
        disp(['>> PATH = getenv(''PATH'');']);
        disp(['>> setenv(''PATH'', [PATH '':~/Documents/AeroCodes/' codes{missingCode} '/bin'']);']);
       
    else
        disp(['To add the missing code to your path add the corresponding' ...
            ' folder to your system environment variables']);
    end
else
    isReady = true;
end