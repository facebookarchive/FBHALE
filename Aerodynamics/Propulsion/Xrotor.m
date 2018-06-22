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
  
function Xrotor(commandSequence)
% This routine automates the command prompt input sequence in XROTOR.

% Check system archictecture win vs unix
isWin = contains(computer, 'WIN');

if ~isWin
    % Write input file
    inputFilePath = 'xrotorInput.sh';
    fid = fopen([pwd '/' inputFilePath], 'w');
    fprintf(fid, 'xrotor << EOF \n');
    fprintf(fid, commandSequence);
    fprintf(fid, 'EOF');
    fclose(fid);

    % Execute xrotor
    % Move to the folder where the prop is and where the input was generated
    system(['sh xrotorInput.sh > xrotorlog ' ' 2>&1']);
    delete('xrotorInput.sh');
else
	% Write input file
    inputFilePath = 'xrotorInput.inp';
    fid = fopen([pwd '/' inputFilePath], 'w');
    fprintf(fid, commandSequence);
    fclose(fid);

	% Execute xrotor
    % Move to the folder where the prop is and where the input was generated
    system('xrotor < xrotorInput.inp > xrotorlog 2>&1');
    delete('xrotorInput.inp');
end

end
