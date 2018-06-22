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

Modified from the original coblade repository. License for the original 
file appears below:

Copyright (c) 2012, Danny Sale 
Copyright (c) 2016, H.J. Sommer 
Copyright (c) 2009, John D'Errico 
Copyright (c) 2017, Yair Altman 
Copyright (c) 2016, S. Samuel Chen 
All rights reserved.

Redistribution and use in source and binary forms, with or without 
modification, are permitted provided that the following conditions are 
met:

* Redistributions of source code must retain the above copyright 
notice, this list of conditions and the following disclaimer. 
* Redistributions in binary form must reproduce the above copyright 
notice, this list of conditions and the following disclaimer in 
the documentation and/or other materials provided with the distribution 
* Neither the name of the Penn State University nor the names 
of its contributors may be used to endorse or promote products derived 
from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" 
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE 
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE 
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE 
LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR 
CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF 
SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS 
INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN 
CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) 
ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE 
POSSIBILITY OF SUCH DAMAGE.
%}
  
function [ANLS OPT ENV BLADE WEB OUT MATS AF Coord] = CoBlade_init(SIM)

%% read the main input file
fid = fopen(SIM.inpFile{SIM.iSIM}, 'r');
if fid == -1
    error(['ERROR: Could not locate and open file ' [SIM.rootDir filesep SIM.inpFile{SIM.iSIM}]]);
end

% Skip through the header of the file
for i = 1:3
    fgetl(fid);
end

% Read the "Analysis Options" section
fgetl(fid);
ANLS.SELF_WEIGHT = readLogical(fid);
ANLS.BUOYANCY    = readLogical(fid);     
ANLS.CENTRIF     = readLogical(fid);    
ANLS.DISP_CF     = readLogical(fid);    
ANLS.N_MODES     = readScalar(fid);
ANLS.N_ELEMS     = readScalar(fid);

% Read the "Optimization" section
fgetl(fid);
OPT.OPTIMIZE     = readLogical(fid);
OPT.OPT_METHOD   = readString(fid);
OPT.OPT_PITAXIS  = readLogical(fid);
OPT.PITAXIS_VAL  = readScalar(fid);
OPT.INB_STN      = readScalar(fid);
OPT.TRAN_STN     = readScalar(fid);
OPT.OUB_STN      = readScalar(fid);
OPT.NUM_CP       = readScalar(fid);
OPT.READ_INITX   = readLogical(fid);
OPT.INITX_FILE   = readString(fid);
OPT.WRITE_STR    = readLogical(fid);
OPT.WRITE_F_ALL  = readLogical(fid);
OPT.WRITE_X_ALL  = readLogical(fid);
OPT.WRITE_X_ITER = readLogical(fid);

% Read the "Constraints" section
fgetl(fid);
OPT.MAX_TIP_D    = readScalar(fid);
OPT.MIN_FREQ_SEP = readScalar(fid);

% Read the "Environmental Data" section
fgetl(fid);
ENV.FLUID_DEN  = readScalar(fid);
ENV.GRAV       = readScalar(fid);

% Read the "Blade Data" section
fgetl(fid);
BLADE.NUM_SEC     = readScalar(fid);
BLADE.BLD_LENGTH  = readScalar(fid);
BLADE.HUB_RAD     = readScalar(fid);
BLADE.SHAFT_TILT  = readScalar(fid);
BLADE.PRE_CONE    = readScalar(fid);
BLADE.AZIM        = readScalar(fid);
BLADE.BLD_PITCH   = readScalar(fid);
BLADE.ROT_SPD     = readScalar(fid);
BLADE.INTERP_AF   = readString(fid);
BLADE.N_AF        = readScalar(fid);
BLADE.MATS_FILE   = readString(fid);
BLADE.FILLER_DENS = readScalar(fid);

% Read the array of "Blade Data"
fgetl(fid);
fgetl(fid);
bladeDataArray = readCellArray(fid, '%f %f %f %f %f %f %f %q %q', BLADE.NUM_SEC);
BLADE.zFrac     = bladeDataArray{1}; 
BLADE.aeroTwst  = bladeDataArray{2}; 
BLADE.chord     = bladeDataArray{3}; 
BLADE.pitAxis   = bladeDataArray{4};
BLADE.px_a      = bladeDataArray{5}; 
BLADE.py_a      = bladeDataArray{6}; 
BLADE.qz_a      = bladeDataArray{7}; 
BLADE.afFile    = bladeDataArray{8}; 
BLADE.strFile   = bladeDataArray{9}; 

% Read the "Shear Web Data" section
fgetl(fid);
WEB.NUM_WEBS  = readScalar(fid);
WEB.WEB_NODES = readScalar(fid);

if OPT.OPTIMIZE
    % skip the array of "Shear Web Data", it is unused
    % move to the "Output Options" section
    line = [];
    while ~strncmpi(line,'-----  Output Options', 21)
        line = fgetl(fid);
    end

else
    % Read the array of "Shear Web Data"
    fgetl(fid);
    fgetl(fid);
    if WEB.NUM_WEBS >= 1
        webArray     = readCellArray(fid, '%f %f %f %f %f', WEB.NUM_WEBS);
        WEB.webNum   = webArray{1}; 
        WEB.inbStn   = webArray{2};
        WEB.oubStn   = webArray{3};
        WEB.inbChLoc = webArray{4}; 
        WEB.oubChLoc = webArray{5}; 
    end
    fgetl(fid);

end

% Read the "Output Options" section
OUT.TAB_DEL       = readLogical(fid);
OUT.PROPS_FILE    = readLogical(fid);
OUT.LOAD_DSP_FILE = readLogical(fid);
OUT.PANEL_FILE    = readLogical(fid);
OUT.LAMINA_FILE   = readLogical(fid);
OUT.DATA_GUI      = readLogical(fid);
OUT.SAVE_PLOTS    = readLogical(fid);
OUT.SAVE_FIG_FMT  = readString(fid);
OUT.PLOT_OPT_ITER = readLogical(fid);
OUT.PLOT_F_BLD    = readLogical(fid);
OUT.PLOT_DISP_BLD = readLogical(fid);
OUT.PLOT_GBL_SYS  = readLogical(fid);
OUT.PLOT_YMOD     = readLogical(fid);
OUT.PLOT_GMOD     = readLogical(fid);
OUT.PLOT_MASS_DEN = readLogical(fid);
OUT.PLOT_PRIN_ANG = readLogical(fid);
OUT.PLOT_AT_STFF  = readLogical(fid);
OUT.PLOT_BSTFF    = readLogical(fid);
OUT.PLOT_INER     = readLogical(fid);
OUT.PLOT_CENTERS  = readLogical(fid);                   
OUT.PLOT_NORMS    = readLogical(fid);
OUT.PLOT_SHEARS   = readLogical(fid);
OUT.PLOT_BCRIT    = readLogical(fid);
OUT.PLOT_E11      = readLogical(fid);
OUT.PLOT_E22      = readLogical(fid);
OUT.PLOT_E12      = readLogical(fid);
OUT.PLOT_S11      = readLogical(fid);
OUT.PLOT_S22      = readLogical(fid);
OUT.PLOT_S12      = readLogical(fid);
OUT.PLOT_S11_FC   = readLogical(fid);
OUT.PLOT_S22_FC   = readLogical(fid);
OUT.PLOT_S12_FC   = readLogical(fid);
OUT.PLOT_MODE_D   = readLogical(fid);
OUT.PLOT_MODE_S   = readLogical(fid);
OUT.PLOT_APPLOADS = readLogical(fid);
OUT.PLOT_RESLOADS = readLogical(fid);
OUT.PLOT_DEFLECT  = readLogical(fid);

fclose(fid); % close main input file

BLADE.zSec = BLADE.zFrac .* BLADE.BLD_LENGTH; % z-coordinate in blade coordinate system (m)

%% check for errors on user inputs within main input file
checkInpErrors(ANLS,OPT,ENV,BLADE,WEB)

%% Read the materials input file
MATS = readMaterialsFile(SIM,BLADE,OPT);

%% Read the normalized airfoil coordinates (*.prof files) and interpolate (if requested by user) 
AF.OrigNormAFcoords = readAirfoilCoordFiles(SIM, BLADE);
AF.NormAFcoords     = interpAirfoilCoords(AF.OrigNormAFcoords, BLADE);	% structure array of the interpolated chord normalized airfoil x-y coordinates, in the reference airfoil coordinate system

%% Define the coordinate systems and the tranformations matrices between them
Coord = defineCoordSystems(BLADE);

%% Determine the number of webs and cells at each station
existWeb = zeros(BLADE.NUM_SEC, WEB.NUM_WEBS);
if OPT.OPTIMIZE
    existWeb(OPT.INB_STN:OPT.OUB_STN,:) = 1;    
else
    for n = 1:WEB.NUM_WEBS
        existWeb(WEB.inbStn(n):WEB.oubStn(n),n) = 1;
    end
end
WEB.nWebs    = sum(existWeb,2);
BLADE.nCells = WEB.nWebs + 1;
    
%% if pitch axis was not defined, define it now via optimization
if OPT.OPT_PITAXIS
    BLADE.pitAxis = definePitchAxis(BLADE, OPT);
end

end % function CoBlade_init

