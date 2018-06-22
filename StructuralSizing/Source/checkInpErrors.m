%{
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
  
function checkInpErrors(ANLS,OPT,ENV,BLADE,WEB)

if ANLS.N_MODES < 0 || ~mod(ANLS.N_MODES,1) == 0
    error('ERROR: N_MODES must be a positive integer, or equal to 0.')
end

if ANLS.N_MODES > 0 && exist(fullfile(pwd,'BModes.exe'),'file') ~= 2
    fprintf(1,'Could not locate BModes.exe\n'); 
    fprintf(1,'Either set N_MODES = 0, or download the BModes v3.00.00 code from:\n');
    fprintf(1,'http://wind.nrel.gov/designcodes/preprocessors/bmodes/\n');
    error(['ERROR: could not locate BModes.exe in ' pwd ]);
end

if ANLS.N_ELEMS < 2 || ~mod(ANLS.N_ELEMS,1) == 0
    error('ERROR: N_ELEMS must be a positive integer, greater than or equal to 2.')
end

if OPT.PITAXIS_VAL < 0 || OPT.PITAXIS_VAL > 1
    error('ERROR: PITAXIS_VAL must be between 0 and 1.');
end

if ~mod(OPT.INB_STN,1) == 0 || ~mod(OPT.TRAN_STN,1) == 0 || ... 
   ~mod(OPT.OUB_STN,1) == 0 || ~mod(BLADE.NUM_SEC,1) == 0 
    error('ERROR: INB_STN, TRAN_STN, OUB_STN, and NUM_SEC must all be positive integers, and they must be related as follows: 1 <= INB_STN < TRAN_STN < OUB_STN <= NUM_SEC.');
end

if any(diff([OPT.INB_STN; OPT.TRAN_STN; OPT.OUB_STN]) <= 0) || OPT.INB_STN < 1 || OPT.OUB_STN > BLADE.NUM_SEC     
    error('ERROR: INB_STN, TRAN_STN, OUB_STN, and NUM_SEC must all be positive integers, and they must be related as follows: 1 <= INB_STN < TRAN_STN < OUB_STN <= NUM_SEC.');
end

if OPT.NUM_CP < 2 || OPT.NUM_CP > BLADE.NUM_SEC || ~mod(OPT.NUM_CP,1) == 0
    error('ERROR: NUM_CP must be a positive integer: 2 <= NUM_CP <= NUM_SEC.');
end

if OPT.MAX_TIP_D < 0 
    error('ERROR: MAX_TIP_D must be a positive value.');
end

if ENV.FLUID_DEN < 0 
    error('ERROR: FLUID_DEN must be a positive value.');
end

if ENV.GRAV < 0 
    error('ERROR: GRAV is expected to be a positive value.');
end

if BLADE.NUM_SEC < 2 || ~mod(BLADE.NUM_SEC,1) == 0
    error('ERROR: NUM_SEC must be an integer, greater than or equal to 2.')
end

if BLADE.BLD_LENGTH < 0 
    error('ERROR: BLD_LENGTH must be a positive value.');
end

if BLADE.HUB_RAD < 0 || BLADE.HUB_RAD >= BLADE.BLD_LENGTH
    error('ERROR: HUB_RAD must be a positive value, also HUB_RAD < BLD_LENGTH');
end

if BLADE.ROT_SPD < 0 
    error('ERROR: ROT_SPD is expected to be a positive value.');
end

if BLADE.N_AF < 4 || ~mod(BLADE.N_AF,1) == 0
    error('ERROR: N_AF must be a positive integer, and greater than or equal to 4.');
end

if numel(BLADE.zFrac) ~= BLADE.NUM_SEC
    error('ERROR: The number rows entered in the "Blade Data" section should be equal to NUM_SEC.');
end

if any(diff(BLADE.zFrac) <= 0)
    error('ERROR: The blade station data is not entered in sequential order, or duplicate stations exist.');
end

if BLADE.zFrac(1) ~= 0 || BLADE.zFrac(end) ~= 1
    error('ERROR: The first and last values of zFrac should be equal to 0 and 1, respectively.');
end

if any(BLADE.chord) <= 0
    error('ERROR: The chord length must be greater than 0.');
end

if any(BLADE.pitAxis) < 0 || any(BLADE.pitAxis) > 1
    error('ERROR: Values for pitAxis must be between 0 and 1.');
end

if WEB.NUM_WEBS < 0 || ~mod(WEB.NUM_WEBS,1) == 0
    error('ERROR: NUM_WEBS must be an integer, greater than or equal to 0.')
end

if WEB.WEB_NODES < 3 || ~mod(WEB.WEB_NODES,1) == 0
    error('ERROR: WEB_NODES must be an integer, greater than or equal to 3.')
end

if ~OPT.OPTIMIZE
    if any(diff(WEB.webNum) <= 0) || any(~mod(WEB.webNum,1) == 0)
        error('ERROR: The web data must be entered in sequential order, webNum must be a list of monotonically increasing integers.');
    end
    if any(WEB.inbStn < 1) || any(WEB.inbStn > BLADE.NUM_SEC) || any(~mod(WEB.inbStn,1) == 0)
        error('ERROR: The values of inbStn must be integers between 1 and NUM_SEC (inclusive)');
    end
    if any(WEB.oubStn < 1) || any(WEB.oubStn > BLADE.NUM_SEC) || any(~mod(WEB.oubStn,1) == 0)
        error('ERROR: The values of oubStn must be integers between 1 and NUM_SEC (inclusive)');
    end
    if any(WEB.inbStn > WEB.oubStn)
        error('ERROR: The values of inbStn must be less than or equal to oubStn');
    end
    if any(WEB.inbChLoc < 0) || any(WEB.inbChLoc > 1) || ...
       any(WEB.oubChLoc < 0) || any(WEB.oubChLoc > 1)     
        error('ERROR: Values of inbChLoc and oubChLoc should be between 0 and 1.');
    end
end

end % function checkInpErrors

