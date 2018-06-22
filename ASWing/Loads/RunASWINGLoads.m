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
  
function [ optim ] = RunASWINGLoads( optim, entity, useStructure)
% Runs the aircraft through a series of unsteady load cases and outputs
% forces and moments at each structural node. Depending what entity is
% passed in a different load set is run based on which cases size the
% desired entity. 

%Based on surface being sized select appropriate operating mode for sizing
if strcmp(entity, 'wing')
    operatingMode = 'wingloads';
elseif strcmp(entity, 'htail') || strcmp(entity, 'vtail') || strcmp(entity, 'boom')
    operatingMode = 'tailloads';
end

% Write .ASW Aircraft file
optim = WriteASWingFile(optim, [optim.ASWINGIODir 'LoadsAircraft' num2str(optim.designID) '.asw'] , operatingMode, useStructure, 'LoadsADB');
%
%% Write ASWing input string
if strcmp(operatingMode, 'wingloads')
    % write input strings
    WriteWingSizingLoadsInput( optim )
    % run cases and save outputs to file
    system(['aswing<' optim.ASWINGIODir 'EmpanngeDeflectioLoadsInput' num2str(optim.designID) '.txt>' GetNullDevice() ' 2>&1']);
    system(['aswing<' optim.ASWINGIODir 'UnsteadyInputString' num2str(optim.designID) '.txt>' GetNullDevice() ' 2>&1']);
    system(['aswing<' optim.ASWINGIODir 'WingControlSurfaceImpulseInputString' num2str(optim.designID) '.txt>' GetNullDevice() ' 2>&1']);
    system(['aswing<' optim.ASWINGIODir 'DARPAGustInputString' num2str(optim.designID) '.txt>' GetNullDevice() ' 2>&1']);
    system(['aswing<' optim.ASWINGIODir 'CGz' num2str(optim.designID) '.txt>' optim.ASWINGIODir 'ZCG' num2str(optim.designID) '.txt' ' 2>&1']);  
elseif strcmp(operatingMode, 'tailloads')
    % write input strings
    WriteTailandBoomSizingLoadsInput( optim )
    % run cases and save outputs to file
    system(['aswing<' optim.ASWINGIODir 'UnsteadyInputString' num2str(optim.designID) '.txt>' GetNullDevice() ' 2>&1']);
    system(['aswing<' optim.ASWINGIODir 'EmpanngeDeflectioLoadsInput' num2str(optim.designID) '.txt>' GetNullDevice() ' 2>&1']);
    system(['aswing<' optim.ASWINGIODir 'DARPAGustInputString' num2str(optim.designID) '.txt>' GetNullDevice() ' 2>&1']);
    system(['aswing<' optim.ASWINGIODir 'CGz' num2str(optim.designID) '.txt>' optim.ASWINGIODir 'ZCG' num2str(optim.designID) '.txt' ' 2>&1']);
end

%% Parse Outputs
if strcmp(operatingMode, 'wingloads')
    lastWingIndex =  length(optim.wing.structure.spanLocation)*2;
    
    %% parse files and add each time step to martix
    structuralTable = [ParseStructuralQuantities([optim.ASWINGIODir 'HtailDeflectionLoads' num2str(optim.designID) '.txt']);
        ParseStructuralQuantities([optim.ASWINGIODir 'VtailDeflectionLoads' num2str(optim.designID) '.txt']);
        ParseStructuralQuantities([optim.ASWINGIODir 'LoadsOutput2' num2str(optim.designID) '.txt']); ...
        ParseStructuralQuantities([optim.ASWINGIODir 'GustLoadsOutput' num2str(optim.designID) '.txt']); ...
        ParseStructuralQuantities([optim.ASWINGIODir 'UnsteadyGustLoadsOutput' num2str(optim.designID) '.txt'])];
    
    % record load case type
    loadCaseType(1:size(ParseStructuralQuantities([optim.ASWINGIODir 'HtailDeflectionLoads' num2str(optim.designID) '.txt']),1),1) = {'HtailDeflection'};
    loadCaseType(end+1:end+size(ParseStructuralQuantities([optim.ASWINGIODir 'VtailDeflectionLoads' num2str(optim.designID) '.txt']),1),1) = {'VtailDeflection'};
    loadCaseType(end+1:end+size(ParseStructuralQuantities([optim.ASWINGIODir 'LoadsOutput2' num2str(optim.designID) '.txt']),1),1) = {'WingControlSurfDef'};
    loadCaseType(end+1:end+size(ParseStructuralQuantities([optim.ASWINGIODir 'GustLoadsOutput' num2str(optim.designID) '.txt']) ,1),1) = {'Gust'};
    loadCaseType(end+1:end+size(ParseStructuralQuantities([optim.ASWINGIODir 'UnsteadyGustLoadsOutput' num2str(optim.designID) '.txt']),1),1) = {'Gust'};
    
    % remove NANs
    structuralTable = structuralTable(:,~any(isnan(structuralTable),1));
    
    % remove first two columns that contain tstep and and i
    structuralTable(:,1) = [];
    structuralTable(:,1) = [];
    
    %Break up into output vectors
    Fc    =  [structuralTable(:,1:9:end)];
    Fs    =  [structuralTable(:,2:9:end)];
    Fn    =  [structuralTable(:,3:9:end)];
    Mc    =  [structuralTable(:,4:9:end)];
    Ms    =  [structuralTable(:,5:9:end)];
    Mn    =  [structuralTable(:,6:9:end)];
    % rx_m  =  structuralTable(:,7:9:end);
    % ry_m  =  structuralTable(:,8:9:end);
    % rz_m  =  structuralTable(:,9:9:end);
end

if strcmp(operatingMode,'tailloads')
    previousLastInstance = 0;
    %get last indicies
    enum = 1;
    for b = 1:length(optim.aircraft.beamNames)
        if ~max(strcmp(optim.aircraft.beamNames{b}, {'pod', 'wing'}))
            entity = optim.aircraft.beamNames{b};
            entities{enum} = entity;
            loads.(entity).startIndex = previousLastInstance + 1; % assumes code is constructed so that al instances of a surface have the same number of sensors
            loads.(entity).sensorsPerInstance = length(optim.(entity).sensorLocationsY_m{1});
            loads.(entity).numInstances = sum(optim.(entity).N);
            previousLastInstance = previousLastInstance + loads.(entity).sensorsPerInstance ;
            enum = enum+1;
        end
    end 
    
    % get unstead loads
    structuralTable = [ParseStructuralQuantities([optim.ASWINGIODir 'VtailDeflectionLoads' num2str(optim.designID) '.txt']); ...
        ParseStructuralQuantities([optim.ASWINGIODir 'HtailDeflectionLoads' num2str(optim.designID) '.txt']); ...
        ParseStructuralQuantities([optim.ASWINGIODir 'GustLoadsOutput' num2str(optim.designID) '.txt']);
        ParseStructuralQuantities([optim.ASWINGIODir 'UnsteadyGustLoadsOutput' num2str(optim.designID) '.txt'])];
    
    % record load case type
    loadCaseType(1:size(ParseStructuralQuantities([optim.ASWINGIODir 'VtailDeflectionLoads' num2str(optim.designID) '.txt']),1),1) = {'HtailDeflection'};
    loadCaseType(end+1:end+size(ParseStructuralQuantities([optim.ASWINGIODir 'HtailDeflectionLoads' num2str(optim.designID) '.txt']),1),1) = {'VtailDeflection'};
    loadCaseType(end+1:end+size(ParseStructuralQuantities([optim.ASWINGIODir 'GustLoadsOutput' num2str(optim.designID) '.txt']) ,1),1) = {'Gust'};
    loadCaseType(end+1:end+size(ParseStructuralQuantities([optim.ASWINGIODir 'UnsteadyGustLoadsOutput' num2str(optim.designID) '.txt']),1),1) = {'Gust'};
    
    
    % remove NANs
    structuralTable = structuralTable(:,~any(isnan(structuralTable),1));
    
    % remove first two colums containg tstep and i
    structuralTable(:,1) = [];
    structuralTable(:,1) = [];
    
    Fc    =  [structuralTable(:,1:9:end)];
    Fs    =  [structuralTable(:,2:9:end)];
    Fn    =  [structuralTable(:,3:9:end)];
    Mc    =  [structuralTable(:,4:9:end)];
    Ms    =  [structuralTable(:,5:9:end)];
    Mn    =  [structuralTable(:,6:9:end)];
end

%% Add output quantities to optim
if strcmp(operatingMode, 'wingloads')
    
    %get wing values
    optim.wing.structure.limitLoads.Fc = Fc(:,1:1:lastWingIndex);
    optim.wing.structure.limitLoads.Fs = Fs(:,1:1:lastWingIndex);
    optim.wing.structure.limitLoads.Fn = Fn(:,1:1:lastWingIndex);
    optim.wing.structure.limitLoads.Mc = Mc(:,1:1:lastWingIndex);
    optim.wing.structure.limitLoads.Ms = Ms(:,1:1:lastWingIndex);
    optim.wing.structure.limitLoads.Mn = Mn(:,1:1:lastWingIndex);
    
    % Get worst case aero loads
    optim.wing.structure.rawLoads.Mn = [optim.wing.structure.limitLoads.Mn(:,1:2:end);optim.wing.structure.limitLoads.Mn(:,2:2:end)];
    optim.wing.structure.rawLoads.Mc = [optim.wing.structure.limitLoads.Mc(:,1:2:end);optim.wing.structure.limitLoads.Mc(:,2:2:end)];
    optim.wing.structure.rawLoads.Ms = [optim.wing.structure.limitLoads.Ms(:,1:2:end);optim.wing.structure.limitLoads.Ms(:,2:2:end)];
    optim.wing.structure.rawLoads.Fn = [optim.wing.structure.limitLoads.Fn(:,1:2:end);-1*optim.wing.structure.limitLoads.Fn(:,2:2:end)];
    optim.wing.structure.rawLoads.Fc = [optim.wing.structure.limitLoads.Fc(:,1:2:end);-1.*optim.wing.structure.limitLoads.Fc(:,2:2:end)];
    optim.wing.structure.rawLoads.Fs = [optim.wing.structure.limitLoads.Fs(:,1:2:end);-1.*optim.wing.structure.limitLoads.Fs(:,2:2:end)];
    
%     optim.wing.structure.limitLoads.Mn  = max( abs([optim.wing.structure.limitLoads.Mn(:,1:2:end);optim.wing.structure.limitLoads.Mn(:,2:2:end)]));
%     optim.wing.structure.limitLoads.Mc  = max( abs([optim.wing.structure.limitLoads.Mc(:,1:2:end);optim.wing.structure.limitLoads.Mc(:,2:2:end)]));
%     optim.wing.structure.limitLoads.Ms  = max( abs([optim.wing.structure.limitLoads.Ms(:,1:2:end);optim.wing.structure.limitLoads.Ms(:,2:2:end)]));
%     optim.wing.structure.limitLoads.Fn  = max( abs([optim.wing.structure.limitLoads.Fn(:,1:2:end);-1*optim.wing.structure.limitLoads.Fn(:,2:2:end)]));
%     optim.wing.structure.limitLoads.Fc  = max( abs([optim.wing.structure.limitLoads.Fc(:,1:2:end);-1.*optim.wing.structure.limitLoads.Fc(:,2:2:end)]));
%     optim.wing.structure.limitLoads.Fs  = max( abs([optim.wing.structure.limitLoads.Fs(:,1:2:end);-1.*optim.wing.structure.limitLoads.Fs(:,2:2:end)]));
    
    % populate load case type
    optim.wing.structure.caseType = [loadCaseType;loadCaseType];
    
    % get CGz
    optim.ZCG_m = GetZCG(['ZCG' num2str(optim.designID) '.txt']);
    
elseif strcmp(operatingMode, 'tailloads')
    
    % loop through entities and slice each surface
    for e = 1:length(entities)
        firstIndex = loads.(entities{e}).startIndex;
        lastIndex = loads.(entities{e}).startIndex + loads.(entities{e}).sensorsPerInstance * 1 - 1;
        
        % assign loads
        [optim.(entities{e}).structure.limitLoads.Fc, optim.(entities{e}).structure.limitLoads.Fs,  optim.(entities{e}).structure.limitLoads.Fn, ...
            optim.(entities{e}).structure.limitLoads.Mc, optim.(entities{e}).structure.limitLoads.Ms, optim.(entities{e}).structure.limitLoads.Mn] ...
            = deal (Fc(:,firstIndex:1:lastIndex) , Fs(:,firstIndex:1:lastIndex), Fn(:,firstIndex:1:lastIndex), ...
            Mc(:,firstIndex:1:lastIndex), Ms(:,firstIndex:1:lastIndex), Mn(:,firstIndex:1:lastIndex), optim.(entities{e}).structure);
        
        % check is surface entity boom and if so set repeated points factor to
        % 1, else 2. repeatedPoints factor indicates if surface is reflected
        % and every other point should be taken or not.
        if strcmp((entities{e}), 'boom')
            repeatedPointsFactor = 1;
        else
            repeatedPointsFactor = 2;
        end
        
        % break up entity output per instance
        optim.(entities{e}).structure.limitLoads.Mn = FindLimitLoads(optim.(entities{e}).structure.limitLoads.Mn, loads.(entities{e}).numInstances, loads.(entities{e}).sensorsPerInstance, repeatedPointsFactor);
        optim.(entities{e}).structure.limitLoads.Mc = FindLimitLoads(optim.(entities{e}).structure.limitLoads.Mc, loads.(entities{e}).numInstances, loads.(entities{e}).sensorsPerInstance, repeatedPointsFactor);
        optim.(entities{e}).structure.limitLoads.Ms = FindLimitLoads(optim.(entities{e}).structure.limitLoads.Ms, loads.(entities{e}).numInstances, loads.(entities{e}).sensorsPerInstance, repeatedPointsFactor);
        optim.(entities{e}).structure.limitLoads.Fn = FindLimitLoads(optim.(entities{e}).structure.limitLoads.Fn, loads.(entities{e}).numInstances, loads.(entities{e}).sensorsPerInstance, repeatedPointsFactor);
        optim.(entities{e}).structure.limitLoads.Fc = FindLimitLoads(optim.(entities{e}).structure.limitLoads.Fc, loads.(entities{e}).numInstances, loads.(entities{e}).sensorsPerInstance, repeatedPointsFactor);
        optim.(entities{e}).structure.limitLoads.Fs = FindLimitLoads(optim.(entities{e}).structure.limitLoads.Fs, loads.(entities{e}).numInstances, loads.(entities{e}).sensorsPerInstance, repeatedPointsFactor);
         
        optim.(entities{e}).structure.rawLoads.Mn = FormatTailLoads(optim.(entities{e}).structure.limitLoads.Mn, loads.(entities{e}).numInstances, loads.(entities{e}).sensorsPerInstance, repeatedPointsFactor);
        optim.(entities{e}).structure.rawLoads.Mc = FormatTailLoads(optim.(entities{e}).structure.limitLoads.Mc, loads.(entities{e}).numInstances, loads.(entities{e}).sensorsPerInstance, repeatedPointsFactor);
        optim.(entities{e}).structure.rawLoads.Ms = FormatTailLoads(optim.(entities{e}).structure.limitLoads.Ms, loads.(entities{e}).numInstances, loads.(entities{e}).sensorsPerInstance, repeatedPointsFactor);
        optim.(entities{e}).structure.rawLoads.Fn = FormatTailLoads(optim.(entities{e}).structure.limitLoads.Fn, loads.(entities{e}).numInstances, loads.(entities{e}).sensorsPerInstance, repeatedPointsFactor);
        optim.(entities{e}).structure.rawLoads.Fc = FormatTailLoads(optim.(entities{e}).structure.limitLoads.Fc, loads.(entities{e}).numInstances, loads.(entities{e}).sensorsPerInstance, repeatedPointsFactor);
        optim.(entities{e}).structure.rawLoads.Fs = FormatTailLoads(optim.(entities{e}).structure.limitLoads.Fs, loads.(entities{e}).numInstances, loads.(entities{e}).sensorsPerInstance, repeatedPointsFactor);
    
         %Populate Load Case Type
        optim.(entities{e}).structure.caseType = repmat(loadCaseType,repeatedPointsFactor,1);
    end
    
    %Get CGz
    optim.ZCG_m = GetZCG([optim.ASWINGIODir 'ZCG' num2str(optim.designID) '.txt']); 
end
end

