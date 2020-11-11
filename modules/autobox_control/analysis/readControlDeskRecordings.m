%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Apollo Autobox Control
% Copyright (C) 2020 fortiss GmbH
% Authors: Tobias Kessler, Jianjie Lin, Julian Bernhard, Klemens Esterle,
% Patrick Hart
%
% This library is free software; you can redistribute it and/or modify it
% under the terms of the GNU Lesser General Public License as published by
% the Free Software Foundation; either version 3 of the License, or (at 
% your option) any later version.
%
% This library is distributed in the hope that it will be useful, but 
% WITHOUT ANY WARRANTY; without even the implied warranty of 
% MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU Lesser 
% General Public License for more details.
%
% You should have received a copy of the GNU Lesser General Public License
% along with this library; if not, write to the Free Software Foundation,
% Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301, USA.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [ results ] = readControlDeskRecordings( pathToRaw )
%% readControlDeskRecordings
% rearranges the structure of the Control Desk Recorded Data:
%
% results
%  - Time
%  - Length
%  - Comment
%  + Bus1
%     - Signal1
%     - Signal2
%     - ....
%  + Bus2
%     - ...
%  + ...
%
% Inputs:
%		pathToRaw	(absolute) location of the file you want to read
%
% Outputs:
%		results		rearranged results

%% load data
if ~exist(pathToRaw,'file')
	error(['File ', pathToRaw, ' does not exist']);
end

file_info = dir(fullfile(fileparts(which(pathToRaw)),pathToRaw));
matfile= load(fullfile(fileparts(which(pathToRaw)),pathToRaw));
name = fieldnames(matfile);
raw = matfile.(name{1});


%% process data
s = size(raw.Y,2);
list_of_signals = cell(s,1);

% Add file creation time plus recording start time
results.EndTime = file_info.date;
results.StartTime = raw.Description.General.DateTime;

% Add time
results.Time = raw.X.Data;

% Add comment
results.Comment = raw.Description.General.Description;

% Add lenght of vectors
results.length = size(raw.X.Data,2);

% Add Signals, decomposed in the bus structure
% NOTE: Non-generic, recordings have to match the structure!!
for i=1:s
	name = raw.Y(i).Name;
	name = strrep(name,'<','');
	name = strrep(name,'>','');
    name = strrep(name,'{','_');
	name = strrep(name,'}','_');
    name = strrep(name,'(','_');
	name = strrep(name,')','_');
    name = strrep(name,' ','_');
	%name = strrep(name,']','_');
    %name = strrep(name,'[','_');
	name = strrep(name,'-','_');
    name = strrep(name,'\n','');
    
    idx_name_split = strsplit(name,{'[',']'});
    if length(idx_name_split)>1
        name = idx_name_split{1};
        idx = ['i',idx_name_split{2}];
        substructure = strsplit(name,'.');
        substructure = [substructure, idx];
    else
        name = idx_name_split{1};
        substructure = strsplit(name,'.');
    end
	
	results = setfield(results, substructure{:}, raw.Y(i).Data);
	
	list_of_signals{i} = name;
end

% Add list of all recorded signals
results.SignalsList = list_of_signals;

end

