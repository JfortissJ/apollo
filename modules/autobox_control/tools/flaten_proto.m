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

function [signal_list] = flaten_proto(proto_struct,name)

signal_list = struct('name',{},'type',{},'width',{});

fn = fieldnames(proto_struct);
% iterate over all sub bus objects
for k=1:numel(fn)
    element =  fn{k};
     
    substruct = eval(strcat("proto_struct.",element));
    if isstruct(substruct) && isempty(fieldnames(substruct))
        continue;
    elseif isstruct(substruct)
        if name == ""
            subname = element;
        else
            subname = strcat(name,"->",element);
        end
        signal_list = cat(1,signal_list,flaten_proto(substruct,subname));
    else
        new_signal.name = strcat(name,"->",element);
        new_signal.type = class(substruct);
        new_signal.width = max(size(substruct));
        signal_list = cat(1,signal_list,new_signal);
    end
end