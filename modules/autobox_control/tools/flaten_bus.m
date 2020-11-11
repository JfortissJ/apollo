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

function [signal_list] = flaten_bus(bus_object,name)

signal_list = struct('name',{},'type',{},'width',{});

% iterate over all sub bus objects
for k=1:numel(bus_object.Elements)
    element =  bus_object.Elements(k);
     
    if contains(element.DataType,'Bus')
        del_str = "Bus: ";
        sub_bus_name = erase(element.DataType, del_str);
        sub_bus_object = evalin('base',sub_bus_name);
        signal_list = cat(1, flaten_bus(sub_bus_object, strcat(name,"_",sub_bus_name )), signal_list);   
    else
        new_signal = struct('name',strcat(name,"_",element.Name),'type',element.DataType,'width',element.Dimensions);
        signal_list = cat(1,signal_list,new_signal);
    end
end