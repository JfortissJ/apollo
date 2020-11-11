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

function res = convert_logsout(logsout)

    n = numElements(logsout);
   
    for i = 1:n
       
        %if strcmp(logsout{i}.BlockPath, 'Trajectory')
        if contains(logsout{i}.BlockPath.getBlock(1), 'Trajectory')
            res.TrajectoryBus = logsout{i}.Values;
        elseif contains(logsout{i}.BlockPath.getBlock(1), 'Localization')
            res.LocalizationBus = logsout{i}.Values; 
        end
        
    end
    
    res.Time = res.TrajectoryBus.a.Time;
    res.Comment = 'offline logs';
    
end