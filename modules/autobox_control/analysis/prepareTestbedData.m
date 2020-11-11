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

function [trajectory_testbed_matrix, localization_testbed_matrix, vehicle_can_testbed_matrix, time_testbed_vector] = prepareTestbedData(cda)

nr_idxs = length(cda.res.Time);

nr_traj_points = 11002;
nr_loca_points = 27;
nr_can_points = 15;

trajectory_testbed_matrix = zeros(nr_traj_points, nr_idxs);
localization_testbed_matrix = zeros(nr_loca_points, nr_idxs);
vehicle_can_testbed_matrix = zeros(nr_can_points, nr_idxs);

time_testbed_vector = cda.res.Time;

for i=1:nr_idxs
   [~, trajectory_vector, ~, localization_vector, ~, vehicle_data_can_vector] = getRecordedStateAtIndex(cda, i);
   trajectory_testbed_matrix(:,i) = trajectory_vector;
   localization_testbed_matrix(:,i) = localization_vector;
   vehicle_can_testbed_matrix(:,i) = vehicle_data_can_vector;
end

end