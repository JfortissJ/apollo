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

function [ref, idx_larger, idx_smaller, interp_factor, time, time_smaller, time_larger, lmt, tts]  = ref_point_determination_time_based(trajectory, localization)

time_smaller = 0;
time_larger = 0;

idx_larger = double(-2);
idx_smaller = double(-1);
ref = struct('x',double(0), 'y',double(0), ...
    'z', double(0), 'theta', double(0),'kappa',double(0),...
    'dkappa', double(0),'ddkappa', double(0),'s',double(0),...
    'v',double(0),'a', double(0),'relative_time',double(0));

interp_factor = 0;
% todo: check only for valid trajectory points (max number)
% todo: consider absolute timing

% subtract absolute timestamp and search only in relative coordinates
time = localization.MeasurementTime-trajectory.timestamp_sec; 
lmt = localization.MeasurementTime;
tts = trajectory.timestamp_sec; 

% defines max number of valid trajectory points
trajectory_len =  min(trajectory.n_trajectory_point, length(trajectory.x)); 

if(trajectory_len < 1)
   %print("Trajectory is empty."); 
   return;
end

idx_larger = 1;
while(trajectory.relative_time(idx_larger) < time)
    idx_larger = idx_larger+1;
    if((idx_larger>trajectory_len) )
        %loca time is larger than last trajectory point, take last
        %trajectory point
        ref.x = trajectory.x(trajectory_len);
        ref.y = trajectory.y(trajectory_len);
        ref.z = trajectory.z(trajectory_len);
        ref.theta = trajectory.theta(trajectory_len);
        ref.kappa = trajectory.kappa(trajectory_len);
        ref.dkappa = trajectory.dkappa(trajectory_len);
        ref.ddkappa = trajectory.ddkappa(trajectory_len);
        ref.s = trajectory.s(trajectory_len);
        ref.v = trajectory.v(trajectory_len);
        ref.a = trajectory.a(trajectory_len);
        return;
    end
end
idx_smaller = idx_larger - 1;

% loca time and trajectory point match exactly 
% no interpolation required
if( trajectory.relative_time(idx_larger) == time || idx_larger == 1)
    ref.x = trajectory.x(idx_larger);
    ref.y = trajectory.y(idx_larger);
    ref.z = trajectory.z(idx_larger);
    ref.theta = trajectory.theta(idx_larger);
    ref.kappa = trajectory.kappa(idx_larger);
    ref.dkappa = trajectory.dkappa(idx_larger);
    ref.ddkappa = trajectory.ddkappa(idx_larger);
    ref.s = trajectory.s(idx_larger);
    ref.v = trajectory.v(idx_larger);
    ref.a = trajectory.a(idx_larger);

% interpolation
elseif (idx_larger > 1) 
    trajectory_point_smaller = [...
        trajectory.x(idx_smaller),...
        trajectory.y(idx_smaller),...
        trajectory.z(idx_smaller),...
        trajectory.theta(idx_smaller),...
        trajectory.kappa(idx_smaller),...
        trajectory.dkappa(idx_smaller),...
        trajectory.ddkappa(idx_smaller),...
        trajectory.s(idx_smaller),...
        trajectory.v(idx_smaller),...
        trajectory.a(idx_smaller)
    ];
    trajectory_point_larger = [...
        trajectory.x(idx_larger),...
        trajectory.y(idx_larger),...
        trajectory.z(idx_larger),...
        trajectory.theta(idx_larger),...
        trajectory.kappa(idx_larger),...
        trajectory.dkappa(idx_larger),...
        trajectory.ddkappa(idx_larger),...
        trajectory.s(idx_larger),...
        trajectory.v(idx_larger),...
        trajectory.a(idx_larger)
    ];
    
    time_smaller = trajectory.relative_time(idx_smaller);
    time_larger = trajectory.relative_time(idx_larger);
    
    interp_factor = (time-time_smaller)/(time_larger-time_smaller);
    interp_trajectory_point = trajectory_point_smaller + interp_factor*(trajectory_point_larger-trajectory_point_smaller);

    % interpolate orientation considering jump at a full turn (unit circle
    % transformation -> then atan2)
    y_unit_circle_interp = (1-interp_factor)*sin(trajectory_point_smaller(4)) + interp_factor*sin(trajectory_point_larger(4));
    x_unit_circle_interp = (1-interp_factor)*cos(trajectory_point_smaller(4)) + interp_factor*cos(trajectory_point_larger(4));
    theta_interpolated = atan2(y_unit_circle_interp, x_unit_circle_interp); 
    theta_2pi = mod(theta_interpolated, 2*pi); % [-2pi, 2pi]
    if theta_2pi > 2*pi
        interp_trajectory_point(4) = theta_2pi - 2*pi;
    elseif theta_2pi < -2*pi
        interp_trajectory_point(4) = theta_2pi + 2*pi;
    else
        interp_trajectory_point(4) = theta_2pi;
    end
    
    ref.x = interp_trajectory_point(1);
    ref.y = interp_trajectory_point(2);
    ref.z = interp_trajectory_point(3);
    ref.theta = interp_trajectory_point(4);
    ref.kappa = interp_trajectory_point(5);
    ref.dkappa = interp_trajectory_point(6);
    ref.ddkappa = interp_trajectory_point(7);
    ref.s = interp_trajectory_point(8);
    ref.v = interp_trajectory_point(9);
    ref.a = interp_trajectory_point(10);
    
 % no appropriate matching found   
else
 error("This error should not occur.");
end
return;
