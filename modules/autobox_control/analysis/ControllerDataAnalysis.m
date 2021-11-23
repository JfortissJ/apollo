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

classdef ControllerDataAnalysis  < PlotBase
    
    %% Properties
    properties (SetAccess = public, GetAccess = public)
        res; %Actual data to plot
        active; %Vector of valid state indices
    end
    
    %% Public Methods
    methods
        
        %% Constructor
		% Distinguish the four cases: ControlDesk Logs, ADTF Logs,
		% OfflineControllerTestbed Logs, Result Strucutre 
        function self = ControllerDataAnalysis(res_)
			if isa(res_,'struct') %already a result data structure
				self.res = res_;
			elseif isa(res_,'char')
				[~, ~, ext] = fileparts(res_);
				if strcmp(ext,'.mat') %a ControlDesk Log
					self.res = readControlDeskRecordings(res_);
                    %postprocess to new control desk logger
                    if isfield(self.res,'LocalizationBus')
                        self.res.Localization = self.res.LocalizationBus;
                    end
                    if isfield(self.res,'TrajectoryBus')
                        self.res.Trajectory = self.res.TrajectoryBus;
                    end
				else
					error('If you pass a string it has to be a filename. MAT files will be interpreted as ControlDesk Logs, CSV files as ADTF SignalLogs.');
                end
			elseif isa(res_,'Simulink.SimulationData.Dataset') %Offline Controller Simulink Testbed logs
				self.res = convert_logsout(res_);
			else
				error('Pass path to file or Result Structe as argument for the constructor!');
			end
			self.setPlotStructName({'res'});
			calcActive(self);
            isValid(self);
        end
        
        %% Destructor
        function delete(self)
            %nothing to do here
        end
        
        %% Setter for self.res.
        function setRes(self, res_)
            self.res = res_;
            calcActive(self);
        end
        
        %% Get valid states indices (aka controller state != 0,1)
        function calcActive(self)
            self.active = 1:length(self.res.Time);
        end
        
        %% Check if control desk log is not broken
        function isValid(self)
           if(sum(self.res.Localization.MeasurementTime) == 0 && ...
              sum(self.res.ControlBus.timestamp_sec) == 0 && ...
              sum(self.res.Trajectory.timestamp_sec) == 0)
               disp('!! All data zero!');
           end
           if(length(self.res.Localization.MeasurementTime) == 1)
               disp('!! Log empty!');
           end
        end
		
		%% Acc Steer
        function plotAccSteer(self)
           self.doNextPlot();
           clf; hold on;
           set(gcf, 'Name', 'Acc + Steering');
           
           subplot(2,1,1);
           hold on
           title('acc');
           %acc = sqrt(self.res.Localization.Pose.LinearAcceleration.x.^2+self.res.Localization.Pose.LinearAcceleration.y.^2);
           acc = self.res.Localization.Pose.LinearAcceleration.x./cos(self.res.Localization.Pose.Heading);
           plot(self.res.Time(self.active), acc(self.active));
           plot(self.res.Time(self.active), self.res.VehicleDataCan.Motion.LongitudinalAcceleration(self.active));
           plot(self.res.Time(self.active), self.res.acc_limited(self.active));
           plot(self.res.Time(self.active), self.res.controller_active(self.active));
           legend('ist loca', 'ist can', 'control command', 'controller active')
           
           subplot(2,1,2);
           hold on
           title('steering')
           steering_angle_range_rad_to_steering_wheel_angle_range_deg_gain = 852.7216;
           steer = self.res.VehicleDataCan.Steering.SteeringWheelAngle(self.active)/steering_angle_range_rad_to_steering_wheel_angle_range_deg_gain;
           sign=cast(self.res.VehicleDataCan.Steering.SteeringWheelAngleSign(self.active),'like', steer);
           sign(sign==1) = -1;
           sign(sign==0) = 1;
           plot(self.res.Time(self.active), steer.*sign);
           plot(self.res.Time(self.active), self.res.steering_angle_limited(self.active));
           plot(self.res.Time(self.active), self.res.controller_active(self.active));
           legend('can', 'control command', 'controller active')
        end
        
        %% Loca
        function plotLoca(self)
           self.doNextPlot(); clf; hold on;
           set(gcf, 'Name', 'Localization');
           
           subplot(5,1,1);
           hold on
           title('x');
           plot(self.res.Time(self.active), self.res.Localization.Pose.Position.x(self.active));
           
           subplot(5,1,2);
           hold on
           title('y')
           plot(self.res.Time(self.active), self.res.Localization.Pose.Position.y(self.active));
           
           subplot(5,1,3);
           hold on
           title('theta')
           plot(self.res.Time(self.active), wrapTo2Pi(self.res.Localization.Pose.Heading(self.active)));
           
           subplot(5,1,4);
           hold on
           title('v')
           v = sqrt(self.res.Localization.Pose.LinearVelocity.x(self.active).^2 + self.res.Localization.Pose.LinearVelocity.y(self.active).^2);
           v1 = self.res.Localization.Pose.LinearVelocity.x(self.active)./cos(self.res.Localization.Pose.Heading(self.active));
           v2 = self.res.Localization.Pose.LinearVelocity.y(self.active)./sin(self.res.Localization.Pose.Heading(self.active));
           plot(self.res.Time(self.active), v);
           plot(self.res.Time(self.active), v1);
           plot(self.res.Time(self.active), v2);
           
           subplot(5,1,5);
           hold on
           title('Measurement Time')
           t=self.res.Localization.MeasurementTime(self.active);
           t(t<=0.001) = nan;
           plot(self.res.Time(self.active), t); 
           %plot(self.res.Time(self.active), self.res.Trajectory.timestamp_sec(self.active));  
            
        end
        
        function plotLocaXY(self)
           self.doNextPlot(); clf; hold on;
           set(gcf, 'Name', 'Localization XY');
           hold on
           xlabel('x')
           ylabel('y')
           plot(self.res.Localization.Pose.Position.x(self.active), self.res.Localization.Pose.Position.y(self.active),'x-');
        end
        
        %% Loca Frequncy
        function plotLocaFrequency(self)
           self.doNextPlot(); clf; hold on;
           set(gcf, 'Name', 'Localization Frequency');
           
           plot(self.res.Time(1:end-1), diff(self.res.Time),'b-'); 
           plot(self.res.Time(1:end-1), min(10,diff(self.res.Localization.Pose.Position.x)),'r-'); 
           plot(self.res.Time(1:end-1), min(10,diff(self.res.Localization.Pose.Position.y)),'c-'); 
           plot(self.res.Time(1:end-1), diff(rad2deg(wrapTo2Pi(self.res.Localization.Pose.Heading))),'k-'); 
           plot(self.res.Time(1:end-1), diff(self.res.Localization.MeasurementTime),'g-');
           legend('diff time', 'diff loca pose x', 'diff loca pose y', 'diff loca pose theta', 'diff loca time')
           
           disp(max(diff(self.res.Localization.MeasurementTime)));
           
        end
        
        function plotLocaVsReference(self)
            self.doNextPlot(); clf; hold on;
           set(gcf, 'Name', 'Localization vs. Reference');
           
           subplot(5,1,1);
           hold on
           title('x');
           plot(self.res.Time(self.active), self.res.Localization.Pose.Position.x(self.active));
           plot(self.res.Time(self.active), self.res.ReferencePoint.x(self.active));
           legend('Loca', 'Ref');
           
           subplot(5,1,2);
           hold on
           title('y')
           plot(self.res.Time(self.active), self.res.Localization.Pose.Position.y(self.active));
           plot(self.res.Time(self.active), self.res.ReferencePoint.y(self.active));
           legend('Loca', 'Ref');
           
           subplot(5,1,3);
           hold on
           title('theta')
           plot(self.res.Time(self.active), wrapTo2Pi(self.res.Localization.Pose.Heading(self.active)));
           plot(self.res.Time(self.active), wrapTo2Pi(self.res.ReferencePoint.theta(self.active)));
           
           dx = diff(self.res.Localization.Pose.Position.x(self.active));
           dy = diff(self.res.Localization.Pose.Position.y(self.active));
           theta_calc = atan2(dy, dx);
           tt=self.res.Time(self.active);
           %plot(tt(1:end-1), wrapTo2Pi(theta_calc));
           
           legend('Loca', 'Ref');
           subplot(5,1,4);
           hold on
           title('v')
           v = sqrt(self.res.Localization.Pose.LinearVelocity.x(self.active).^2 + self.res.Localization.Pose.LinearVelocity.y(self.active).^2);
           plot(self.res.Time(self.active), v);
           plot(self.res.Time(self.active), self.res.ReferencePoint.v(self.active));
           legend('Loca', 'Ref');
           
           subplot(5,1,5);
           hold on
           title('kappa')
           ss = self.res.VehicleDataCan.Steering.SteeringWheelAngleSign;
           kappa_is = tan(self.res.VehicleDataCan.Steering.SteeringWheelAngle./852.7216)./2.786;
           kappa_is(ss==true) = -kappa_is(ss==true);
           plot(self.res.Time(self.active), kappa_is);
           plot(self.res.Time(self.active), self.res.ReferencePoint.kappa(self.active));
           legend('CAN', 'Ref');
           
           
        end
        
        %% Trajectory
        function plotTrajectoryRangeFromTime(self, starttime, endtime)
            idx0 = self.getIdxFromTime(starttime);
            idx1 = self.getIdxFromTime(endtime);
            self.plotTrajectoryRange([idx0:idx1]);
        end
        
        function plotTrajectoryRange(self, number_trajectory_range)
           self.doNextPlot(); clf; hold on;
           set(gcf, 'Name', 'Trajectory1');
           h1 = gcf;
           
           self.doNextPlot(); clf; hold on;
           set(gcf, 'Name', 'Trajectory2');
           h2 = gcf;
           
           x_offset = self.res.Localization.Pose.Position.x(number_trajectory_range(1));
           y_offset = self.res.Localization.Pose.Position.y(number_trajectory_range(1));

%            number_trajectory = 200; % i.e. first trajectory that get's published
            for number_trajectory = number_trajectory_range
                
               if iscell(self.res.Trajectory)
                % data from apollo bag
                
                   initial_time = self.res.Trajectory{number_trajectory}.initial_time(1);
                   relative_time = self.res.Trajectory{number_trajectory}.relative_time;
                   
                   x= self.res.Trajectory{number_trajectory}.x;
                   y = self.res.Trajectory{number_trajectory}.y;
                   theta= self.res.Trajectory{number_trajectory}.theta;
                   velocity = self.res.Trajectory{number_trajectory}.v;
                   a = self.res.Trajectory{number_trajectory}.a;
                   kappa = self.res.Trajectory{number_trajectory}.kappa;
                   dkappa = self.res.Trajectory{number_trajectory}.dkappa;
                   
                   idx_L = find(self.res.Localization.MeasurementTime>initial_time(1));
                   idx_matched_traj = idx_L(1);
                   loca_x = self.res.Localization.Pose.Position.x(idx_matched_traj);
                   loca_y = self.res.Localization.Pose.Position.y(idx_matched_traj);
                   loca_v = sqrt(self.res.Localization.Pose.LinearVelocity.x(idx_matched_traj)^2+...
                       self.res.Localization.Pose.LinearVelocity.y(idx_matched_traj)^2);
                   loca_theta = self.res.Localization.Pose.Heading(idx_matched_traj);
                   loca_a = self.res.Localization.Pose.LinearAcceleration.x(idx_matched_traj)/cos(loca_theta);
                   loca_time = self.res.Localization.MeasurementTime(idx_matched_traj);
               
               else
                   initial_time = self.res.Trajectory.timestamp_sec(number_trajectory);
                   nr_points = self.res.Trajectory.n_trajectory_point(number_trajectory);

                   loca_x = self.res.Localization.Pose.Position.x(number_trajectory);
                   loca_y = self.res.Localization.Pose.Position.y(number_trajectory);
                   loca_v = sqrt(self.res.Localization.Pose.LinearVelocity.x(number_trajectory)^2+...
                        self.res.Localization.Pose.LinearVelocity.y(number_trajectory)^2);
                   loca_theta = self.res.Localization.Pose.Heading(number_trajectory);
                   loca_a = self.res.Localization.Pose.LinearAcceleration.x(number_trajectory)/cos(loca_theta);
                   loca_time = self.res.Localization.MeasurementTime(number_trajectory);
                    
                   ref_time = ...
                       ((self.res.time_larger(number_trajectory) - self.res.time_smaller(number_trajectory)).*self.res.interp_factor(number_trajectory)) + ...
                       self.res.time_smaller(number_trajectory) + initial_time;
                   ref_x = self.res.ReferencePoint.x(number_trajectory);
                   ref_y = self.res.ReferencePoint.y(number_trajectory);
                   ref_v = self.res.ReferencePoint.v(number_trajectory);
                   ref_theta = self.res.ReferencePoint.theta(number_trajectory);
                   ref_kappa = self.res.ReferencePoint.kappa(number_trajectory);
                   ref_a = self.res.ReferencePoint.a(number_trajectory);
                   
                   
                   relative_time = zeros(1,nr_points);
                   x = zeros(1,nr_points);
                   y = zeros(1,nr_points);
                   theta = zeros(1,nr_points);
                   velocity = zeros(1,nr_points);
                   a = zeros(1,nr_points);
                   kappa = zeros(1, nr_points);
                   dkappa = zeros(1,nr_points);
                   for i=1:nr_points
                       number = ['i',num2str(i-1,'%02d')]; %or 02d if 100pts
                       relative_time(i) = self.res.Trajectory.relative_time.(number)(number_trajectory);
                       x(i) = self.res.Trajectory.x.(number)(number_trajectory);
                       y(i) = self.res.Trajectory.y.(number)(number_trajectory);
                       theta(i) = self.res.Trajectory.theta.(number)(number_trajectory);
                       velocity(i) = self.res.Trajectory.v.(number)(number_trajectory);
                       a(i) = self.res.Trajectory.a.(number)(number_trajectory);
                       kappa(i) = self.res.Trajectory.kappa.(number)(number_trajectory);
                       dkappa(i) = self.res.Trajectory.dkappa.(number)(number_trajectory);
                   end
               end

               absolute_time = relative_time+initial_time;
               [~, ideal_idx]=find(relative_time+initial_time >= self.res.Localization.MeasurementTime(number_trajectory), 1, 'first');
               
               figure(h1)
               subplot(4,1,1); hold on
               plot(absolute_time,relative_time);
               xlabel('absolute time')
               ylabel('relative time')
               
               subplot(4,1,2); hold on
               plot(absolute_time, x-x_offset)
               plot(ref_time, ref_x-x_offset, 'kx')
               plot(loca_time, loca_x-x_offset, 'ko')
               plot([loca_time, ref_time], [loca_x-x_offset, ref_x-x_offset], 'k-')
               ylabel('x')
               xlabel('absolute time')
               
               subplot(4,1,3); hold on
               plot(absolute_time, y-y_offset)
               plot(ref_time, ref_y-y_offset, 'kx')
               plot(loca_time, loca_y-y_offset, 'ko')
               plot([loca_time, ref_time], [loca_y-y_offset, ref_y-y_offset], 'k-')
               ylabel('y')
               xlabel('absolute time')
               
               subplot(4,1,4); hold on
               plot(absolute_time, wrapTo2Pi(theta))
               plot(ref_time, wrapTo2Pi(ref_theta), 'kx')
               plot(loca_time, wrapTo2Pi(loca_theta), 'ko')
               plot([loca_time, ref_time], [wrapTo2Pi(loca_theta), wrapTo2Pi(ref_theta)], 'k-')
               ylabel('theta')
               xlabel('absolute time')
               
               figure(h2)
               
               subplot(4,1,1); hold on
               plot(absolute_time, velocity)
               plot(ref_time, ref_v, 'kx')
               plot(loca_time, loca_v, 'ko')
               plot([loca_time, ref_time], [loca_v, ref_v], 'k-')
               ylabel('v')
               xlabel('absolute time')
               
               subplot(4,1,2); hold on
               plot(absolute_time, a)
               plot(ref_time, ref_a, 'kx')
               plot(loca_time, loca_a, 'ko')
               plot([loca_time, ref_time], [loca_a, ref_a], 'k-')
               ylabel('a')
               xlabel('absolute time')
               
               subplot(4,1,3); hold on
               plot(absolute_time, kappa)
               plot(ref_time, ref_kappa, 'kx')
               %compute loca kappa from loca?
               ss = self.res.VehicleDataCan.Steering.SteeringWheelAngleSign(number_trajectory);
               kappa_is = tan(self.res.VehicleDataCan.Steering.SteeringWheelAngle(number_trajectory)/852.7216)/2.786;
               kappa_is(ss==true) = -kappa_is(ss==true);
               plot(loca_time, kappa_is, 'ko')
               plot([loca_time, ref_time], [kappa_is, ref_kappa], 'k-')
               ylabel('kappa')
               xlabel('absolute time')
               
               subplot(4,1,4); hold on
               plot(absolute_time, dkappa)
               plot(absolute_time(ideal_idx), dkappa(ideal_idx), 'kx')
               ylabel('dkappa')
               xlabel('absolute time')
            end
        end
        
        %% Trajectory
        function plotTrajectoryTime(self)
           self.doNextPlot(); clf; hold on;
           set(gcf, 'Name', 'Trajectory time');
           subplot(211); hold on
           t = self.res.Trajectory.timestamp_sec(:);
           t(t==0) = nan;
           l=self.res.Localization.MeasurementTime(:);
           l(l<=0.001) = nan;
           plot(self.res.Time, t-l(1));    
           plot(self.res.Time, l-l(1)); 
           subplot(212); hold on
           plot(self.res.Time(2:end), diff(t));
           plot(self.res.Time(2:end), diff(l));
           legend('traj', 'loca')
           ylim([0,1])
        end
        
        %% Trajectory
        function plotSteeringAngleCan(self)
           self.doNextPlot(); clf; hold on;
           set(gcf, 'Name', 'Steering Angle Can');
           s = self.res.VehicleDataCan.Steering.SteeringWheelAngle;
           ss = self.res.VehicleDataCan.Steering.SteeringWheelAngleSign;
           s(ss==false) = -s(ss==false);
           plot(self.res.Time, s);        
        end
        
        function plotDiffSteeringAngleCan(self)
           self.doNextPlot(); clf; hold on;
           set(gcf, 'Name', 'Diff Steering Angle Can');
           steering_angle = diff(self.res.VehicleDataCan.Steering.SteeringWheelAngle);
           steering_angle_range_rad_to_steering_wheel_angle_range_deg_gain = 852.7216;
           control_command = diff(self.res.steering_angle_limited(self.active))*steering_angle_range_rad_to_steering_wheel_angle_range_deg_gain;
           control_command(control_command==0) = nan;
           
           plot(self.res.Time(1:end-1), steering_angle);
           plot(self.res.Time(1:end-1), control_command,'x-');
           plot(self.res.Time(self.active), self.res.controller_active(self.active));
           plot([self.res.Time(1), self.res.Time(end)], [2.6, 2.6], 'k-');
           plot([self.res.Time(1), self.res.Time(end)], [-2.6, -2.6], 'k-');
           ylim([-10, 10])
           legend('Diff Steering Angle', 'Diff Steering Control Command', 'controller active')
        end
        
        %% Tracking Error
        function plotTrackingError(self)
            self.doNextPlot(); clf; hold on;
           set(gcf, 'Name', 'Tracking Errors');
           subplot(411); hold on
           plot(self.res.Time(self.active), self.res.TrackingError.e_n(self.active));
           title('e_n')
           subplot(412); hold on
           plot(self.res.Time(self.active), self.res.TrackingError.e_t(self.active));
           title('e_t')
           subplot(413); hold on
           plot(self.res.Time(self.active), self.res.TrackingError.e_psi(self.active));
           title('e_psi')
           subplot(414); hold on
           plot(self.res.Time(self.active), self.res.TrackingError.e_v(self.active));
           title('e_v')
        end
        %% Matching idxs
        function plotMatchingIdxs(self)
            self.doNextPlot(); clf; hold on;
            plot(self.res.Time(self.active), self.res.idx_larger(self.active));
            plot(self.res.Time(self.active), self.res.idx_smaller(self.active));
            plot(self.res.Time(self.active), self.res.interp_factor(self.active));
            legend('idx lager', 'idx smaller', 'interpolation factor');
        end
        
        %% First move idx
        function idx = getFirstMoveIdx(self)
            movement = sqrt(diff(self.res.Localization.Pose.Position.x).^2 + diff(self.res.Localization.Pose.Position.y).^2);
            idx = find(movement > 0.02, 1, 'first');
            idx = idx - 20;
            if idx < 1
                idx = 1;
            end
        end
        
        function idx = getLastMoveIdx(self)
            movement = sqrt(diff(self.res.Localization.Pose.Position.x).^2 + diff(self.res.Localization.Pose.Position.y).^2);
            idx = find(movement > 0.01, 1, 'last');
            idx = idx + 20;
            if idx < 1
                idx = 1;
            end
        end
        
        %%Get index from time
        function idx = getIdxFromTime(self, t)
            idx = zeros(size(t));
            for i=1:length(t)
                idx(i) = find(self.res.Time <= t(i), 1, 'last');
            end
        end
		
        
    end
    
end


%% Customize Data Tips 

		% In order to get corresponing time values in an XY
		% plot we get the xy values from the plot and look them up in result data
		% structure. Afterwards we get the corresponing time indices and plot all
		% occurences (we have to distinguish between the ReferencePoint and the
		% Vehicle State values).
		function txt = local_datatipupdatefcn(~,event_obj,res)
			
			pos = get(event_obj,'Position');
			des = [res.ReferencePoint.x; res.ReferencePoint.y]';
			cur = [res.VehicleState.x; res.VehicleState.y]';
			ind_des = strmatch(pos,des);
			ind_cur = strmatch(pos,cur);
			
			
			txt = {['X = ', num2str(pos(1))], ['Y = ', num2str(pos(2))]};
			if ~isempty(ind_des)
				txt = {txt{:},['Hits on desired XY Pos:']};
				for i=1:length(ind_des)
					txt = {txt{:},['Time = ', num2str(res.Time(ind_des(i)))]};
				end
			end
			if ~isempty(ind_cur)
				txt = {txt{:},['Hits on current XY Pos:']};
				for i=1:length(ind_cur)
					txt = {txt{:},['Time = ', num2str(res.Time(ind_cur(i)))]};
				end
			end
			
        end
        
        % Match Time to ADTF Timestamp
        function txt = local_datatipupdateADTFfcn(~,event_obj,res)
			
			pos = get(event_obj,'Position');
            
			[~, ind] = min(abs(pos(1)-res.Time));
            ADTF_time = res.VehicleState.time_stamp(ind);
			
			txt = {['X = ', num2str(pos(1))], ['Y = ', num2str(pos(2))], ['ADTF Time = ', num2str(ADTF_time)]};
		end