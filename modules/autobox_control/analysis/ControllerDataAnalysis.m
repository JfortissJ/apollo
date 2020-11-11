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
		
		
		%% Acc Steer
        function plotAccSteer(self)
           self.doNextPlot();
           clf; hold on;
           set(gcf, 'Name', 'Acc + Steering');
           
           subplot(2,1,1);
           hold on
           title('acc');
           acc = sqrt(self.res.Localization.Pose.LinearAcceleration.x.^2+self.res.Localization.Pose.LinearAcceleration.y.^2);
           plot(self.res.Time(self.active), acc(self.active));
%            plot(self.res.Time(self.active), self.res.CustomerCanData.Acceleration_Interface.AccelerationRequest(self.active));
           
           subplot(2,1,2);
           hold on
           title('steering')
           plot(self.res.Time(self.active), self.res.CustomerCanData.Steering_Interface.SteeringAngleRequest(self.active));
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
           %plot(self.res.Time(self.active), v);
           plot(v);
           
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
        
        %% Trajectory
        function plotTrajectoryRange(self, number_trajectory_range)
           self.doNextPlot(); clf; hold on;
           set(gcf, 'Name', 'Trajectory1');
           h1 = gcf;
           
           self.doNextPlot(); clf; hold on;
           set(gcf, 'Name', 'Trajectory2');
           h2 = gcf;

%            number_trajectory = 200; % i.e. first trajectory that get's published
            for number_trajectory = number_trajectory_range
               initial_time = self.res.Trajectory.timestamp_sec(number_trajectory);
               nr_points = self.res.Trajectory.n_trajectory_point(number_trajectory);


               relative_time = zeros(1,nr_points);
               x = zeros(1,nr_points);
               y = zeros(1,nr_points);
               theta = zeros(1,nr_points);
               velocity = zeros(1,nr_points);
               a = zeros(1,nr_points);
               kappa = zeros(1, nr_points);
               dkappa = zeros(1,nr_points);
               for i=1:nr_points
                   number = ['i',num2str(i-1,'%03d')];
                   relative_time(i) = self.res.Trajectory.relative_time.(number)(number_trajectory);
                   x(i) = self.res.Trajectory.x.(number)(number_trajectory);
                   y(i) = self.res.Trajectory.y.(number)(number_trajectory);
                   theta(i) = self.res.Trajectory.theta.(number)(number_trajectory);
                   velocity(i) = self.res.Trajectory.v.(number)(number_trajectory);
                   a(i) = self.res.Trajectory.a.(number)(number_trajectory);
                   kappa(i) = self.res.Trajectory.kappa.(number)(number_trajectory);
                   dkappa(i) = self.res.Trajectory.dkappa.(number)(number_trajectory);
               end
               
               absolute_time = relative_time+initial_time;
               [~, ideal_idx]=find(relative_time+initial_time >= self.res.Localization.MeasurementTime(number_trajectory), 1, 'first');
               
               figure(h1)
               subplot(4,1,1); hold on
               plot(absolute_time,relative_time);
               plot(absolute_time(ideal_idx), relative_time(ideal_idx), 'kx')
               xlabel('abslute time')
               ylabel('relative time')
               
               subplot(4,1,2); hold on
               plot(absolute_time, x)
               plot(absolute_time(ideal_idx), x(ideal_idx), 'kx')
               ylabel('x')
               xlabel('abslute time')
               
               subplot(4,1,3); hold on
               plot(absolute_time, y)
               plot(absolute_time(ideal_idx), y(ideal_idx), 'kx')
               ylabel('y')
               xlabel('abslute time')
               
               subplot(4,1,4); hold on
               plot(absolute_time, wrapTo2Pi(theta))
               plot(absolute_time(ideal_idx), wrapTo2Pi(theta(ideal_idx)), 'kx')
               ylabel('theta')
               xlabel('abslute time')
               
               figure(h2)
               
               subplot(4,1,1); hold on
               plot(absolute_time, velocity)
               plot(absolute_time(ideal_idx), velocity(ideal_idx), 'kx')
               ylabel('v')
               xlabel('abslute time')
               
               subplot(4,1,2); hold on
               plot(absolute_time, a)
               plot(absolute_time(ideal_idx), a(ideal_idx), 'kx')
               ylabel('a')
               xlabel('abslute time')
               
               subplot(4,1,3); hold on
               plot(absolute_time, kappa)
               plot(absolute_time(ideal_idx), kappa(ideal_idx), 'kx')
               ylabel('kappa')
               xlabel('abslute time')
               
               subplot(4,1,4); hold on
               plot(absolute_time, dkappa)
               plot(absolute_time(ideal_idx), dkappa(ideal_idx), 'kx')
               ylabel('dkappa')
               xlabel('abslute time')
            end
        end
        
        %% Trajectory
        function plotTrajectoryTime(self)
           self.doNextPlot(); clf; hold on;
           set(gcf, 'Name', 'Trajectory time');
           subplot(211); hold on
           t = self.res.Trajectory.timestamp_sec(:);
           t(t==0) = nan;
           plot(self.res.Time(:), t);    
           l=self.res.Localization.MeasurementTime(:);
           l(l<=0.001) = nan;
           plot(self.res.Time, l); 
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