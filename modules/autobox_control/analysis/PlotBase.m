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

classdef PlotBase < handle
	
	%% Properties
	properties (Access = private)
		allPlotHandles; %A list of all plot handles this instance has created; used to avoid overwriting plots and to distroy the correct plots
		plotStructName; %Name of the actual data struct. For future: All functions operating on the actual struct CAN be generic/inherited (like the disp function)
        plotNameHandles;
	end
	
	%% Methods
	methods
		
		%% Constructor
		function self = PlotBase()
			self.allPlotHandles = [];
		end
		
		%% Destructor
		function delete(self)
			self.allPlotHandles = intersect(self.allPlotHandles, findobj('Type','figure')); %all figures are already closed
			close(self.allPlotHandles);
			self.allPlotHandles = [];
		end
		
		%% Setter for plotStructName
		function setPlotStructName(self, name)
			self.plotStructName = name;
        end
        
        %%
        function allPlotsInTabs(self)
            %figs2tabs(self.allPlotHandles);
        end
		
		%% Execute all plot functions
		function doAllPlots(self)
			
			allMethods = methods(self);
			isPlotMethod = strfind(allMethods,'plot');
			
			%TODO das geht sicher auch ohne Schleife!
			for i=1:length(allMethods)
				if ~isempty(isPlotMethod{i})
                    try
                        disp(['Executing plot function ', allMethods{i}]);
                        self.(allMethods{i});
                    catch ex
                       warning(['Error execution plot function ', allMethods{i}, ': ' ex.message]); 
                    end
				end
			end
			
		end
		
		%% Disp signal list
		function dispSignalsList(self)
			disp(self.res.SignalsList);
		end
		
		%% Print some infos on the command line
		function dispInfo(self)
			
			% Loop over all available structs
			for j=1:length(self.plotStructName)
				
				disp(['Object of type ', class(self),' variable ', self.plotStructName{j}, ' holds the following data: '])
				fields = fieldnames(self.(self.plotStructName{j}));
				
				%fields_to_disp = {'Filename', 'Comment', 'RecordDate', 'RecordTime'};
				
				% Loop over all char fields
				for i=1:length(fields)
					if ischar(self.(self.plotStructName{j}).(fields{i}))
						disp([fields{i}, ': ', self.(self.plotStructName{j}).(fields{i})]);
					end
				end
				
			end
			
        end
        
        function linkAllXAxes(self)
            axes = [];
            for handle = self.allPlotHandles
                if (ishandle(handle))
                    axes = [axes; findall(handle, 'type','axes')];
                end
            end
            linkaxes(axes,'x');
        end
		
	end
	
	methods (Access = protected)
		
		%% Select a next plot handle and add it to the index
		function n = getNextPlotNumber(self)
			
			current = findobj('Type','figure');
			if isempty(current)
				n = 1;
            else
                v = version('-release');

				if (strcmp(v,'2014b')||strcmp(v,'2016a')||strcmp(v,'2013b'))
                    n = max(current.Number) + 1; %TODO intelligentere logik
                elseif(strcmp(v,'2017b')||strcmp(v,'2019b')||strcmp(v,'2020a'))
                    n = length(current) + 1;
                else
					n = max(current)+1;
                end
			end
			self.allPlotHandles = [self.allPlotHandles, n];
			
		end
		
		%% open figure and maximize, return figure handle
		function h = doNextPlot(self, figure_title)
			stack = dbstack;
            names = strsplit(stack(2).name,'.');
            name = strrep(names{2},'plot','');
            %if (~isfield(self.plotNameHandles,name) || ~isvalid(self.plotNameHandles.(name)))
                h=figure(self.getNextPlotNumber());
                self.plotNameHandles.(name) = h;
            %else
          %      h = self.plotNameHandles.(name);
          %      figure(h);
           %     clf;
            %end
			
            if (nargin < 2)
                set(h,'Name',name)
            else
                set(h,'Name',figure_title)
            end
			set(h,'units','normalized');
			set(h,'outerposition',[0 0 1 1])
			
        end
        
        
		
	end
	
	
end