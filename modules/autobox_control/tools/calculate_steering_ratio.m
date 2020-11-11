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

function [] = calculate_steering_ratio()

    close all
    
    folder = '\\fs01\kf\AS\# fortiss car\recordings\20190911\Lenkradwinkelkennlinie';
    files = dir([folder, '\', '*.mat']);
    
    delta = zeros(1, length(files));
    steer = zeros(1, length(files)); 
    v = zeros(1, length(files));
    for i=1:length(files)
        file = fullfile(folder, files(i).name);
        [delta(i), steer(i), v(i)] = fit_one_drive(file);
    end
    
    f = fittype('a*x'); 
    [fit1, gof, fitinfo] = fit([0, delta]', [0, steer]', f, 'StartPoint', 900);
    ratio = fit1.a;
    
    %coefficients = polyfit([0, delta], [0, steer], 1);
    %ratio2 = coefficients(1);
    
    figure;
    title('steering ratio')
    hold on
    grid on
    plot([0, delta], [0, steer], 'bo')
    plot([0,0.5],[0,ratio*0.5],'g-')
    legend('data points', 'fit')
    
    disp(['Fitted Gain is: ', num2str(ratio)])
    
    
end

function [delta, meansteer, meanv] = fit_one_drive(file)
    
    data = load(file);
    content = fieldnames(data);
    content = content{1,1};
    data = data.(content);
    
    % get data from file
    xrow = find(~cellfun(@isempty,strfind({data.Y.Name},'.x')));
    yrow = find(~cellfun(@isempty,strfind({data.Y.Name},'.y')));
    steerrow_tmp = find(~cellfun(@isempty,strfind({data.Y.Name},'.SteeringWheelAngle')));
    for i=1:length(steerrow_tmp)
        if isa(data.Y(steerrow_tmp(i)).Data,'double')
            steerrow = steerrow_tmp(i);
        elseif isa(data.Y(steerrow_tmp(i)).Data,'uint8')
            steersignrow = steerrow_tmp(i);
        end
    end
    x = data.Y(xrow).Data - data.Y(xrow).Data(1);
    y = data.Y(yrow).Data - data.Y(yrow).Data(2);
    steer = data.Y(steerrow).Data / 180 *pi;
    dt = data.X.Data(3) - data.X.Data(2);
    
    % interpolate circle
    Par = PrattSVD([x;y]');
    lambda_circ = @(m, r) [m(1)+r*cos(0:0.1:2*pi);m(2)+r*sin(0:0.1:2*pi)];
    circle_sampled = lambda_circ([Par(1), Par(2)], Par(3));
    
    % calc steering angle
    l = 2.786;
    r = Par(3);
    kappa = 1/r;
    %delta = atan(l/r);
    delta = atan2(l,r);
    
    % mean steering:
    meansteer = mean(steer);
    minsteer = min(steer);
    maxsteer = max(steer);
    
    % speed
    v = sqrt(diff(x).^2 + diff(y).^2)/dt;
    meanv = mean(v);
    
    % ratio
    steering_ratio = meansteer/delta;
    
    % plot
    figure;
    hold on;
    axis equal;
    title(content)
    plot(x,y,'bx-')
    plot(circle_sampled(1,:),circle_sampled(2,:),'r-')
    disp(['Steering angle = ', num2str(delta),...
        ' with Steering WHEEL angle = ', num2str(meansteer),...
        ' yields ratio = ', num2str(steering_ratio),...
        ' at speed = ', num2str(meanv),...
        ' with radius r = ', num2str(r)]);
    

end


% https://people.cas.uab.edu/~mosya/cl/MATLABcircle.html
function Par = PrattSVD(XY)

%--------------------------------------------------------------------------
%  
%     Algebraic circle fit by Pratt
%      V. Pratt, "Direct least-squares fitting of algebraic surfaces",
%      Computer Graphics, Vol. 21, pages 145-152 (1987)
%
%     Input:  XY(n,2) is the array of coordinates of n points x(i)=XY(i,1), y(i)=XY(i,2)
%
%     Output: Par = [a b R] is the fitting circle:
%                           center (a,b) and radius R
%
%     Note: this is a version optimized for stability, not for speed
%
%--------------------------------------------------------------------------

centroid = mean(XY);   % the centroid of the data set

[U,S,V]=svd([(XY(:,1)-centroid(1)).^2+(XY(:,2)-centroid(2)).^2,...
        XY(:,1)-centroid(1), XY(:,2)-centroid(2), ones(size(XY,1),1)],0);

if (S(4,4)/S(1,1) < 1e-12)   %  singular case
    A = V(:,4);
    disp('Pratt singular case');
else                         %  regular case
    W=V*S;
    Binv = [0 0 0 -0.5; 0 1 0 0; 0 0 1 0; -0.5 0 0 0];
    [E,D] = eig(W'*Binv*W);
    [Dsort,ID] = sort(diag(D));
    A = E(:,ID(2));
    for i=1:4
        S(i,i)=1/S(i,i); 
    end
    A = V*S*A;
end

Par = -(A(2:3))'/A(1)/2 + centroid;
Par = [Par , sqrt(A(2)^2+A(3)^2-4*A(1)*A(4))/abs(A(1))/2];

end   %  PrattSVD