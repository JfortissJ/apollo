clc
% close all
clear all

rootpath = 'miqp_testdata'
input_file = 'test_trajectory_miqp_20210429-135518.pb.txt'
subsampling = 1
miqp_traj = ReadTrajectory(rootpath, input_file)
sqp_traj = ReadTrajectory(rootpath, ['sqp_out_', num2str(subsampling), '_', input_file])

figure
subplot(4,1,1)
hold on
plot(miqp_traj.relative_time, miqp_traj.da, 'o-.')
plot(sqp_traj.relative_time, sqp_traj.da, 'x-')
legend('miqp_traj', 'sqp_traj')
ylabel('jerk')

subplot(4,1,2)
hold on
plot(miqp_traj.relative_time, miqp_traj.a, 'o-.')
plot(sqp_traj.relative_time, sqp_traj.a, 'x-')
legend('miqp', 'sqp')
ylabel('a')

subplot(4,1,3)
hold on
plot(miqp_traj.relative_time, miqp_traj.v, 'o-.')
plot(sqp_traj.relative_time, sqp_traj.v, 'x-')
legend('miqp', 'sqp')
ylabel('v')

subplot(4,1,4)
hold on
plot(miqp_traj.relative_time, miqp_traj.dkappa, 'o-.')
plot(sqp_traj.relative_time, sqp_traj.dkappa, 'x-')
legend('miqp', 'sqp')
ylabel('dkappa')

figure
subplot(4,1,1)
hold on
plot(miqp_traj.relative_time, miqp_traj.x - miqp_traj.x(1), 'o-.')
plot(sqp_traj.relative_time, sqp_traj.x - sqp_traj.x(1), 'x-')
legend('miqp', 'sqp')
ylabel('x')

subplot(4,1,2)
hold on
plot(miqp_traj.relative_time, miqp_traj.y - miqp_traj.y(1), 'o-.')
plot(sqp_traj.relative_time, sqp_traj.y - sqp_traj.y(1), 'x-')
legend('miqp', 'sqp')
ylabel('y')

subplot(4,1,3)
hold on
plot(miqp_traj.relative_time, miqp_traj.theta, 'o-.')
plot(sqp_traj.relative_time, sqp_traj.theta, 'x-')
legend('miqp', 'sqp')
ylabel('theta')

subplot(4,1,4)
hold on
plot(miqp_traj.relative_time, miqp_traj.kappa, 'o-.')
plot(sqp_traj.relative_time, sqp_traj.kappa)
legend('miqp', 'sqp')
ylabel('kappa')

figure
plot(miqp_traj.x - miqp_traj.x(1), miqp_traj.y - miqp_traj.y(1), 'o-.')
hold on
plot(sqp_traj.x - sqp_traj.x(1), sqp_traj.y - sqp_traj.y(1), 'x-')
axis equal

%%
function [t] = ReadTrajectory(rootpath, filename)
parameterstring = fileread(fullfile(rootpath, filename));

x = [];
y = [];
z = [];
theta = [];
kappa = [];
s = [];
dkappa = [];
v = [];
a = [];
da = [];
relative_time = [];

%#parameterstring(parameterstring == char(13)) = ''; % delete all carriage return
%parameterstring(parameterstring == char(10)) = ''; % delete all newlines

parameterstring = replaceBetween(parameterstring,"header ","trajectory_point","");
parameterstring=regexprep(parameterstring,'header trajectory_point {','');

parameterstring=regexprep(parameterstring,'trajectory_point {','');
parameterstring=regexprep(parameterstring,'path_point {','');
parameterstring=regexprep(parameterstring,'}','');
parameterstring = strrep(parameterstring, 'x: ','x = [x,');
parameterstring = strrep(parameterstring, 'y: ','y = [y,');
parameterstring = strrep(parameterstring, 'z: ','z = [z,');
parameterstring = strrep(parameterstring, 'theta: ','theta = [theta,');
parameterstring = strrep(parameterstring, 'dkappa: ','dkappa = [dkappa,');
parameterstring = strrep(parameterstring, 'kappa: ','kappa = [kappa,');
parameterstring = strrep(parameterstring, 's: ','s = [s,');
parameterstring = strrep(parameterstring, 'v: ','v= [v,');
parameterstring = strrep(parameterstring, 'da: ','da= [da,');
parameterstring = strrep(parameterstring, 'a: ','a= [a,');
parameterstring = strrep(parameterstring, 'relative_time: ','relative_time= [relative_time,');

parameterstring = regexprep(parameterstring,'  *',''); % delete multiple spaces
parameterstring = regexprep(parameterstring, '\n\n+', '\n'); % remove empty lines
parameterstring = strrep(parameterstring, char(10),['];',char(10)]); % append semicolon at end of line


parameterstring = parameterstring(4:end); % remove first three elements (first line)
%parameterstring = [parameterstring, '];'];

try
    eval(parameterstring);
catch
    x=nan;
    y=nan;
    theta = nan;
    kappa = nan;
    s=nan;
    dkappa=nan;
    v=nan;
    a=nan;
    relative_time=nan;
end

if isempty(da)
    da = nan*relative_time;
end
if isempty(dkappa)
    dkappa = nan*relative_time;
end

t.x = x
t.y = y
t.theta = theta
t.kappa = kappa
t.s = s
t.dkappa = dkappa
t.v = v
t.a = a
t.da = da
t.relative_time = relative_time

end