clc
close all
clear all

rootpath = 'miqp_testdata'
miqp_traj = ReadTrajectory(rootpath, 'test_trajectory_miqp.pb.txt')
sqp_traj = ReadTrajectory(rootpath, 'sqp_traj_out.pb.txt')

figure
subplot(4,1,1)
hold on
plot(miqp_traj.relative_time, miqp_traj.da, '-.')
plot(sqp_traj.relative_time, sqp_traj.da)
legend('miqp_traj', 'sqp_traj')
ylabel('jerk')

subplot(4,1,2)
hold on
plot(miqp_traj.relative_time, miqp_traj.a, '-.')
plot(sqp_traj.relative_time, sqp_traj.a)
legend('miqp_traj', 'sqp_traj')
ylabel('a')

subplot(4,1,3)
hold on
plot(miqp_traj.relative_time, miqp_traj.v, '-.')
plot(sqp_traj.relative_time, sqp_traj.v)
legend('miqp_traj', 'sqp_traj')
ylabel('v')

subplot(4,1,4)
hold on
plot(miqp_traj.relative_time, miqp_traj.s, '-.')
plot(sqp_traj.relative_time, sqp_traj.s)
legend('miqp_traj', 'sqp_traj')
ylabel('s')

figure
subplot(3,1,1)
hold on
plot(miqp_traj.relative_time, miqp_traj.x, '-.')
plot(sqp_traj.relative_time, sqp_traj.x)
legend('miqp_traj', 'sqp_traj')
ylabel('x')

subplot(3,1,2)
hold on
plot(miqp_traj.relative_time, miqp_traj.y, '-.')
plot(sqp_traj.relative_time, sqp_traj.y)
legend('miqp_traj', 'sqp_traj')
ylabel('y')

subplot(3,1,3)
hold on
plot(miqp_traj.relative_time, miqp_traj.theta, '-.')
plot(sqp_traj.relative_time, sqp_traj.theta)
legend('miqp_traj', 'sqp_traj')
ylabel('theta')

%%
function [t] = ReadTrajectory(rootpath, filename)
parameterstring = fileread(fullfile(rootpath, filename));

x = [];
y = [];
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