% TODO die vx,vy,vz geschwindigkeit in der odometrie ergeben keinen sinn
% für mich...

%%
localization = readmatrix('localization.csv');
odometry = readmatrix('odometry.csv');
imu = readmatrix('imu.csv');

%%
close all

%%
t = localization(:,1);
x = localization(:,2);
y = localization(:,3);
vx = localization(:,5);
vy = localization(:,6);
ax = localization(:,8);
ay = localization(:,9);
az = localization(:,10);
qx = localization(:,11);
qy = localization(:,12);
qz = localization(:,13);
qw = localization(:,14);
h = localization(:,15);
vrotx = localization(:,16);
vroty = localization(:,17);
vrotz = localization(:,18);

dt = diff(t);
dx = diff(x);
dy = diff(y);
dvx = diff(vx);
dvy = diff(vy);
ddx = diff(dx);
ddy = diff(dy);

%%

t_i = imu(:,1);
ax_i = imu(:,2);
ay_i = imu(:,3);
az_i = imu(:,4);
vrotx_i = imu(:,5);
vroty_i = imu(:,6);
vrotz_i = imu(:,7);
dt_i = diff(t);
roll_i = imu(:,8);
pitch_i = imu(:,9);
yaw_i = imu(:,10);

%%

t_o = odometry(:,1);
x_o = odometry(:,2);
y_o = odometry(:,3);
vx_o = odometry(:,8);
vy_o = odometry(:,9);
vz_o = odometry(:,10);
qx_o = odometry(:,4);
qy_o = odometry(:,5);
qz_o = odometry(:,6);
qw_o = odometry(:,7);

dt_o = diff(t_o);
dx_o = diff(x_o);
dy_o = diff(y_o);
dvx_o = diff(vx_o);
dvy_o = diff(vy_o);
ddx_o = diff(dx_o);
ddy_o = diff(dy_o);




%%
lever_static = [0.018; 0.123; -0.8725];
arotx = [diff(vrotx)./dt; 0];
aroty = [diff(vroty)./dt; 0];
arotz = [diff(vrotz)./dt; 0];
l = min(length(t), length(t_i));
ax_rot = zeros(l,1);
ay_rot = zeros(l,1);
az_rot = zeros(l,1);
eul = zeros(l,3);
rotm_ = cell(l);
for i=1:l
    %rotm = quat2rotm([qx(i), qy(i), qz(i), qw(i)]);
     eul(i,:) = quat2eul([qx(i), qy(i), qz(i), qw(i)]);
     rotm_{i} = eul2rotm(eul(i,:), 'XYZ');
%     rotm = rotm_{i};
    rotm = eul2rotm([h(i), 0, 0]); %GEHT!!!!!!!
    a = [ax_i(i); -ay_i(i); az_i(i)]; %GEHT!!!!!!! mit -
    a_trans = rotm*a;
    
    %a_trans = cross([arotx_i(i); aroty_i(i); arotz_i(i)], lever);
    
    %lever = rotm*lever_static;
    
    %alpha = 1/norm(lever)*cross(lever, [ax_i(i); ay_i(i); az_i(i)]);
    %a_trans = cross(alpha, lever);
    
    %a_trans = cross([arotx(i); aroty(i); arotz(i)], lever);
    
    ax_rot(i) = a_trans(1);
    ay_rot(i) = a_trans(2);
    az_rot(i) = a_trans(3); 
end


%%
figure
hold on
title('vx')
plot(t(1:end-1), dx./dt)
plot(t, vx)

figure
hold on
title('vy')
plot(t(1:end-1), dy./dt)
plot(t, vy)


%%
figure
hold on
title('ax')
plot(medfilt1(ddx./dt(2:end)./dt(2:end)))
plot(medfilt1(ax))
plot(medfilt1(dvx./dt))
%plot(medfilt1(ax_rot))
legend('ddx/dt2', 'ax loca', 'dv/dt')
ylim([-5,5])

figure
hold on
title('ay')
plot(medfilt1(ddy./dt(2:end)./dt(2:end)))
plot(medfilt1(ay))
plot(medfilt1(dvy./dt))
%plot(medfilt1(ay_rot))
legend('ddy/dt2', 'ay_loca', 'dv/dt')
ylim([-5,5])

figure
hold on
title('az')
plot(az_i)
plot(az_rot)

%%
figure
axis equal
hold on
x0 = x(1);
y0 = y(1);
plot(x-x0,y-y0)
x1 = 1*cos(h);
y1 = 1*sin(h);
r = 1:50:length(x);
quiver(x(r)-x0, y(r)-y0, x1(r), y1(r))

%%
figure
hold on
plot(t-t(1),ax)
plot(t_i-t_i(1),ax_i,'LineWidth',2)
plot(t-t(1),ay)
plot(t_i-t_i(1),ay_i)
%plot(t-t(1),az)
%plot(t_i-t_i(1),az_i)
v_abs = sqrt(vx.^2+vy.^2);
plot(t-t(1), v_abs)
legend('loca ax', 'imu ax', 'loca ay', 'imu ay', 'vabs')

%%
figure
hold on
plot(t_i-t_i(1), vrotx_i)
plot(t_i-t_i(1), vroty_i)
plot(t_i-t_i(1), vrotz_i)
legend('x','y','z')

%%
figure
hold on
plot(roll_i)
plot(pitch_i)
plot(yaw_i)
plot(h-pi/2)
legend('roll', 'pitch', 'yaw', 'loca heading')


