close all
clear all
clc

%%
syms x y theta v a kappa jerk xi h

%% dx
fx = x+0.5*h*v*cos(theta)+0.5*h*(v+h*a)*cos(theta+h*v*kappa);
fx_x = diff(fx,x);
fx_y = diff(fx,y);
fx_theta = diff(fx,theta);
fx_v = diff(fx,v);
fx_a = diff(fx,a);
fx_kappa = diff(fx,kappa);
fx_jerk = diff(fx,jerk);
fx_xi = diff(fx,xi);

fx_x 
fx_y
fx_theta
fx_v
fx_a
fx_kappa
fx_jerk
fx_xi

%% dy
fy = y+0.5*h*v*sin(theta)+0.5*h*(v+h*a)*sin(theta+h*v*kappa);
fy_x = diff(fy,x);
fy_y = diff(fy,y);
fy_theta = diff(fy,theta);
fy_v = diff(fy,v);
fy_a = diff(fy,a);
fy_kappa = diff(fy,kappa);
fy_jerk = diff(fy,jerk);
fy_xi = diff(fy,xi);

fy_x 
fy_y
fy_theta
fy_v
fy_a
fy_kappa
fy_jerk
fy_xi

%% dtheta
ftheta = theta+0.5*h*v*kappa+0.5*h*(v+h*a)*(kappa+h*xi);
ftheta_x = diff(ftheta,x);
ftheta_y = diff(ftheta,y);
ftheta_theta = diff(ftheta,theta);
ftheta_v = diff(ftheta,v);
ftheta_a = diff(ftheta,a);
ftheta_kappa = diff(ftheta,kappa);
ftheta_jerk = diff(ftheta,jerk);
ftheta_xi = diff(ftheta,xi);

ftheta_x 
ftheta_y
ftheta_theta
ftheta_v
ftheta_a
ftheta_kappa
ftheta_jerk
ftheta_xi


%% dv
fv = v+0.5*h*a+0.5*h*(a+h*jerk);
fv_x = diff(fv,x);
fv_y = diff(fv,y);
fv_theta = diff(fv,theta);
fv_v = diff(fv,v);
fv_a = diff(fv,a);
fv_kappa = diff(fv,kappa);
fv_jerk = diff(fv,jerk);
fv_xi = diff(fv,xi);

fv_x 
fv_y
fv_theta
fv_v
fv_a
fv_kappa
fv_jerk
fv_xi