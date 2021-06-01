close all
clc
clear all

x_k = [0, 0, 0, 1, 0, 0];
u_k = [0.2, 0.5];


t_end = 5;
h=[0.001, 0.01, 0.1, 1];
for j = 1:length(h)
    
x_k1H{j} = x_k';
x_k1E{j} = x_k';
t_vec{j} = 0;
for i=1:t_end/h(j)
    x_k1H{j}(:,i+1) = IntegrateModelHeun(x_k1H{j}(:,i), u_k, h(j))';
    x_k1E{j}(:,i+1) = IntegrateEuler(x_k1E{j}(:,i), u_k, h(j))';
    t_vec{j}(i+1) = t_vec{j}(i) + h(j);
end
end

figure
subplot(3,2,1)
hold on
for j = 1:length(h)
    if h(j) == 1e-3
        plot(t_vec{j}, x_k1E{j}(1,:), 'DisplayName',['Euler ts=', num2str(h(j))])
    end
    plot(t_vec{j}, x_k1H{j}(1,:), 'DisplayName',['Heun ts=', num2str(h(j))])
end
ylabel('x')
legend('Location', 'northwest')

subplot(3,2,2)
hold on
for j = 1:length(h)
    if h(j) == 1e-3
        plot(t_vec{j}, x_k1E{j}(2,:), 'DisplayName',['Euler ts=', num2str(h(j))])
    end
    plot(t_vec{j}, x_k1H{j}(2,:), 'DisplayName',['Heun ts=', num2str(h(j))])
end
ylabel('y')
legend('Location', 'northwest')

subplot(3,2,3)
hold on
for j = 1:length(h)
    if h(j) == 1e-3
        plot(t_vec{j}, x_k1E{j}(3,:), 'DisplayName',['Euler ts=', num2str(h(j))])
    end
    plot(t_vec{j}, x_k1H{j}(3,:), 'DisplayName',['Heun ts=', num2str(h(j))])
end
ylabel('theta')
legend('Location', 'northwest')

subplot(3,2,4)
hold on
for j = 1:length(h)
    if h(j) == 1e-3
        plot(t_vec{j}, x_k1E{j}(4,:), 'DisplayName',['Euler ts=', num2str(h(j))])
    end
    plot(t_vec{j}, x_k1H{j}(4,:), 'DisplayName',['Heun ts=', num2str(h(j))])
end
ylabel('v')
legend('Location', 'northwest')

subplot(3,2,5)
hold on
for j = 1:length(h)
    if h(j) == 1e-3
        plot(t_vec{j}, x_k1E{j}(5,:), 'DisplayName',['Euler ts=', num2str(h(j))])
    end
    plot(t_vec{j}, x_k1H{j}(5,:), 'DisplayName',['Heun ts=', num2str(h(j))])
end
ylabel('a')
legend('Location', 'northwest')

subplot(3,2,6)
hold on
for j = 1:length(h)
    if h(j) == 1e-3
        plot(t_vec{j}, x_k1E{j}(6,:), 'DisplayName',['Euler ts=', num2str(h(j))])
    end
    plot(t_vec{j}, x_k1H{j}(6,:), 'DisplayName',['Heun ts=', num2str(h(j))])
end
ylabel('kappa')
legend('Location', 'northwest')

%%

figure
subplot(2,1,1)
hold on
for j = 1:length(h)
    plot(t_vec{j}, x_k1H{j}(1,:), 'DisplayName',['Heun ts=', num2str(h(j))])
end
ylabel('x')
legend('Location', 'northwest')

subplot(2,1,2)
hold on
for j = 1:length(h)
    plot(t_vec{j}, x_k1H{j}(4,:), 'DisplayName',['Heun ts=', num2str(h(j))])
end
ylabel('v')
legend('Location', 'northwest')
%%

figure
subplot(2,1,1)
hold on
for j = 1:length(h)
    plot(t_vec{j}, x_k1E{j}(1,:), 'DisplayName',['Euler ts=', num2str(h(j))])
end
ylabel('x')
legend('Location', 'northwest')

subplot(2,1,2)
hold on
for j = 1:length(h)
    plot(t_vec{j}, x_k1E{j}(4,:), 'DisplayName',['Euler ts=', num2str(h(j))])
end
ylabel('v')
legend('Location', 'northwest')

%%

function [x_k1] = IntegrateModelHeun(x_k, u_k, h)
theta_k = x_k(3);
v_k = x_k(4);
acc_k = x_k(5);
kappa_k = x_k(6);

x_k1P = x_k + h*[v_k*cos(theta_k);
    v_k*sin(theta_k);
    v_k*kappa_k;
    acc_k;
    u_k(1);
    u_k(2);
    ];
v__ = v_k + h*acc_k;
theta__ = theta_k + h*v_k*kappa_k;
kappa__ =  kappa_k + h*u_k(2);
acc__ = acc_k + h*u_k(1);
f_xk1P = [v__*cos(theta__); v__*sin(theta__); v__*kappa__; acc__; u_k(1); u_k(2)];
x_k1 = 0.5*x_k + 0.5*(x_k1P+h*f_xk1P);
end

function [x_k1] = IntegrateEuler(x_k, u_k, h)
theta_k = x_k(3);
v_k = x_k(4);
acc_k = x_k(5);
kappa_k = x_k(6);

x_k1 = x_k + h*[v_k*cos(theta_k);
    v_k*sin(theta_k);
    v_k*kappa_k;
    acc_k;
    u_k(1);
    u_k(2);
    ];

end