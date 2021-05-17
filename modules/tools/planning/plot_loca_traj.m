%clear all
clc
close all

loca = load('loca.txt');
traj = load('traj.txt');

figure
hold on
axis equal
plot(loca(:,1), loca(:,2),':x')

timesteps = unique(traj(:,1));
for i = 1:length(timesteps)
    idxs = find(traj(:,1) == timesteps(i));
    reltime = traj(idxs,2);
    zerotime = find(reltime > 0);
    zerotime = zerotime(1) + min(idxs);
    plot(traj(idxs,3),traj(idxs,4));
    plot(traj(idxs(end)-19,3),traj(idxs(end)-19,4),'k*');
    plot(traj(zerotime,3),traj(zerotime,4),'ko');
end
