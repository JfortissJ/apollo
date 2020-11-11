%%
ed = load('ethernetdump_001.mat');


%%
time = ed.ethernetdump_001.X.Data';

numtraj = 50000;
numloca = 50000;
numtime = length(time);

trajectorydump = zeros(numtime, numtraj);
locadump = zeros(numtime, numloca);

for i=1:length(ed.ethernetdump_001.Y)
    
    field = ed.ethernetdump_001.Y(i);
    fieldidx = str2double(field.Name(13:17)) + 1;
    
    
    if isempty(strfind(field.Path,'Localization'))
        %Trajectory
        trajectorydump(:,fieldidx) = field.Data;
    else
        %Localization
        locadump(:,fieldidx) = field.Data;
    end
    
end

%%
%csvwrite('traj.csv',trajectorydump);
%csvwrite('loca.csv',locadump);


%%
trajectorydump = unique(trajectorydump(:,1:10000),'rows');
locadump = unique(locadump(:,1:200),'rows');


