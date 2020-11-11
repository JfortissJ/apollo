% a skript to rename the input signals to outport names as only the signals
% can be logged in control desk.

sys = {'Lib_DeserializationTrajectory/DeserializationTrajectory/DeserializeTrajectoryToSignals',...
    'Lib_DeserializationLocalization/DeserializationLocalization/DeserializeLocaToSignal'};
for j=1:length(sys)
    all_outs = find_system(sys{j},'blocktype','Outport');
    for i=1:length(all_outs)
       name = get_param(all_outs{i},'name');
       ph = get_param(all_outs{i},'LineHandles');
       line = ph.Inport;
       set_param(line,'name',name);
    end
end