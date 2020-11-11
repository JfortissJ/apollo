if exist('AutoboxModel_rti1401','dir') ~= 0
    eval('rmdir ''AutoboxModel_rti1401'' s')
end

delete AutoboxModel.sdf
delete AutoboxModel.dsbuildinfo
delete AutoboxModel.map
delete AutoboxModel.ppc
delete AutoboxModel.ppc.hex
delete AutoboxModel.ppc.srec
delete AutoboxModel.slxc
delete AutoboxModel.trc
delete AutoboxModel.trz
delete AutoboxModel_usr.c

model='AutoboxModel.slx';
load_system(model)

rtwbuild('AutoboxModel');
