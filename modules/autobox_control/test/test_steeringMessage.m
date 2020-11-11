% clc
clear all
close all

%% Test Data (input as decimal)
TestData.ActivationRequest = [0, 0, 0, 0, 0, 0, 0, 0];
TestData.AdvancedSteeringAngleLimitation = [0, 0, 0, 0, 0, 0, 0, 0];
TestData.CancelRequest = [0, 0, 0, 0, 0, 0, 0, 0];
TestData.ClearanceSI = [0, 0, 0, 1, 1, 1, 1, 0];
TestData.Counter = [10, 6, 2, 4, 13, 3, 7, 3];
TestData.CRC = [223, 203, 207, 46, 39, 47, 43, 146];
TestData.Frequency = [0, 0, 0, 0, 0, 0, 0, 0];
TestData.SteeringAngleRequest = [209, 201, 201, 2359, 2359, 3120, 3120, 149];
TestData.SteeringAngleRequestSign = [1, 1, 1, 1, 1, 0, 0, 1];
TestData.SteeringTorqueRequest = [0, 0, 0, 0, 0, 0, 0, 0];
TestData.SteeringTorqueRequestSign = [0, 0, 0, 0, 0, 0, 0, 0];
TestData.MessageHex = {'DF0AD100000400', 'CB06C900000400', 'CF02C900000400', '2E143709000400', '271D3709000400', '2F13300C000000', '2B17300C000000', '92039500000400'};

num_tests= 8;

%% Calculate Checksum and Check with Test Data

for i = 1:size(TestData.MessageHex,2)
    
       % pass test input, not that amplitude is not an input of the function
       [crc_fortiss, message_bin_fortiss] = Steering_CRC_Calculator(... 
       TestData.Counter(i),...
       TestData.ClearanceSI(i),...
       TestData.ActivationRequest(i),...
       TestData.CancelRequest(i),...
       TestData.SteeringAngleRequest(i),...
       TestData.SteeringTorqueRequest(i),...
       TestData.SteeringTorqueRequestSign(i),...
       TestData.SteeringAngleRequestSign(i),...
       TestData.AdvancedSteeringAngleLimitation(i),...
       TestData.Frequency(i));
   
   % convert 
   crc_bin = dec2bin(crc_fortiss);
   
   % convert test data checksum to binary
   testdata_crc_bin = padded_bin(TestData.CRC(i), 8, false);
   testdata_crc_bin = testdata_crc_bin{1};
   
   % convert test data hex message
   testdata_message_bytestring = dec2bin(hex2dec(TestData.MessageHex{i}));
   testdata_message_bytestring = [repmat('0', 1, 7*8-length(testdata_message_bytestring)), testdata_message_bytestring]; % add zeros if its too short
   testdata_message_bin = reshape(testdata_message_bytestring,8,7)';
   testdata_message_bin = testdata_message_bin(1:end-1, :);
   
   % Assertions
   assert(strcmp(testdata_message_bin(1,:), testdata_crc_bin), 'Testdata not plausibel');
   assert(strcmp(testdata_message_bin, message_bin_fortiss), 'Test message and generated message including checksum not equal')
   assert(crc_fortiss == TestData.CRC(i), 'Checksum in decimal failed')
   disp(['Test ', num2str(i), ' succeeded'])
end

