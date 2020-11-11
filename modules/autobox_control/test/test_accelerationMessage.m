% clc
clear all
close all

%% Test Data (input as decimal)
TestData.ActivationRequest = [0, 0, 0, 0, 0, 0, 1, 0, 0, 0];
TestData.CancelRequest = [0, 0, 0, 0, 0, 0, 1, 0, 0, 0];
TestData.ClearanceAI = [0, 1, 1, 0, 1, 1, 1, 1, 0, 0];
TestData.Counter = [3, 15, 4, 8, 10, 3, 1, 8, 12, 2];
TestData.CRC = [211, 86, 93, 216, 29, 165, 229, 140, 136, 154];
TestData.AccelerationRequest = [1444, 1684, 1684, hex2dec('5A4'), hex2dec('608'), hex2dec('66B'), hex2dec('52C'), hex2dec('52C'), hex2dec('20C'), hex2dec('234')];
TestData.ClearanceStopDistance = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0];
TestData.StopDistanceRequest = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0];
TestData.AcousticDriverHint = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0];
TestData.MessageHex = {'D303D2020000', '561F4A030000', '5D144A030000', 'D808D2020000', '1D1A04030000', 'A59335030000', 'E57196020000', '8C1896020000', '880C06010000', '9A021A010000'};

num_tests= length(TestData.MessageHex);

%% Calculate Checksum and Check with Test Data

%for i = 1:num_tests
 for i=10
    % pass test input, not that amplitude is not an input of the function
    [crc_fortiss, message_bin_fortiss] = Acceleration_CRC_Calculator(...
        TestData.Counter(i),...
        TestData.ClearanceAI(i),...
        TestData.ActivationRequest(i),...
        TestData.CancelRequest(i),...
        TestData.AccelerationRequest(i),...
        TestData.ClearanceStopDistance(i),...
        TestData.StopDistanceRequest(i),...
        TestData.AcousticDriverHint(i));
    
    % convert
    crc_bin = dec2bin(crc_fortiss);
    
    % convert test data checksum to binary
    testdata_crc_bin = padded_bin(TestData.CRC(i), 8, false);
    testdata_crc_bin = testdata_crc_bin{1};
    
    % convert test data hex message
    testdata_message_bytestring = dec2bin(hex2dec(TestData.MessageHex{i}));
    testdata_message_bytestring = [repmat('0', 1, 6*8-length(testdata_message_bytestring)), testdata_message_bytestring]; % add zeros if its too short
    testdata_message_bin = reshape(testdata_message_bytestring,8,6)';
    testdata_message_bin = testdata_message_bin(1:end-1, :);
    
    % Assertions
    assert(strcmp(testdata_message_bin(1,:), testdata_crc_bin), 'Testdata not plausibel');
    assert(strcmp(testdata_message_bin, message_bin_fortiss), 'Test message and generated message including checksum not equal')
    assert(crc_fortiss == TestData.CRC(i), 'Checksum in decimal failed')
    
    disp(['Test ', num2str(i), ' succeeded'])
end

