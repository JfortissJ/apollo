%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Apollo Autobox Control
% Copyright (C) 2020 fortiss GmbH
% Authors: Tobias Kessler, Jianjie Lin, Julian Bernhard, Klemens Esterle,
% Patrick Hart
%
% This library is free software; you can redistribute it and/or modify it
% under the terms of the GNU Lesser General Public License as published by
% the Free Software Foundation; either version 3 of the License, or (at 
% your option) any later version.
%
% This library is distributed in the hope that it will be useful, but 
% WITHOUT ANY WARRANTY; without even the implied warranty of 
% MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU Lesser 
% General Public License for more details.
%
% You should have received a copy of the GNU Lesser General Public License
% along with this library; if not, write to the Free Software Foundation,
% Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301, USA.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [byte0, acceleration_bytes] = Acceleration_CRC_Calculator(counter, clearanceAI, activationRequest, cancelRequest, accelerationRequest, clearanceStopDistance, stopDistanceRequest, acousticDriverHint)
% Calculate the checksum signal byte 1 XOR byte 2 XOR byte 3 XOR byte 4 XOR byte 5
% Implemenation idea here: in the order of the signals on the bus cast
% everything to 0/1 char arrays in 8-bit blocks, afterwards cast to logical
% an perform xor bitwise, then cast to uint8

% start 8 bit (CRC are first 8 bits)
counter_bin = padded_bin(counter, 4, true); 
% 8+4=12 bit
clearance_bin = padded_bin(clearanceAI, 1, true); 
% 12+1=13 bit
activation_bin = padded_bin(activationRequest, 1, true); 
% 13+1=14 bit
cancel_bin = padded_bin(cancelRequest, 1, true);
% 14+1=15 bit
acceleration_bin = padded_bin(accelerationRequest, 11, true, 1);
% 15+11=26 bit

byte1 = [counter_bin{1}, clearance_bin{1}, activation_bin{1}, cancel_bin{1}, acceleration_bin{1}]; % 9...16
byte1 = fliplr(byte1);

byte2 = [acceleration_bin{2}]; % 17 ... 24
byte2  = fliplr(byte2);

clearance_stop_distance_bin = padded_bin(clearanceStopDistance, 1, true);
% 26+1=27 bit
stop_distance_bin = padded_bin(stopDistanceRequest, 11, true, 5);
% 27+11=38 bit

byte3 = [acceleration_bin{3}, clearance_stop_distance_bin{1}, stop_distance_bin{1}]; % 25 ... 32
byte3 = fliplr(byte3);

acoustic_hint_bin = padded_bin(acousticDriverHint, 2, true);
% 40 bit

byte4 = [stop_distance_bin{2}, acoustic_hint_bin{1}];  % 33 ... 40
byte4 = fliplr(byte4);

bytesize = 8;
bytestring = '00000000';
for i=1:bytesize
    v1=str2num_internal(byte1(i));
    v2=str2num_internal(byte2(i));
    v3=str2num_internal(byte3(i));
    v4=str2num_internal(byte4(i));
    
    if xor(v4,xor(v3,xor(v2,v1)))
        bytestring(i) = '1';
    end
end
    
byte0=uint8(bin2dec(bytestring));
acceleration_bytes = [bytestring; byte1; byte2; byte3; byte4];


end
