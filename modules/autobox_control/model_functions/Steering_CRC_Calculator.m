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

function [byte0, steering_bytes] = Steering_CRC_Calculator(counter, clearance, activation, cancel, angle, torque, torquesign, anglesign, advanceanglelim, frequency)
% Calculate the checksum signal byte 1 XOR byte 2 XOR byte 3 XOR byte 4 XOR byte 5
% Implemenation idea here: in the order of the signals on the bus cast
% everything to 0/1 char arrays in 8-bit blocks, afterwards cast to logical
% an perform xor bitwise, then cast to uint8

counter_bin = padded_bin(counter, 4, true); % start 8 bit
clearance_bin = padded_bin(clearance, 2, true); % 12
activation_bin = padded_bin(activation, 1, true); % 14
cancel_bin = padded_bin(cancel, 1, true); % 15

byte1 = [counter_bin{1}, clearance_bin{1}, activation_bin{1}, cancel_bin{1}];
byte1 = fliplr(byte1);

angle_bin = padded_bin(angle, 16, true); % 16
byte2 = angle_bin{1};
byte2  = fliplr(byte2);

byte3 = angle_bin{2};
byte3 = fliplr(byte3);

torque_bin = padded_bin(torque, 9, true); % 32
byte4 = torque_bin{1};
byte4 = fliplr(byte4);

torquesign_bin = padded_bin(torquesign, 1, true); %41
anglesign_bin = padded_bin(anglesign, 1, true); % 42
advanceanglelim_bin = padded_bin(advanceanglelim,1, true); %43
frequency_bin = padded_bin(frequency,4, true); %44
byte5 = [torque_bin{2}, torquesign_bin{1}, anglesign_bin{1}, advanceanglelim_bin{1}, frequency_bin{1}];
byte5 = fliplr(byte5);

bytesize = 8;
bytestring = '00000000';
for i=1:bytesize
    v1=str2num_internal(byte1(i));
    v2=str2num_internal(byte2(i));
    v3=str2num_internal(byte3(i));
    v4=str2num_internal(byte4(i));
    v5=str2num_internal(byte5(i));
    
    if xor(v5,xor(v4,xor(v3,xor(v2,v1))))
        bytestring(i) = '1';
    end
end
    
byte0=uint8(bin2dec(bytestring));

steering_bytes = [bytestring; byte1; byte2; byte3; byte4; byte5];

end
