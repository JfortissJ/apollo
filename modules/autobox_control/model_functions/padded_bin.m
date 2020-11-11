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

function out = padded_bin(dec, dimension, flip_bins, bit0)
% inputs:
%  - dec: decimal number
%  - dimension: signal length [bits]
%  - bit0: signal bit no
%  - flip_bins:  

if nargin < 4
    bit0 = 8;
end

bin = dec2bin(dec);

if(length(bin)>dimension)
    bin = '0';
%    disp('>>>>>>> out of range error! <<<<<<<<<');
end
padding0 = dimension-length(bin);
padding0 = max(padding0, 0);
bin = [repmat('0',1,padding0),bin];

if flip_bins
    bin = fliplr(bin);
end

if(length(bin)~=dimension)
    error('dimension error');
end

bytesize = 8;
num_bytes = ceil((dimension-bit0)/bytesize)+ceil(bit0/bytesize);

out=cell(num_bytes,1);

if dimension <= bytesize
    
    out = {bin};

else
   
    startbit = 1;
    endbit = bit0;
    for i=1:num_bytes
        out{i} = bin(startbit:endbit);
        startbit = endbit+1;
        endbit = min(endbit+bytesize, dimension);
    end
        
end

end