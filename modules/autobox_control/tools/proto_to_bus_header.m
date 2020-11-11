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

function proto_to_bus_header(bus_object, name, path)

    name_copy = name;
    signal_list = flaten_proto(bus_object, "");
    
    
    keys = {'double','logical','int32','char'};
    vals = {'SS_DOUBLE','SS_BOOLEAN','SS_INT32','SS_INT8'};
    type_map = containers.Map(keys,vals);
    
    % create function in file which takes sim struct as argument and
    % initializes all output ports from flattened bus
    defines = "";
    function_header = 'void initialize_outputs(SimStruct *S) {\n';
    function_body= "\n";
    function_end = '}';
    
    %% Write function to add output ports
    for k=1:numel(signal_list)
        name = signal_list(k).name;
        
        define_new = sprintf("#define %s %d \n",upper(strrep(name,"->","_")),k-1);
        
        defines = strcat(defines, define_new);
               
    end
    
    set_num_output_ports = sprintf("if (!ssSetNumOutputPorts(S, %d)) return;\n\n",numel(signal_list));
    
    function_body = strcat(function_body, set_num_output_ports);
    
    for k=1:numel(signal_list)
        width = signal_list(k).width;
        name = signal_list(k).name;
        
        set_port_width = sprintf("ssSetOutputPortWidth(S,%s,%d); // %s \n",upper(strrep(name,"->","_")),width, name );
        
        function_body = strcat(function_body, set_port_width);
               
    end
    
    for k=1:numel(signal_list)
        type = signal_list(k).type;
        
        set_port_type = sprintf("ssSetOutputPortDataType(S,%d,%s);\n",k-1,type_map(type));
        
        function_body = strcat(function_body, set_port_type);
               
    end
   	output_port_function = strcat(defines, function_header,function_body,function_end);
    
    %% Write function to fill output ports
    
    function_header = 'void fill_outputs(SimStruct *S, Apollo__Localization__LocalizationToAutobox *ptr_localization_unpacked) {\n uint32_t i; \n\n';
    function_body= "\n";
    
    ports = "";

     for k=1:numel(signal_list)
        name = signal_list(k).name;
        type = signal_list(k).type;
        
        port_new = sprintf("const %s* port_%s = (const %s*) ssGetInputPortSignal(S,%s);   \n",type,lower(strrep(name,"->","_")),type,upper(strrep(name,"->","_")));
        
        ports = strcat(ports, port_new);
               
     end
     
     fills = "!!!!ADD FILLING MANUALLY !!!!";
     
     fill_function = strcat(function_header, function_body,ports, fills, function_end);
    
     file_str = strcat( output_port_function,"\n\n\n", fill_function);
    
    fileID = fopen(fullfile(char(path),char(strcat('_init_sfunc_outputs_',name_copy,'.h'))),'w');
    fprintf(fileID,file_str);
    fclose(fileID);
    
   
    
    
    



