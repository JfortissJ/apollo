# A python script to prepare cyber for distributed execution:
# Read the cyber/setup.bash file and set the cyber ip configuration
# to the PCs current IP address.

import sys
import socket
import fcntl
import struct
import netifaces

setup_bash_filepath = '/apollo/cyber/setup.bash'

# Possible network adapters on PC1, PC2, laptops with cables plugged, laptops with wifi
# The order is relevant: have loopback as last
def get_adapter_names():
    adapter_names = {'enp0s31f6', 'enp4s0', 'wlp59s0', 'lo'}
    return adapter_names

# Get the correct network according to the IP:
# In the car, this is the 192.168.140.* range and not the 10.0.4.* range
def get_correct_ip_address(adapter_names):
    for name in adapter_names:
        interfaces = netifaces.interfaces()
        if name in interfaces:
            ip = get_ip_address(name)
            if ip.find('10.'):
                continue
            else:
                break
    return ip


# Get IP Adress by adapter name
def get_ip_address(ifname):
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    return socket.inet_ntoa(fcntl.ioctl(
        s.fileno(),
        0x8915,  # SIOCGIFADDR
        struct.pack('256s', ifname[:15])
    )[20:24])


# find the (first) line that sets the cyber_ip
def find_cyber_ip_line(data):
    for idx, line in enumerate(data):
        if line[0:16] == "export CYBER_IP=":
            return idx


# main function: modify the cyber/setup.bash file to set the CYBER_IP to the ip address of the pc
if __name__ == '__main__':
    with open(setup_bash_filepath, 'r') as file:
        data = file.readlines()

    line_with_cyber_ip = find_cyber_ip_line(data)

    if line_with_cyber_ip == None:
        print("Could not replace cyber ip!")
        sys.exit(0)

    print('Original cyber IP in ', setup_bash_filepath,
          ' was: ', data[line_with_cyber_ip])

    adapter_names = get_adapter_names()
    current_ip = get_correct_ip_address(adapter_names)
    data[line_with_cyber_ip] = 'export CYBER_IP='+current_ip+'\n'

    with open(setup_bash_filepath, 'w') as file:
        file.writelines(data)

    print('cyber IP is now set to: ', data[line_with_cyber_ip])
