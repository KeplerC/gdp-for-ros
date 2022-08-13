#from scapy.layers.tuntap import TunTapInterface
'''
import scapy.all as scapy
from time import sleep 

# method 1 is to use scapy directly
t = scapy.TunTapInterface('tun0')  
while(True):
    print(t.recv_raw())

'''

# method 2 is to write on raw sockets 
import fcntl
import struct
import os
import socket
import threading
import sys

TUNSETIFF = 0x400454ca
TUNSETOWNER = TUNSETIFF + 2
IFF_TUN = 0x0001
IFF_TAP = 0x0002
IFF_NO_PI = 0x1000


def udp_send(dst, packet):
    print ("udp_send")
    sock.sendto(packet, (dst, 40000))

def recv():
     ss = socket.socket(socket.AF_INET, socket.SOCK_DGRAM) 
     ss.bind(("0.0.0.0", 40000))
     while True:
         data, addr = ss.recvfrom(1024)
         print ("udp_recv")
         os.write(tun.fileno(), data)

if __name__ == "__main__":

    print ("Working on tunros inteface")
    tun = open('/dev/net/tun', 'r+b', buffering = 0)
    ifr = struct.pack('16sH', b'tunros', IFF_TUN | IFF_NO_PI)
    fcntl.ioctl(tun, TUNSETIFF, ifr)
    fcntl.ioctl(tun, TUNSETOWNER, 1000)
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM) # UDP
    t = threading.Thread(target=recv)
    try:
        t.start()
        while True:
            packet = os.read(tun.fileno(), 2048)
            print(packet)

            

    except KeyboardInterrupt:
        print("Terminating ...")
        os._exit(0)
