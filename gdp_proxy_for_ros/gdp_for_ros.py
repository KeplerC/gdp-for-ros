import json
import queue
from random import randint
from scapy.all import *
import threading
from utils import *
from sshkeyboard import listen_keyboard



class GDP(Packet):
    name = "GDP"
    fields_desc = [
        BitField("src_gdpname", 0, 256), # 32
        BitField("dst_gdpname", 0, 256), # 32
        BitField("uuid", 0, 128), # 16
        IntField("num_packets", 1), # 4
        IntField("packet_no", 1), # 4
        ShortField("data_len", 0), # 2
        ByteField("action", 1), # 1        # 5 means advertise topic, 6 means topic message push
        ByteField("ttl", 64) # 1
    ]

bind_layers(UDP, GDP, dport=31415)

DATA_ASSEMBLER_MUTEX = threading.Lock()

class DataAssembler():
    def __init__(self, local_gdpname, local_ip, switch_ip):
        # map from series uuid to a list of received packet data for this series
        self.series_packets = dict()
        # map from series uuid to the number of packets should be received to complete this series
        self.series_packets_countdown = dict()
        # synchronized queue for assembled message
        self.message_queue = queue.Queue()

        self.local_gdpname = local_gdpname
        self.local_ip = local_ip
        self.switch_ip = switch_ip
    
    def process_packet(self, packet):
        '''
        Extract data from gdp_packet and put in series_packets.
        If all packets are received for this series, move complete payload to series_payload.
        '''
        if not packet.haslayer(GDP):
            return
        ip_layer = packet.getlayer(IP)
        # Checking if packet ip destination is current client proxy
        if ip_layer.dst != self.local_ip or ip_layer.src != self.switch_ip:
            print("Packet not from switch, ignoring..")
            return
        gdp_layer = packet.getlayer(GDP)
        # Checking if packet gdpname destination is current client proxy
        if gdp_layer.dst_gdpname != self.local_gdpname:
            return
        packet_num = gdp_layer.packet_no
        num_packets = gdp_layer.num_packets
        series_uuid = gdp_layer.uuid.to_bytes(16, "big").hex()

        # Critical section below, using a python mutex
        DATA_ASSEMBLER_MUTEX.acquire()

        if series_uuid in self.series_packets.keys():
            data_list = self.series_packets[series_uuid]
            index_in_list = packet_num - 1 # We do this because packet_num starts from 1 instead of 0 for each series
            if data_list[index_in_list] == None:
                data_list[index_in_list] = gdp_layer.load
                self.series_packets_countdown[series_uuid] -= 1
                # Assemble data and move to series_payload if all packets are presented      
        else:
            self.series_packets[series_uuid] = [None]*num_packets
            index_in_list = packet_num - 1
            self.series_packets[series_uuid][index_in_list] = gdp_layer.load
            self.series_packets_countdown[series_uuid] = num_packets - 1
        
        if self.series_packets_countdown[series_uuid] == 0:
            data_list = self.series_packets[series_uuid]
            assembled_data = reduce(lambda x, y: x+y, data_list)
            self.message_queue.put((series_uuid, hex(gdp_layer.src_gdpname), assembled_data))
            self.series_packets.pop(series_uuid)
            self.series_packets_countdown.pop(series_uuid)

        
        # Releasing the mutex
        DATA_ASSEMBLER_MUTEX.release()


def register_proxy(local_ip, switch_ip, local_GdpName, dst_GdpName):
    '''
    Register current client-side proxy to Global Data Plane. 
    Arguments:
        local_ip: the ipv4 address of current proxy in string format. e.g. "127.12.0.1"
        switch_ip: the ipv4 address of the switch this proxy is binding to. e.g. "127.12.0.15"
        local_GdpName: integer representation of the 256-bit-long SHA256-generated name for this proxy
    Returns:
        None
    '''
    bytes_array = map(lambda x: x.to_bytes(1, 'little'), map(lambda str_octets: int(str_octets), local_ip.split("."))) 
    print(local_ip)
    payload = reduce(lambda x, y: x+y, list(bytes_array))
    print(payload)

    packet = Ether(dst = 'ff:ff:ff:ff:ff:ff') / \
                IP(src=local_ip, dst=switch_ip)/ \
                    UDP(sport=31415, dport=31415)/ \
                        GDP(data_len=4, src_gdpname=local_GdpName, dst_gdpname=dst_GdpName, action=1,uuid=0, packet_no=1, num_packets=1 )/\
                             payload
    
    sendp(packet)



def advertise_topic_to_gdp(topic_name, is_by_pub, local_ip, local_gdpname, switch_ip):
    '''
    Advertise a topic to gdp. If it is called by a publisher, also register current gdp_client as topic publisher. 
    Otherwise, register as subscriber 

    Returns an integer representation of this topic's gdpname. Shared by the entire GDP
    '''
    print("The following is generated gdpname for topic={}".format(topic_name))
    topic_gdpname = generate_gdpname(topic_name + local_ip)
    
    
    payload = json.dumps(
        {
            'topic_name': topic_name,
            'topic_gdpname': list(topic_gdpname.to_bytes(32, 'big')),
            'is_pub': '1' if is_by_pub else '0'
        }
    ).encode('utf-8')

    packet = Ether(dst = 'ff:ff:ff:ff:ff:ff') / \
                IP(src=local_ip, dst=switch_ip)/ \
                    UDP(sport=31415, dport=31415)/ \
                        GDP(data_len=len(payload), src_gdpname=local_gdpname, dst_gdpname=0, action=5,uuid=0, packet_no=1, num_packets=1 )/\
                             payload
    sendp(packet)
    return topic_gdpname



def connect_self_to_topic(topic_gdpname, is_pub, local_ip, local_gdpname, switch_ip):
    '''
    Add current gdp client to the remote topic
    '''
    # Convert gdpname from hex string to int if input gdpname is in hex string instead of integer type
    if type(topic_gdpname) == str:
        topic_gdpname = gdpname_hex_to_int(topic_gdpname)

    payload = json.dumps(
        {
            'topic_name': "__",
            'topic_gdpname': list(topic_gdpname.to_bytes(32, 'big')),
            'is_pub': '1' if is_pub else '0'
        }
    ).encode('utf-8')

    packet = Ether(dst = 'ff:ff:ff:ff:ff:ff') / \
                IP(src=local_ip, dst=switch_ip)/ \
                    UDP(sport=31415, dport=31415)/ \
                        GDP(data_len=len(payload), src_gdpname=local_gdpname, dst_gdpname=0, action=5,uuid=0, packet_no=1, num_packets=1 )/\
                             payload
    sendp(packet)

    

def push_message_to_remote_topic(topic_name, topic_gdpname_str, local_ip, local_gdpname, switch_ip, message):
    '''
    Send the message to the specified remote topic on the GDP
    '''
    # Convert gdpname from hex string to int if input gdpname is in hex string instead of integer type
    if type(topic_gdpname_str) == str:
        topic_gdpname = gdpname_hex_to_int(topic_gdpname_str)

    payload = json.dumps({
        'topic_name': topic_name,
        'topic_gdpname': topic_gdpname_str,
        'message': message
    })
    
    send_packets(local_ip, switch_ip, local_gdpname, topic_gdpname, payload, action_no=6)




def start_sniffing(for_each):
    '''
    Start packet sniffing.
      for_each: a function that will be applied to every received packet. 
    '''
    sniff(prn=for_each)



def send_packets(local_ip, switch_ip, src_GdpName, dst_GdpName, serialized_string, action_no=3):
    '''
    Dissect message and send the packets one by one
    '''
    # dissect raw data if needed
    max_payload_size_per_packet = 500
    chunks = [serialized_string[i: i+max_payload_size_per_packet] for i in range(0, len(serialized_string), max_payload_size_per_packet)]
    
    series_uuid = generate_uuid()
    num_packets = len(chunks)
    # print(num_packets)   
    
    # packets need to be sent
    packet_list = []

    for i in range(num_packets):
        chunk = chunks[i]
        packet_no = i + 1
        packet = Ether(dst = 'ff:ff:ff:ff:ff:ff') / \
                    IP(src=local_ip, dst=switch_ip)/ \
                        UDP(sport=31415, dport=31415)/ \
                            GDP(
                                src_gdpname=src_GdpName, 
                                dst_gdpname=dst_GdpName, 
                                action=action_no, 
                                data_len=len(chunk), 
                                uuid=series_uuid, 
                                packet_no=packet_no, 
                                num_packets=num_packets
                                )/ \
                                chunk
        packet_list.append(packet)

    # send the packets one by one
    for i in range(len(packet_list)):
        sendp(packet_list[i], verbose=False)


if __name__ == "__main__":
    import argparse
    
    parser = argparse.ArgumentParser(description="Just an example", formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    parser.add_argument("switch_ip", help="switch ip that this proxy is binding to")
    parser.add_argument("switch_gdpname", help="switch gdpname")
    parser.add_argument("is_pub", help="1 to pub, 0 to sub")
    parser.add_argument("topic_gdpname", help="topic gdpname")

    args = parser.parse_args()
    # print(args.switch_ip, type(args.switch_ip))
    # print(args.switch_gdpname, type(args.switch_gdpname))
    # print(args.to_send_packets, type(args.to_send_packets))


    switch_ip = args.switch_ip
    switch_gdpname = int.from_bytes(bytes.fromhex(args.switch_gdpname), "big")
    data_assembler = None

    # register current proxy to a switch
    local_ip = get_local_ip()
    local_gdpname = generate_gdpname(local_ip)
    register_proxy(local_ip, switch_ip, local_gdpname, switch_gdpname)

    if args.is_pub == '1':
        topic_gdpname_int = advertise_topic_to_gdp("helloworld", True, local_ip, local_gdpname, switch_ip)
        print("This topic of helloworld has a gdpname = " + hex(topic_gdpname_int))
        listen_keyboard(on_press=lambda key: push_message_to_remote_topic("helloworld", hex(topic_gdpname_int)[2:], local_ip, local_gdpname, switch_ip, "Greetings, subscribers!"))
        
        

    # start receiving thread
    if args.is_pub == '0':
        data_assembler = DataAssembler(local_gdpname, local_ip, switch_ip)
        t = threading.Thread(target=start_sniffing, args=(lambda packet: data_assembler.process_packet(packet),))
        t.start()

        # time.sleep(0.4)

        connect_self_to_topic(args.topic_gdpname, False, local_ip, local_gdpname, switch_ip)
    

        # dst_gdpname = int.from_bytes(bytes.fromhex(args.dst_gdpname), "big")
        # def heartbeat():
        #     while True:
        #         time.sleep(randint(0,5))
        #         bytes_message = os.urandom(20)
        #         send_packets(local_ip, switch_ip, local_gdpname, dst_gdpname, bytes_message)
        # if args.to_send_packets == "1":
        #     heartbeat_thread = threading.Thread(target=heartbeat)
        #     heartbeat_thread.start()

        while True:
            uuid_and_message = data_assembler.message_queue.get()
            print(uuid_and_message, flush=True)

    





