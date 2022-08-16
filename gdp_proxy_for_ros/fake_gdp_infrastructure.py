import zmq
import time
import sys



context = zmq.Context(1)
# Socket facing clients
frontend = context.socket(zmq.SUB)
frontend.bind("tcp://*:5559")
frontend.setsockopt(zmq.SUBSCRIBE, b"")        
# Socket facing services
backend = context.socket(zmq.PUB)
backend.bind("tcp://*:5560")
zmq.device(zmq.FORWARDER, frontend, backend)
