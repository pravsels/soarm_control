
import time, json 
import zmq 
import numpy as np 
from bus import FeetechBus

JOINT_IDS = [1,2,3,4,5,6]
ADDR = "tcp://*:6000"

with open('so101_motorbus_port.json') as f: 
    port = json.load(f)['port']

bus = FeetechBus(port, JOINT_IDS, calib_file='so101_calibration.json')

def main():
    # setup ZeroMQ publisher 
    ctx = zmq.Context()
    pub = ctx.socket(zmq.PUB)
    pub.bind(ADDR)
    print(f'Publishing joint states on {ADDR}, topic = so101.state_real')

    try:
        while True: 
            qpos = bus.get_qpos().astype(float).tolist()
            msg = {'t': time.time(), 'qpos': qpos}
            pub.send_multipart([b'so101.state_real', json.dumps(msg).encode()])

            time.sleep(0.02)    # 50Hz, so delay is 1/50s 

    except KeyboardInterrupt:
        pass 
    finally: 
        pub.close(0)
        ctx.term()
        bus.disconnect()

if __name__ == '__main__':
    main()
