
import numpy as np 
import time, zmq

def joint_space_waypoints(current, target, steps=100):
    s = np.linspace(0., 1., steps, dtype=np.float32)
    # current is repeated STEPS times and the differnce (target - current)
    # is repeated STEPS times and each time it's scaled by 
    # an element of s, taking small steps towards the target. 
    return current + (target - current) * s[:,  None]

def send_waypoints(bus, target, duration, verbose=False):
    current = bus.get_qpos()

    waypoints = joint_space_waypoints(current, target, 500)
    delay_per_step = duration / len(waypoints)

    for i, intermediate_waypoint in enumerate(waypoints):
        bus.set_qpos(intermediate_waypoint)
        time.sleep(delay_per_step)  # small delay between steps
        if verbose:
            print(f"Step {i}/{len(waypoints)}")

def make_pub(ctx, addr, topic_name):
    pub = ctx.socket(zmq.PUB)
    pub.bind(addr)
    print(f"Publishing on {addr}, topic = {topic_name}")
    return pub

def make_sub(ctx, addr, topic_name):
    sub = ctx.socket(zmq.SUB)
    sub.connect(addr)
    sub.setsockopt(zmq.SUBSCRIBE, topic_name.encode())
    print(f"Subscribed to {addr}, topic = {topic_name}")
    return sub

