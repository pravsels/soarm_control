
import zmq

def make_pub(ctx, addr, topic_name, bind=True):
    pub = ctx.socket(zmq.PUB)
    if bind:
        pub.bind(addr)
        print(f"Publishing (bind) on {addr}, topic = {topic_name}")
    else:
        pub.connect(addr)
        print(f"Publishing (connect) to {addr}, topic = {topic_name}")
    return pub

def make_sub(ctx, addr, topic_name):
    sub = ctx.socket(zmq.SUB)
    sub.connect(addr)
    sub.setsockopt(zmq.SUBSCRIBE, topic_name.encode())
    sub.setsockopt(zmq.CONFLATE, 1)
    print(f"Subscribed to {addr}, topic = {topic_name}")
    return sub

