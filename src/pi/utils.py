import time

def wait(seconds):
    time.sleep(seconds)

def floor(*args):
    return tuple(int(arg) for arg in args)

def sign(x):
    return 1 if x >= 0 else -1