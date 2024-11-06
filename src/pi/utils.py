import time

def wait(seconds):
    time.sleep(seconds)

def floor(*args):
    return tuple(int(arg) for arg in args)