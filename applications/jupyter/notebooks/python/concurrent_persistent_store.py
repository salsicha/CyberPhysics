#!/usr/bin/python

import threading
import dbm
from random import random
import time
import json

# TODO: dbm doesn't store large objects well performantly, is there an alternative? 

# TODO: Switch to SQLite

def get(key):
    """get values from db"""
    with dbm.open('my_store', 'r') as db:
        value = json.loads(db[key])
    return value

def put(key, value):
    """put values in db"""
    with dbm.open('my_store', 'w') as db:
        db[key] = json.dumps(value)

def writer_thread(counter, write_lock):
    """write thread"""
    ts_array = []
    while counter > 0:
        counter -= 1
        ts = time.monotonic()
        with write_lock:
            put(str(counter), int(random() * seed))
        ts_array.append(time.monotonic() - ts)
    print("average write time: ", sum(ts_array) / len(ts_array))

def reader_thread(counter, write_lock):
    """read thread"""
    ts_array = []
    while counter > 0:
        counter -= 1
        ts = time.monotonic()
        with write_lock:
            read = get(str(counter))
        ts_array.append(time.monotonic() - ts)
    print("average read time: ", sum(ts_array) / len(ts_array))

# Initialize data
seed = 10000
values = [int(random() * seed) for _ in range(seed)]
with dbm.open('my_store', 'c') as db:
    for i, val in enumerate(values):
        db[str(i)] = str(val)

# Create lock and start threads
write_lock = threading.Lock()
threading.Thread(target=writer_thread, args=(seed, write_lock)).start()
threading.Thread(target=reader_thread, args=(seed, write_lock)).start()

# Example output:
# average read time:  0.0002128542541999994
# average write time:  0.0003347929823999983