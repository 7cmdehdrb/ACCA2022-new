import time

def check_old_ndt(old_time, time_thres):
    print("old_time: , time: ", old_time, time.time())
    dt = time.time() - old_time
    if dt > time_thres:
        return True
    else:
        return False
