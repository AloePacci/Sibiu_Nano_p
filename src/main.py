import time
import os
from dronekit import connect, VehicleMode, LocationGlobal
import pymavlink
import traceback
from logger import Logger
from send_messages import Message_sender

if __name__ == '__main__':

    try:
        submarino=connect("192.168.2.1:8002", timeout=6.0, source_system=1, source_component=93)
    except ConnectionRefusedError:
        print(f"Connection refused")
        print("Log module is dead")
        exit()
    except OSError:
        error = traceback.format_exc()
        print(f"not found in the same network \n\n{error}")
        print("Log module is dead")
        exit()
    except TimeoutError:
        print(f"port was busy, timeout error")
        print("Log module is dead")
        exit()
    except:
        error = traceback.format_exc()
        print(f"Connectiom could not be made, unknown error:\n {error}")
        print("Log module is dead")
        exit()


    print(f"Connection SUCCESS")  
    log=Logger(submarino)
    time.sleep(1)
    handler=Message_sender()

    
    submarino.mode = VehicleMode("ALT_HOLD")
    submarino.arm()
    print(f"armed")
    #self.vehicle.play_tune("AAAA")
    for i in range(2):
        time.sleep(1)
    submarino.disarm()
    submarino.mode = VehicleMode("MANUAL")
    print(f"disarmed")
    time.sleep(20)
    log.stop_logging()

