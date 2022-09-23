from re import S
import time
import os
from dronekit import connect, VehicleMode, LocationGlobal
import pymavlink
import traceback
from logger import Logger
from send_messages import Message_sender


def main():
    submarino=Sibiu() #instanciate submarine
    try:
        submarino.test() #execute test
    except:
        print("test failed")
        pass
    
    # try:
    #     submarino.handler.play_tune("A") 
    # except:
    #     submarino.log.stop_logging()
    #     submarino.log.error("goto")

        
    submarino.log.stop_logging()
    print("finish")
    #submarino.handler.goto_deep(5) 
    #submarino.handler.orientate(90)
    


class Sibiu:
    def __init__(self):
        self.conectar() #connect to vehicle
        self.log=Logger(vehiculo=self.vehicle) #instanciate logger and start logging system status
        self.handler=Message_sender(vehiculo=self.vehicle, logger=self.log) #instatiate communication handler
        self.log.wait_till_init() #wait for things to init
        self.log.log("Connection Success") 


    def test(self):
        self.log.print(f"performing test")
        self.handler.setmode("ALT_HOLD")
        self.handler.arm()
        #self.vehicle.play_tune("AAAA")
        for i in range(5):
            self.handler.goto_deep(-i) 
            time.sleep(1)
        self.handler.disarm()
        self.handler.setmode("MANUAL")
        time.sleep(10)
        

    """this function manages the connection to the submarine
    @params:
        -ip: ip of the submarine
        -port: port to connect to
        -timeout: timeout for connection success
        -source system: ID for the program in MAVLink
        -source component: Component ID for the program in MAVLink
    @returns:
        it creates the shared variable self.vehicle
    @notes:
        if connection fails, it will log and notice reason of fail, killing the program
    """
    def conectar(self, ip="192.168.2.1", port="8003", timeout=6.0, source_system=36, source_component=93):
        try:
            self.vehicle=connect(ip+":"+port, timeout=timeout, source_system=source_system, source_component=source_component)
        except ConnectionRefusedError:
            self.log.error("Connection to submarine was refused")
        except OSError:
            self.log.error("Sibiu was not found in the same network")
        except TimeoutError:
            self.log.error("Submarine connection timeout, port is busy")
        except:
            self.log.error(f"Connectiom could not be made, unknown error:\n")

        print(f"Connection SUCCESS")  

if __name__ == '__main__':
    main()