from re import S
import time
import os
from dronekit import connect, VehicleMode, LocationGlobal
import pymavlink
import traceback
from logger import Logger
from send_messages import Message_sender

import pygame
import threading



class Program():
    def __init__(self):
        self.submarino=Sibiu() #instanciate submarine
        self.rc_interrupt=threading.Thread(target=self.save_data)
        self.rc_interrupt.start()
        try:
            self.submarino.test() #execute test
        except:
            print("test failed")
            pass

        print("finish")
        # try:
        #     submarino.handler.play_tune("A") 
        # except:
        #     submarino.log.stop_logging()
        #     submarino.log.error("goto")
        #submarino.handler.goto_deep(5) 
        #submarino.handler.orientate(90)
    
    def check_joy_interrupt(self):
        pygame.init()
        self.j = pygame.joystick.Joystick(0)
        self.j.init()
        while True:
            events = pygame.event.get()
            for event in events:
                if event.type == pygame.JOYAXISMOTION:
                    if j.get_axis(5)>0:
                        print("derecho")
                    elif j.get_axis(4)>0:
                        print("izquierdo")

    def close(self):
        self.j.quit()
        self.submarino.log.stop_logging()


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
    try:
        happen=Program()
    except KeyboardInterrupt:
        print("EXCITING NOW")
    except:
        print("something really bad happened")



"""TODO:
def __init__(self):
    ...
    self.can_run = threading.Event()
    self.thing_done = threading.Event()
    self.thing_done.set()
    self.can_run.set()    

def run(self):
    while True:
        self.can_run.wait()
        try:
            self.thing_done.clear()
            print 'do the thing'
        finally:
            self.thing_done.set()

def pause(self):
    self.can_run.clear()
    self.thing_done.wait()

def resume(self):
    self.can_run.set()








class CustomThread(Thread):
    # override the run function
    def run(self):
        # block for a moment
        sleep(1)
        # display a message
        print('This is coming from another thread')
 
# create the thread
thread = CustomThread()
# start the thread
thread.start()
# wait for the thread to finish
print('Waiting for the thread to finish')
thread.join()
"""