from re import S
import time
import os
import traceback
from logger import Logger
from sibiu import Sibiu
import pygame
import threading



class Program():
    def __init__(self):
        self.running=True
        self.log=Logger() #instanciate logger
        self.submarino=Sibiu(log=self.log) #instanciate submarine
        self.rc_interrupt=threading.Thread(target=self.check_joy_interrupt) #instanciate interruptions
        self.__close=False

    def start(self):
        self.rc_interrupt.start() #start threads
        self.submarino.start()

    def check_joy_interrupt(self):
        pygame.init()
        self.j = pygame.joystick.Joystick(0)
        self.j.init()
        while not self.__close:
            events = pygame.event.get()
            for event in events:
                if event.type == pygame.JOYAXISMOTION:
                    if self.j.get_axis(5)>0:
                        self.log.print("asked to close program")
                        self.close()
                    elif self.j.get_axis(4)>0:
                        print("izquierdo")
                        self.submarino.pause()
                if event.type == pygame.JOYBUTTONUP:
                    if self.j.get_button(8):
                        self.submarino.resume()
        self.log.log("joy read stopped")
        self.j.quit()

            
    def close(self):
        self.__close=True
        self.log.stop_logging()
        self.submarino.close()
        self.submarino.handler.safe_close()
        self.running=False

        
    

if __name__ == '__main__':
    happen=Program()
    try:
        happen.start()
        while happen.running:
            pass
    except KeyboardInterrupt:
        print("EXITING NOW")
        happen.log.log("asked to finish program")
        happen.close()
    except:
        error=traceback.format_exc()
        print(f"something really bad happened \n\n{error}\n")

