from logger import Logger
from communication import Message_sender
import time
import os
import threading
import ctypes


class Sibiu(threading.Thread):
    def __init__(self, log=None):
        threading.Thread.__init__(self) #init thread
        if log is None:
            self.log=Logger() #instanciate logger and start logging system statusç
        else:
            self.log=log #restore log

        self.handler=Message_sender(logger=self.log) #instatiate communication handler
        self.log.append_vehicle(self.handler.get_vehicle()) #start logging vehicle data
        self.log.wait_till_init() #wait for things to init
        self.log.log("Connection Success")

    def pause(self):
        pass

    def resume(self):
        pass

    def test(self):
        self.log.print(f"performing test")
        self.handler.setmode("ALT_HOLD")
        self.handler.arm()
        #self.vehicle.play_tune("AAAA")
        for i in range (100000):
            #self.handler.goto_deep(1) 
            self.handler.orientate(90)
            time.sleep(0.1)
        self.handler.disarm()
        self.handler.setmode("MANUAL")
        time.sleep(2)
        self.log.print("test finished")


    def run(self):
        #self.test()
        pass

    def get_id(self):
        # returns id of the respective thread
        if hasattr(self, '_thread_id'):
            return self._thread_id
        for id, thread in threading._active.items():
            if thread is self:
                return id


    def raise_exception(self):
        thread_id = self.get_id()
        res = ctypes.pythonapi.PyThreadState_SetAsyncExc(thread_id,
              ctypes.py_object(SystemExit))
        if res > 1:
            ctypes.pythonapi.PyThreadState_SetAsyncExc(thread_id, 0)
            self.log.error('Exception raise failure')
        else:
            self.log.log("sibiu closed")
        


    def close(self):
        self.raise_exception()

