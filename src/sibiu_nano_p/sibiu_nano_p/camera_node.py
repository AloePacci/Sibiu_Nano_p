import os 
import sys

from rclpy.node import Node
import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from time import sleep
from uuv_interfaces.msg import Status, Nodeupdate, Location, String, Camera, Obstacles
from uuv_interfaces.srv import CommandBool
from rcl_interfaces.msg import Log
from .submodulos.call_service import call_service
from datetime import datetime
import traceback
import cv2
from pcv.vidIO import LockedCamera
from pcv.process import channel_options
import numpy as np
import threading
from math import atan2, degrees
import torch
import torch.backends.cudnn as cudnn
sys.path.insert(0, '~/yolov5')
from models.experimental import attempt_load
from utils.general import check_img_size, non_max_suppression, scale_coords, xyxy2xywh
from utils.torch_utils import select_device
from utils.augmentations import letterbox

class Camera(LockedCamera):
    ''' A camera class handling h264-encoded UDP stream. '''
    command = ('udpsrc port={} ! '
               'application/x-rtp, payload=96 ! '
               '{}rtph264depay ! '
               'decodebin ! videoconvert ! '
               'appsink')
    def __init__(self, port=5600, buffer=True, **kwargs):
        '''
        'port' is the UDP port of the stream. 
        'buffer' is a boolean specifying whether to buffer rtp packets
            -> reduces jitter, but adds latency
        '''
        self.port=port
        jitter_buffer = 'rtpjitterbuffer ! ' if buffer else ''
        super().__init__(self.command.format(port, jitter_buffer),
                         cv2.CAP_GSTREAMER, **kwargs)
        
        self.cap = cv2.VideoCapture(.format(self.port, 'rtpjitterbuffer ! '), cv2.CAP_GSTREAMER)
        fourcc = cv2.VideoWriter_fourcc(*'XVID')
        name=datetime.today()
        name=name.strftime('%Y.%m.%d..%H.%M')
        path=str("~/recording"+name+".avi")
        self.out = cv2.VideoWriter(path, fourcc, 20.0, (640,  480))
        if not cap.isOpened():
            self.get_logger().error("Cannot open camera")

    def get_image_iter(self):
        ret, frame = self.cap.read()
        if not ret:
            return None
        self.out.write(frame)
        return gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)        

    def stop_recording(self):
        self.cap.release()


class Camera_node(Node):

    def parameters(self):
        self.declare_parameter('vehicle_id', 1)
        self.vehicle_id = self.get_parameter('vehicle_id').get_parameter_value().integer_value
        self.declare_parameter('img_size', 640)
        self.img_size = self.get_parameter('img_size').get_parameter_value().integer_value
        self.declare_parameter('enable_obstacle_avoidance', True)
        self.enable_obstacle_avoidance = self.get_parameter('enable_obstacle_avoidance').get_parameter_value().bool_value
        self.declare_parameter('weights_filename', 'best.pt')
        path = "~/ASV_Loyola_US/" + self.get_parameter('weights_filename').get_parameter_value().string_value
        self.weights_filepath = os.path.expanduser(path)
        self.declare_parameter('confidence', 0.4)
        self.confidence = self.get_parameter('confidence').get_parameter_value().double_value

    def declare_services(self):
        self.sendinfo = self.create_service(CommandBool, 'camera_recording', self.camera_recording_callback)
        self.reset_home_service = self.create_service(CommandBool, 'enable_obstacle_avoidance', self.obstacle_avoidance_enable)

    def declare_topics(self):
        self.status_subscriber = self.create_subscription(Status, 'status', self.status_suscriber_callback, 10)
        self.mission_mode_subscriber = self.create_subscription(String, 'mission_mode', self.mission_mode_suscriber_callback, 10)
        self.obstacles_publisher = self.create_publisher(Obstacles, 'camera_obstacles', 10)

    #def declare_actions(self):

    def __init__(self):
        #start the node
        super().__init__('camera_node')

        #call the parameters
        self.parameters()

        #declare variables
        self.status=Status()
        self.mission_mode = ""
        self.recording=False

        #call services
        self.declare_services()

        #call actions
        #self.declare_actions()

        #declare topics
        self.declare_topics()
        
        ##connect to camera

        self.get_logger().info("Initializing camera")

        cam= BlueROVCamera(process=lambda img: channel_options(img))

        #define objects for threads
        self.lock = threading.Lock()
        self.run_signal = False
        self.stop_camera_detection=False

        #declare threads
        self.camera_detection_thread = threading.Thread(target=self.camera_perception, args=(self.weights_filepath,self.img_size, self.confidence,)) #weights, img_size, confidence
            
        #if defined at start, start obstacle avoidance thread   
        if self.enable_obstacle_avoidance:
            self.camera_detection_thread.start()
        else:
            self.get_logger().info("obstacle avoidance not enabled")

        while rclpy.ok():
                # if frame is read correctly ret is True
                if not ret:
                    print("Can't receive frame (stream end?). Exiting ...")
                    break
                # -- Get the image
                self.lock.acquire()
                self.image_net = cam.get_image_iter()
                self.lock.release()
                self.run_signal = True
                # -- Detection running on the other thread
                while self.run_signal:
                    sleep(0.001)
                # Wait for detections
                self.lock.acquire()
                # -- Ingest detections
                obstacles=Obstacles()
                obstacles.angle_increment=1.5 # we will cover an area of 110º (camera aperture) starting from -54 to 54º with an increment of 1.5 degrees as we can send at most 72 values
                obstacles.distance=[2000 for i in range(72)] #2000 or greater is no obstacle
                try:
                    for objeto in self.detections: #we will store everything of the object
                        obj = Camera()
                        obj.id = int(objeto.id) 
                        label=str(repr(objeto.label))
                        sublabel=str(repr(objeto.sublabel))
                        #self.get_logger().info(f"label: {label}, sublabel {sublabel}",once=True)
                        obj.label = label
                        obj.position = [objeto.position[0], objeto.position[1], objeto.position[2]]
                        #self.get_logger().info(f"pos: {[objeto.position[0], objeto.position[1], objeto.position[2]]}")
                        obj.confidence = objeto.confidence
                        #obj.tracking_state = objeto.tracking_state
                        #self.get_logger().info(f"track: {objeto.tracking_state}",once=True)
                        obj.dimensions = [objeto.dimensions[0], objeto.dimensions[1], objeto.dimensions[2]]
                        obj.velocity = [objeto.velocity[0], objeto.velocity[1], objeto.velocity[2]]
                        for i in range(4):
                            for j in range(2):
                                obj.bounding_box_2d.append(objeto.bounding_box_2d[i][j])
                        for i in range(8):
                            for j in range(3):
                                obj.bounding_box.append(objeto.bounding_box[i][j])
                        obstacles.objects.append(obj)

                        #for obstacle detection we apply pythagoras theorem
                        try:
                            minangle=degrees(atan2(objeto.bounding_box_2d[0][0],objeto.position[2]))
                            maxangle=degrees(atan2(objeto.bounding_box_2d[1][0],objeto.position[2]))
                        except:
                            continue
                        self.get_logger().info(f"{objeto.bounding_box_2d}, {objeto.position} , angles: {[minangle, maxangle]}")

                        #self.get_logger().info(f"{label}, {sublabel} , angles: {[minangle, maxangle]}")
                        if abs(minangle)>55 or maxangle>55:
                            self.get_logger().error("object trepassed camera limits")
                        else:
                            for i in range(int((53+minangle)/1.5),int((53+maxangle)/1.5)):                        
                                if obstacles.distance[i]>int(objeto.position[2]*100):
                                    obstacles.distance[i]=int(objeto.position[2]*100)
                    self.obstacles_publisher.publish(obstacles)
                except:
                    pass

        
    def obstacle_avoidance_enable(self, request, response):
        if request.value:
            self.get_logger().info("obstacle avoidance enabled")
            self.stop_camera_detection=False
            self.camera_detection_thread.start()
        else:
            self.stop_camera_detection=True
            self.get_logger().info("obstacle avoidance disabled")
        return response


    def status_suscriber_callback(self, msg):
        self.status = msg


    def mission_mode_suscriber_callback(self, msg):
        self.mission_mode=msg.string

    def camera_recording_callback(self, request, response): #TODO: allow to start/stop recording
        if request.value:
            if self.recording:
                self.get_logger().info("camera already recording")
                response.success=False
                return response #we are already recording
            self.recording=True
        else:
            if not self.recording: #we arent recording
                response.success=False
                self.get_logger().info("camera wasnt recording")
                return response
            self.get_logger().info("camera cannot stop recording, is not developed yet")
            self.recording=False
        return response


    def img_preprocess(self, img, device, half, net_size):
        net_image, ratio, pad = letterbox(img[:, :, :3], net_size, auto=False)
        net_image = net_image.transpose((2, 0, 1))[::-1]  # HWC to CHW, BGR to RGB
        net_image = np.ascontiguousarray(net_image)

        img = torch.from_numpy(net_image).to(device)
        img = img.half() if half else img.float()  # uint8 to fp16/32
        img /= 255.0  # 0 - 255 to 0.0 - 1.0

        if img.ndimension() == 3:
            img = img.unsqueeze(0)
        return img, ratio, pad

    def xywh2abcd(self, xywh, im_shape):
        output = np.zeros((4, 2))

        # Center / Width / Height -> BBox corners coordinates
        x_min = (xywh[0] - 0.5*xywh[2]) * im_shape[1]
        x_max = (xywh[0] + 0.5*xywh[2]) * im_shape[1]
        y_min = (xywh[1] - 0.5*xywh[3]) * im_shape[0]
        y_max = (xywh[1] + 0.5*xywh[3]) * im_shape[0]

        # A ------ B
        # | Object |
        # D ------ C

        output[0][0] = x_min
        output[0][1] = y_min

        output[1][0] = x_max
        output[1][1] = y_min

        output[2][0] = x_min
        output[2][1] = y_max

        output[3][0] = x_max
        output[3][1] = y_max
        return output

    def camera_perception(self, weights, img_size, conf_thres=0.2, iou_thres=0.45):

        self.get_logger().info("Intializing Network...")

        device = select_device()
        half = device.type != 'cpu'  # half precision only supported on CUDA
        imgsz = img_size

        # Load model
        model = attempt_load(weights, device=device)  # load FP32
        stride = int(model.stride.max())  # model stride
        imgsz = check_img_size(imgsz, s=stride)  # check img_size
        if half:
            model.half()  # to FP16
        cudnn.benchmark = True

        # Run inference
        if device.type != 'cpu':
            model(torch.zeros(1, 3, imgsz, imgsz).to(device).type_as(next(model.parameters())))  # run once

        self.get_logger().info("Network initialized")

        while rclpy.ok() and self.stop_camera_detection==False:
            if self.run_signal:
                self.lock.acquire()
                img, ratio, pad = self.img_preprocess(self.image_net, device, half, imgsz)

                pred = model(img)[0]
                det = non_max_suppression(pred, conf_thres, iou_thres)

                self.lock.release()
                self.run_signal = False
            sleep(0.01)

def main():
    rclpy.init()
    try:
        camera_node = Camera_node()
        rclpy.spin(camera_node, executor=MultiThreadedExecutor())
        # After finish close the camera
        camera_node.get_logger().info("normal finish")
        camera_node.destroy_node()
    except:
        #There has been an error with the program, so we will send the error log to the watchdog
        x = rclpy.create_node('camera_node') #we state what node we are
        publisher = x.create_publisher(Nodeupdate, '_internal_error', 10) #we create the publisher
        #we create the message
        msg = Nodeupdate()
        msg.node = "camera_node" #our identity
        msg.message = traceback.format_exc() #the error
        x.get_logger().error(f"error:{msg.message}")
        #to be sure the message reaches, we must wait till wathdog is listening (publisher needs time to start up)
        #TODO: Vulnerable si alguien esta haciendo echo del topic, el unico subscriptor debe ser wathdog
        # este topic está oculto en echo al usar _
        while publisher.get_subscription_count() == 0: #while no one is listening
            sleep(0.01) #we wait
        publisher.publish(msg) #we send the message
        x.destroy_node() #we destroy node and finish



if __name__ == '__main__':
    main()