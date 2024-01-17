from ctypes import sizeof
from matplotlib.pyplot import get
import pyrealsense2 as rs
import numpy as np
import rclpy
import rclpy.node
import cv2
from tc_camera import transformations
from geometry_msgs.msg import Pose
from tc_msgs.srv import ArucoMarkers
from std_msgs.msg import Int32
from datetime import datetime
import math
import os
import time
import getpass
import platform #get devicename
# error code
DEVICE_NOT_FOUND = 610700
ARUCO_DETECTION_FAIL = 620700
class ArucoNode(rclpy.node.Node):

    def __init__(self):
        super().__init__('camera_node')
        # self.get_logger().info("Aruco dectection node init")
        log_write("LOG_CAMERA","Aruco dectection node init starts")
        
        # camera stream
        self.pipelines = []
        self.color_image = []
        self.aruco_size = 0.07
        #camera geometry setting from camera to robot ceter
        self.cam_offset_x = 0.3244
        self.cam_offset_y = -0.075

        # using for t-drive translation x,y
        self.docking_distance = 0.190 #0.175(SKT1) #0.135 (LM38)
        self.charging_distance = 0.20

        # aruco marker info
        self.aruco_dictionary = cv2.aruco.Dictionary_get(cv2.aruco.DICT_5X5_250)
        self.aruco_parameters = cv2.aruco.DetectorParameters_create()

        # Set up service server
        self.aruco_service = self.create_service(ArucoMarkers, 'aruco_pose', self.aruco_pose_callback)
        print("Creating alarm publisher...")
        self.alarm = self.create_publisher(Int32, "amr_alarms", 10)

        # detect available cameras plugged in
        self.detect_query_devices()
        self.start_camera()

        log_write("LOG_CAMERA","Aruco dectection node init finished")

        

    def aruco_pose_callback(self, request, response):
        # start camera
        # self.start_camera()

        # return to robot coordinate system
        v_const = 0.02
        w_const = 0.01
        tolerent = 0.005

        try:
            response.detected, response.id, x, y, theta = self.aruco_reading(self.aruco_size) # aruco_size = 0.07
            if response.detected == True:
                if request.task == 1:  # aquire and depsite, #translation x and y
                    x_trans = x - self.docking_distance             
                    response.dist = x_trans
                    response.vy = 0.0
                    response.vx = v_const
                elif request.task == 2: #charge
                    x_trans = x - self.charging_distance             
                    response.dist = x_trans
                    response.vy = 0.0
                    response.vx = v_const
                else:
                    response.dist = 0.0
                    response.vy = 0.0
                    response.vx = 0.0

                # if theta > 0:
                #     # response.vw = - w_const
                #     response.vw = 0.0
                # else:
                #     # response.vw = w_const
                #     response.vw = 0.0
                    
                # response.theta = abs(theta)
                response.vw = 0.0
                response.theta = theta * 180/3.14159 #convert to deg
                response.error_code = 0
            else:
                msg = Int32()
                msg.data = ARUCO_DETECTION_FAIL
                response.error_code = msg.data
                self.alarm.publish(msg)
            if not self.devices:
                msg = Int32()
                msg.data = DEVICE_NOT_FOUND
                response.error_code = msg.data
                self.alarm.publish(msg)
            
            log_write("LOG_CAMERA","request: " + str(request.task))
            log_write("LOG_CAMERA","response, aruco pose, " + "detected: " +str(response.detected)+ ", id:" + str(response.id) + 
                    ", dist: " + str(response.dist) + ", " + "vx: " + str(response.vx) + ", "+ "vy: " + str(response.vy) + 
                    ", "+ "theta(deg): " + str(response.theta) + ", "+ "vw: " + str(response.vw) + ", error_code: " + str(response.error_code))
        except Exception as e:
            log_write("ERR_CAMERA", str(e))
        
        # stop camera
        # self.stop_camera()

        return response


    def aruco_reading(self, aruco_size):
        
        aruco_detection = False
        id_aruco = 0
        x_aruco = 0.0
        y_aruco = 0.0
        theta_aruco = 0.0
        

        for dev_cam in range(len(self.dev_serial_list)): 

            frames = self.pipelines[dev_cam].wait_for_frames()
            color_frame = frames.get_color_frame()

            # Get the intrinsic matrix and distortion coefficients
            intrinsics = color_frame.get_profile().as_video_stream_profile().get_intrinsics()

            fx = intrinsics.fx
            fy = intrinsics.fy
            ppx = intrinsics.ppx
            ppy = intrinsics.ppy
            
            self.intrinsic_mat =np.array([[fx,   0., ppx],
                                    [  0., fy, ppy],
                                     [  0., 0., 1. ]])
            self.distortion  = np.array([0., 0., 0., 0.])
            # self.distortion = intrinsics.coeffs # from camera
            
            
            # Convert images to numpy arrays
            self.color_image = np.asanyarray(color_frame.get_data())
            corners, marker_ids, rejected = cv2.aruco.detectMarkers(self.color_image,
                                                                    self.aruco_dictionary,
                                                                    parameters=self.aruco_parameters)
            if marker_ids is not None:
                rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(corners,
                                                                    aruco_size, self.intrinsic_mat,
                                                                    self.distortion)
                for i, marker_id in enumerate(marker_ids):
                    
                    pose = Pose()
                    pose.position.x = tvecs[i][0][0]
                    pose.position.y = tvecs[i][0][1]
                    pose.position.z = tvecs[i][0][2]

                    rot_matrix = np.eye(4)
                    rot_matrix[0:3, 0:3] = cv2.Rodrigues(np.array(rvecs[i][0]))[0]
                    quat = transformations.quaternion_from_matrix(rot_matrix)

                    pose.orientation.x = quat[0]
                    pose.orientation.y = quat[1]
                    pose.orientation.z = quat[2]
                    pose.orientation.w = quat[3]

                    #calculate roll pitch yall
                    sy = math.sqrt(rot_matrix[0,0] * rot_matrix[0,0] +  rot_matrix[1,0] * rot_matrix[1,0])
                    roll = math.atan2(rot_matrix[2,1] , rot_matrix[2,2])
                    pitch = math.atan2(-rot_matrix[2,0], sy)
                    yall = math.atan2(rot_matrix[1,0], rot_matrix[0,0])
                    
                    # return aruco pose relatively to camera coordinate
                    aruco_detection = True
                    id_aruco = int(marker_ids[0])
                    x_aruco = pose.position.z
                    y_aruco = - pose.position.x
                    theta_aruco = - pitch

        log_write("LOG_CAMERA"," aruco pose, " + "detected: " +str(aruco_detection)+ ", id:" + str(id_aruco) + 
                  ", x_aruco: " + str(x_aruco) + ", " + "y_aruco: " + str(y_aruco) + ", "+ "theta_aruco(rad): " + str(theta_aruco))

        return aruco_detection, id_aruco, x_aruco,  y_aruco , theta_aruco

    def detect_query_devices(self):
        log_write("LOG_CAMERA","Detecting all available cameras")
        self.ctx = rs.context()
        self.dev_serial_list = []

        self.devices = self.ctx.query_devices()
        query_count = 0
        for i in range(20):
            if not self.devices:
                self.ctx = rs.context()
                self.devices = self.ctx.query_devices()
                i = i + 1
                log_write("LOG_CAMERA","Retry to connect camera count:" + str(i))
                time.sleep(1)

        if not self.devices:
            msg = Int32()
            msg.data = DEVICE_NOT_FOUND
            self.alarm.publish(msg)
            
            log_write("ERR_CAMERA","[ERROR] error_code:" + str(DEVICE_NOT_FOUND) + "-> No RealSense camera devices found")
            # rclpy.shutdown()

        else: 
            for dev in self.ctx.query_devices():
                log_write("LOG_CAMERA","Device name:" + str(dev.get_info(rs.camera_info.name)))

                self.pipeline = rs.pipeline(self.ctx)
                self.config = rs.config()
                self.dev_serial = dev.get_info(rs.camera_info.serial_number)
                self.dev_serial_list.append(self.dev_serial)
                log_write("LOG_CAMERA","serial no:" + str(self.dev_serial))
                del dev
                # self.config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30) 
                # self.config.enable_stream(rs.stream.color, 1280, 720, rs.format.bgr8, 30) 
                self.config.enable_stream(rs.stream.color, 1920, 1080, rs.format.bgr8, 30) 
                
                self.config.enable_device(self.dev_serial)
                # self.pipeline.start(self.config)
                # self.pipelines.append(self.pipeline)

                if len(self.dev_serial_list) > 1:
                    log_write("More than one camera connected, please check")
       
    def start_camera(self):
        self.pipeline.start(self.config)
        self.pipelines.append(self.pipeline)
        time.sleep(1)
        log_write("LOG_CAMERA", "Starting camera")

    def stop_camera(self):
        # Stop the pipeline(s)
        try:
            # for self.pipeline in self.pipelines:
            self.pipeline.stop()
            log_write("LOG_CAMERA", "Stopped camera")
        except Exception as e:
            log_write("ERR_CAMERA", str(e))

def log_write(info, msg):
    try:
        username = platform.node()
        current_second= datetime.now().second
        current_minute = datetime.now().minute
        current_hour = datetime.now().hour

        current_day = datetime.now().day
        current_month = datetime.now().month
        current_year = datetime.now().year

        if current_day < 10:
            current_day = "0" + str(current_day)
        if current_month < 10:
            current_month = "0" + str(current_month)
        
        home_directory = os.path.expanduser("~")
        log_directory = home_directory + "/" + "tcon/log"
        log_date = str(current_year) + str(current_month) + str(current_day)
        log_folder =  log_directory + "/" + log_date
        
        if not os.path.exists(log_folder):
            os.makedirs(log_folder)
            print ("create log folder")

        fileName = '['+ str(current_year) + str(current_month) + str(current_day) + ']' + '[' + str(username)+']'+ str(info) + '.txt'
        file = open(log_folder + "/" + fileName, 'a')
        file.write('['+str(current_year) +'/'+ str(current_month)+'/'+str(current_day)+'T'+ str(current_hour)+':'+ str(current_minute) +':'+str(current_second) +']'+"[camera]"+ str(msg) +"\n")
        print('['+str(current_year) +'/'+ str(current_month)+'/'+str(current_day)+'-'+ str(current_hour)+':'+ str(current_minute) +':'+str(current_second) +']'+"[camera]"+ str(msg))
        file.close
    except Exception as e:
        log_write("ERR_CAMERA", str(e))



def main():
    rclpy.init()
    node = ArucoNode()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()