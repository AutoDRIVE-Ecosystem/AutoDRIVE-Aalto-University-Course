#!/usr/bin/env python3

################################################################################

# Copyright (c) 2025, Tinker Twins
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:

# 1. Redistributions of source code must retain the above copyright notice, this
#    list of conditions and the following disclaimer.
#
# 2. Redistributions in binary form must reproduce the above copyright notice,
#    this list of conditions and the following disclaimer in the documentation
#    and/or other materials provided with the distribution.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
# FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
# DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
# SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
# OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

################################################################################

# ROS 2 module imports
import rclpy # ROS 2 client library (rcl) for Python (built on rcl C API)
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy # Quality of Service (tune communication between nodes)
import tf2_ros # ROS bindings for tf2 library to handle transforms
from std_msgs.msg import Int32, Float32, Header # Int32, Float32 and Header message classes
from geometry_msgs.msg import Point, TransformStamped # Point and TransformStamped message classes
from sensor_msgs.msg import JointState, Imu, LaserScan, Image # JointState, Imu, LaserScan and Image message classes
from tf_transformations import quaternion_from_euler # Euler angle representation to quaternion representation
from threading import Thread # Thread-based parallelism

import cv2 # OpenCV library for computer vision tasks

# Python module imports
from cv_bridge import CvBridge # ROS bridge for OpenCV library to handle images
from gevent import pywsgi # Pure-Python gevent-friendly WSGI server
from geventwebsocket.handler import WebSocketHandler # Handler for WebSocket messages and lifecycle events
import socketio # Socket.IO realtime client and server
import numpy as np # Scientific computing
import base64 # Base64 binary-to-text encoding/decoding scheme
from io import BytesIO # Manipulate bytes data in memory
from PIL import Image # Python Imaging Library's (PIL's) Image module
import gzip # Inbuilt module to compress and decompress data and files
import autodrive_nigel.config as config # AutoDRIVE Ecosystem ROS 2 configuration for Nigel vehicle

################################################################################

# Nigel class
class Nigel:
    def __init__(self):
        # Vehicle data
        self.id                       = 1
        self.throttle                 = 0
        self.steering                 = 0
        self.speed                    = 0
        self.encoder_angles           = np.zeros(2, dtype=float)
        self.position                 = np.zeros(3, dtype=float)
        self.orientation_quaternion   = np.zeros(4, dtype=float)
        self.angular_velocity         = np.zeros(3, dtype=float)
        self.linear_acceleration      = np.zeros(3, dtype=float)
        self.lidar_scan_rate          = 40
        self.lidar_range_array        = np.zeros(360, dtype=float)
        self.lidar_intensity_array    = np.zeros(360, dtype=float)
        self.front_camera_image       = np.zeros((640, 360, 3), dtype=np.uint8)
        self.rear_camera_image        = np.zeros((640, 360, 3), dtype=np.uint8)
        # Vehicle commands
        self.throttle_command         = 0.0 # [-1, 1]
        self.steering_command         = 0.0 # [-1, 1]
        self.headlights_command       = 0 # {0=Disabled, 1=LowBeam, 2=HighBeam}
        self.indicators_command       = 0 # {0=Disabled, 1=LeftTurn, 2=RightTurn, 3=Hazard}
        self.reset_command            = False # {False, True}

# Traffic light class
class TrafficLight:
    def __init__(self):
        # Traffic light data
        self.id         = 1
        self.state      = 0
        # Traffic light command
        self.command    = 0 # {0=Disabled, 1=Red, 2=Yellow, 3=Green}

# Environment class
class Environment:
    def __init__(self):
        # Environmental conditions
        self.auto_time          = False # {False, True}
        self.time_scale         = 60 # [0, inf) (only used if auto_time==True)
        self.time_of_day        = 560 # {minutes in 24 hour format} (only used if auto_time==False)
        self.weather_id         = 3 # {0=Custom, 1=Sunny, 2=Cloudy, 3=LightFog, 4=HeavyFog, 5=LightRain, 6=HeavyRain, 7=LightSnow, 8=HeavySnow}
        self.cloud_intensity    = 0.0 # [0, 1] (only used if weather_id==0)
        self.fog_intensity      = 0.0 # [0, 1] (only used if weather_id==0)
        self.rain_intensity     = 0.0 # [0, 1] (only used if weather_id==0)
        self.snow_intensity     = 0.0 # [0, 1] (only used if weather_id==0)

################################################################################

# Global declarations
global autodrive_bridge, cv_bridge, publishers, transform_broadcaster
environment = Environment()
nigel_1 = Nigel()
tl_1 = TrafficLight()
tl_2 = TrafficLight()
tl_3 = TrafficLight()
tl_4 = TrafficLight()

#########################################################
# ROS 2 MESSAGE GENERATING FUNCTIONS
#########################################################

def create_int_msg(i, val):
    i.data = int(val)
    return i

def create_float_msg(f, val):
    f.data = float(val)
    return f

def create_joint_state_msg(js, joint_angle, joint_name, frame_id):
    js.header = Header()
    js.header.stamp = autodrive_bridge.get_clock().now().to_msg()
    js.header.frame_id = frame_id
    js.name = [joint_name]
    js.position = [joint_angle]
    js.velocity = []
    js.effort = []
    return js

def create_point_msg(p, position):
    p.x = position[0]
    p.y = position[1]
    p.z = position[2]
    return p

def create_imu_msg(imu, orientation_quaternion, angular_velocity, linear_acceleration):
    imu.header = Header()
    imu.header.stamp = autodrive_bridge.get_clock().now().to_msg()
    imu.header.frame_id = 'imu'
    imu.orientation.x = orientation_quaternion[0]
    imu.orientation.y = orientation_quaternion[1]
    imu.orientation.z = orientation_quaternion[2]
    imu.orientation.w = orientation_quaternion[3]
    imu.orientation_covariance = [0.0025, 0.0, 0.0, 0.0, 0.0025, 0.0, 0.0, 0.0, 0.0025]
    imu.angular_velocity.x = angular_velocity[0]
    imu.angular_velocity.y = angular_velocity[1]
    imu.angular_velocity.z = angular_velocity[2]
    imu.angular_velocity_covariance = [0.0025, 0.0, 0.0, 0.0, 0.0025, 0.0, 0.0, 0.0, 0.0025]
    imu.linear_acceleration.x = linear_acceleration[0]
    imu.linear_acceleration.y = linear_acceleration[1]
    imu.linear_acceleration.z = linear_acceleration[2]
    imu.linear_acceleration_covariance = [0.0025, 0.0, 0.0, 0.0, 0.0025, 0.0, 0.0, 0.0, 0.0025]
    return imu

def create_laserscan_msg(ls, lidar_scan_rate, lidar_range_array, lidar_intensity_array):
    ls.header = Header()
    ls.header.stamp = autodrive_bridge.get_clock().now().to_msg()
    ls.header.frame_id = 'lidar'
    ls.angle_min = 0.0 # Minimum angle of laser scan (0 radians)
    ls.angle_max = 6.28318530718 # Maximum angle of laser scan (2*Pi radians)
    ls.angle_increment = 0.0174532923847 # Angular resolution of laser scan (1 degree)
    ls.time_increment = (1/lidar_scan_rate)/360 # Time required to scan 1 degree
    ls.scan_time = 1/lidar_scan_rate # Time required to complete a scan of 360 degrees
    ls.range_min = 0.15 # Minimum sensor range (in meters)
    ls.range_max = 12.0 # Maximum sensor range (in meters)
    ls.ranges = lidar_range_array
    ls.intensities = lidar_intensity_array
    return ls

def create_image_msg(img, frame_id):
    img = cv_bridge.cv2_to_imgmsg(img, encoding="rgb8")
    img.header = Header()
    img.header.stamp = autodrive_bridge.get_clock().now().to_msg()
    img.header.frame_id = frame_id
    return img

def broadcast_transform(tf, tf_broadcaster, child_frame_id, parent_frame_id, position_tf, orientation_tf):
    tf.header.stamp = autodrive_bridge.get_clock().now().to_msg()
    tf.header.frame_id = parent_frame_id
    tf.child_frame_id = child_frame_id
    tf.transform.translation.x = position_tf[0] # Pos X
    tf.transform.translation.y = position_tf[1] # Pos Y
    tf.transform.translation.z = position_tf[2] # Pos Z
    tf.transform.rotation.x = orientation_tf[0] # Quat X
    tf.transform.rotation.y = orientation_tf[1] # Quat Y
    tf.transform.rotation.z = orientation_tf[2] # Quat Z
    tf.transform.rotation.w = orientation_tf[3] # Quat W
    tf_broadcaster.sendTransform(tf)

#########################################################
# ROS 2 MESSAGE DEFINITIONS
#########################################################

msg_int32 = Int32()
msg_float32 = Float32()
msg_jointstate = JointState()
msg_point = Point()
msg_imu = Imu()
msg_laserscan = LaserScan()
msg_transform = TransformStamped()

#########################################################
# ROS 2 PUBLISHER FUNCTIONS
#########################################################

# VEHICLE DATA PUBLISHER FUNCTIONS

def publish_nigel_1_actuator_feedbacks(throttle, steering):
    publishers['pub_nigel_1_throttle'].publish(create_float_msg(msg_float32, throttle))
    publishers['pub_nigel_1_steering'].publish(create_float_msg(msg_float32, steering))

def publish_nigel_1_speed_data(speed):
    publishers['pub_nigel_1_speed'].publish(create_float_msg(msg_float32, speed))

def publish_nigel_1_encoder_data(encoder_angles):
    publishers['pub_nigel_1_left_encoder'].publish(create_joint_state_msg(msg_jointstate, encoder_angles[0], "left_encoder", "left_encoder"))
    publishers['pub_nigel_1_right_encoder'].publish(create_joint_state_msg(msg_jointstate, encoder_angles[1], "right_encoder", "right_encoder"))

def publish_nigel_1_ips_data(position):
    publishers['pub_nigel_1_ips'].publish(create_point_msg(msg_point, position))

def publish_nigel_1_imu_data(orientation_quaternion, angular_velocity, linear_acceleration):
    publishers['pub_nigel_1_imu'].publish(create_imu_msg(msg_imu, orientation_quaternion, angular_velocity, linear_acceleration))

def publish_nigel_1_lidar_scan(lidar_scan_rate, lidar_range_array, lidar_intensity_array):
    publishers['pub_nigel_1_lidar'].publish(create_laserscan_msg(msg_laserscan, lidar_scan_rate, lidar_range_array.tolist(), lidar_intensity_array.tolist()))

def publish_nigel_1_front_camera_image(front_camera_image):
    publishers['pub_nigel_1_front_camera'].publish(create_image_msg(front_camera_image, "front_camera"))

def publish_nigel_1_rear_camera_image(rear_camera_image):
    publishers['pub_nigel_1_rear_camera'].publish(create_image_msg(rear_camera_image, "rear_camera"))

# TRAFFIC LIGHT DATA PUBLISHER FUNCTIONS

def publish_tl_1_state(tl_1_state):
    publishers['pub_tl_1_state'].publish(create_int_msg(msg_int32, tl_1_state))

def publish_tl_2_state(tl_2_state):
    publishers['pub_tl_2_state'].publish(create_int_msg(msg_int32, tl_2_state))

def publish_tl_3_state(tl_3_state):
    publishers['pub_tl_3_state'].publish(create_int_msg(msg_int32, tl_3_state))

def publish_tl_4_state(tl_4_state):
    publishers['pub_tl_4_state'].publish(create_int_msg(msg_int32, tl_4_state))

#########################################################
# ROS 2 SUBSCRIBER CALLBACKS
#########################################################

# VEHICLE DATA SUBSCRIBER CALLBACKS

def callback_nigel_1_throttle_command(throttle_command_msg):
    global nigel_1
    nigel_1.throttle_command = float(np.round(throttle_command_msg.data, 3))

def callback_nigel_1_steering_command(steering_command_msg):
    global nigel_1
    nigel_1.steering_command = float(np.round(steering_command_msg.data, 3))

def callback_nigel_1_headlights_command(headlights_command_msg):
    global nigel_1
    nigel_1.headlights_command = int(headlights_command_msg.data)

def callback_nigel_1_indicators_command(indicators_command_msg):
    global nigel_1
    nigel_1.indicators_command = int(indicators_command_msg.data)

def callback_nigel_1_reset_command(reset_command_msg):
    global nigel_1
    nigel_1.reset_command = reset_command_msg.data

# TRAFFIC LIGHT DATA SUBSCRIBER CALLBACKS

def callback_tl_1_command(tl_1_command_msg):
    global tl_1
    tl_1.command = int(tl_1_command_msg.data)

def callback_tl_2_command(tl_2_command_msg):
    global tl_2
    tl_2.command = int(tl_2_command_msg.data)

def callback_tl_3_command(tl_3_command_msg):
    global tl_3
    tl_3.command = int(tl_3_command_msg.data)

def callback_tl_4_command(tl_4_command_msg):
    global tl_4
    tl_4.command = int(tl_4_command_msg.data)

# ENVIRONMENT DATA SUBSCRIBER CALLBACKS

def callback_environment_auto_time(auto_time_msg):
    global environment
    environment.auto_time = auto_time_msg.data

def callback_environment_time_scale(time_scale_msg):
    global environment
    environment.time_scale = float(time_scale_msg.data)

def callback_environment_time_of_day(time_of_day_msg):
    global environment
    environment.time_of_day = int(time_of_day_msg.data)

def callback_environment_weather_id(weather_id_msg):
    global environment
    environment.weather_id = int(weather_id_msg.data)

def callback_environment_cloud_intensity(cloud_intensity_msg):
    global environment
    environment.cloud_intensity = float(cloud_intensity_msg.data)

def callback_environment_fog_intensity(fog_intensity_msg):
    global environment
    environment.fog_intensity = float(fog_intensity_msg.data)

def callback_environment_rain_intensity(rain_intensity_msg):
    global environment
    environment.rain_intensity = float(rain_intensity_msg.data)

def callback_environment_snow_intensity(snow_intensity_msg):
    global environment
    environment.snow_intensity = float(snow_intensity_msg.data)

#########################################################
# WEBSOCKET SERVER INFRASTRUCTURE
#########################################################

# Initialize the server
sio = socketio.Server(async_mode='gevent')

# Registering "connect" event handler for the server
@sio.on('connect')
def connect(sid, environ):
    print("Connected!")

# Registering "Bridge" event handler for the server
@sio.on('Bridge')
def bridge(sid, data):
    # Global declarations
    global environment, nigel_1, tl_1, tl_2, tl_3, tl_4, autodrive_bridge, cv_bridge, publishers, transform_broadcaster

    # Wait for data to become available
    if data:
        ########################################################################
        # INCOMMING DATA
        ########################################################################

        # VEHICLE DATA

        # Actuator feedbacks
        nigel_1.throttle = float(data["V1 Throttle"])
        nigel_1.steering = float(data["V1 Steering"])
        publish_nigel_1_actuator_feedbacks(nigel_1.throttle, nigel_1.steering)
        # Speed
        nigel_1.speed = float(data["V1 Speed"])
        publish_nigel_1_speed_data(nigel_1.speed)
        # Wheel encoders
        nigel_1.encoder_angles = np.fromstring(data["V1 Encoder Angles"], dtype=float, sep=' ')
        publish_nigel_1_encoder_data(nigel_1.encoder_angles)
        # IPS
        nigel_1.position = np.fromstring(data["V1 Position"], dtype=float, sep=' ')
        publish_nigel_1_ips_data(nigel_1.position)
        # IMU
        nigel_1.orientation_quaternion = np.fromstring(data["V1 Orientation Quaternion"], dtype=float, sep=' ')
        publish_nigel_1_imu_data(nigel_1.orientation_quaternion, nigel_1.angular_velocity, nigel_1.linear_acceleration)
        nigel_1.angular_velocity = np.fromstring(data["V1 Angular Velocity"], dtype=float, sep=' ')
        nigel_1.linear_acceleration = np.fromstring(data["V1 Linear Acceleration"], dtype=float, sep=' ')
        publish_nigel_1_imu_data(nigel_1.orientation_quaternion, nigel_1.angular_velocity, nigel_1.linear_acceleration)
        # LIDAR
        nigel_1.lidar_scan_rate = float(data["V1 LIDAR Scan Rate"])
        nigel_1.lidar_range_array = np.fromstring(gzip.decompress(base64.b64decode(data["V1 LIDAR Range Array"])).decode('utf-8'), sep='\n')
        publish_nigel_1_lidar_scan(nigel_1.lidar_scan_rate, nigel_1.lidar_range_array, nigel_1.lidar_intensity_array)
        # Cameras
        nigel_1.front_camera_image = np.asarray(Image.open(BytesIO(base64.b64decode(data["V1 Front Camera Image"]))))
        publish_nigel_1_front_camera_image(nigel_1.front_camera_image)
        nigel_1.rear_camera_image = np.asarray(Image.open(BytesIO(base64.b64decode(data["V1 Rear Camera Image"]))))
        publish_nigel_1_rear_camera_image(nigel_1.rear_camera_image)
        # Coordinate transforms
        broadcast_transform(msg_transform, transform_broadcaster, "nigel_1", "world", nigel_1.position, nigel_1.orientation_quaternion) # Vehicle frame defined at center of rear axle
        broadcast_transform(msg_transform, transform_broadcaster, "left_encoder", "nigel_1", np.asarray([0.0, 0.12, 0.0]), quaternion_from_euler(0.0, 120*nigel_1.encoder_angles[0]%6.283, 0.0))
        broadcast_transform(msg_transform, transform_broadcaster, "right_encoder", "nigel_1", np.asarray([0.0, -0.12, 0.0]), quaternion_from_euler(0.0, 120*nigel_1.encoder_angles[1]%6.283, 0.0))
        broadcast_transform(msg_transform, transform_broadcaster, "ips", "nigel_1", np.asarray([0.08, 0.0, 0.055]), np.asarray([0.0, 0.0, 0.0, 1.0]))
        broadcast_transform(msg_transform, transform_broadcaster, "imu", "nigel_1", np.asarray([0.08, 0.0, 0.055]), np.asarray([0.0, 0.0, 0.0, 1.0]))
        broadcast_transform(msg_transform, transform_broadcaster, "lidar", "nigel_1", np.asarray([0.2733, 0.0, 0.096]), np.asarray([0.0, 0.0, 0.0, 1.0]))
        broadcast_transform(msg_transform, transform_broadcaster, "front_camera", "nigel_1", np.asarray([-0.015, 0.0, 0.15]), np.asarray([0, 0.0871557, 0, 0.9961947]))
        broadcast_transform(msg_transform, transform_broadcaster, "front_left_wheel", "nigel_1", np.asarray([0.33, 0.118, 0.0]), quaternion_from_euler(0.0, 0.0, np.arctan((2*0.141537*np.tan(nigel_1.steering))/(2*0.141537-2*0.0765*np.tan(nigel_1.steering)))))
        broadcast_transform(msg_transform, transform_broadcaster, "front_right_wheel", "nigel_1", np.asarray([0.33, -0.118, 0.0]), quaternion_from_euler(0.0, 0.0, np.arctan((2*0.141537*np.tan(nigel_1.steering))/(2*0.141537+2*0.0765*np.tan(nigel_1.steering)))))
        broadcast_transform(msg_transform, transform_broadcaster, "rear_left_wheel", "nigel_1", np.asarray([0.0, 0.118, 0.0]), quaternion_from_euler(0.0, nigel_1.encoder_angles[0]%6.283, 0.0))
        broadcast_transform(msg_transform, transform_broadcaster, "rear_right_wheel", "nigel_1", np.asarray([0.0, -0.118, 0.0]), quaternion_from_euler(0.0, nigel_1.encoder_angles[1]%6.283, 0.0))

        # TRAFFIC LIGHT DATA

        # Traffic light 1
        tl_1.state = int(data["TL1 State"])
        publish_tl_1_state(tl_1.state)
        # Traffic light 2
        tl_2.state = int(data["TL2 State"])
        publish_tl_2_state(tl_2.state)
        # Traffic light 3
        tl_3.state = int(data["TL3 State"])
        publish_tl_3_state(tl_3.state)
        # Traffic light 4
        tl_4.state = int(data["TL4 State"])
        publish_tl_4_state(tl_4.state)

        ########################################################################
        # OUTGOING DATA
        ########################################################################
        
        sio.emit('Bridge', data={
                                # VEHICLE DATA
                                'V1 Throttle': str(nigel_1.throttle_command),
                                'V1 Steering': str(nigel_1.steering_command),
                                'V1 Headlights': str(nigel_1.headlights_command),
                                'V1 Indicators': str(nigel_1.indicators_command),
                                'V1 Reset': str(nigel_1.reset_command),
                                # TRAFFIC LIGHT DATA
                                'TL1 State': str(tl_1.command),
                                'TL2 State': str(tl_2.command),
                                'TL3 State': str(tl_3.command),
                                'TL4 State': str(tl_4.command),
                                # ENVIRONMENT DATA
                                'Auto Time': str(environment.auto_time),
                                'Time Scale': str(environment.time_scale),
                                'Time': str(environment.time_of_day),
                                'Weather': str(environment.weather_id),
                                'Clouds': str(environment.cloud_intensity),
                                'Fog': str(environment.fog_intensity),
                                'Rain': str(environment.rain_intensity),
                                'Snow': str(environment.snow_intensity)
                                }
                )

#########################################################
# AUTODRIVE ROS 2 BRIDGE INFRASTRUCTURE
#########################################################

def main():
    # Global declarations
    global environment, nigel_1, tl_1, tl_2, tl_3, tl_4, autodrive_bridge, cv_bridge, publishers, transform_broadcaster

    # ROS 2 infrastructure
    rclpy.init() # Initialize ROS 2 communication for this context
    autodrive_bridge = rclpy.create_node('autodrive_bridge') # Create ROS 2 node
    qos_profile = QoSProfile( # Quality of Service profile
        durability=QoSDurabilityPolicy.VOLATILE, # Volatile durability with no attempt made to persist samples
        reliability=QoSReliabilityPolicy.RELIABLE, # Reliable (not best effort) communication to guarantee that samples are delivered
        history=QoSHistoryPolicy.KEEP_LAST, # Keep/store only up to last N samples
        depth=1 # Queue (buffer) size/depth (only honored if the “history” policy was set to “keep last”)
        )
    cv_bridge = CvBridge() # ROS bridge object for OpenCV library to handle image data
    transform_broadcaster = tf2_ros.TransformBroadcaster(autodrive_bridge) # Initialize transform broadcaster
    publishers = {e.name: autodrive_bridge.create_publisher(e.type, e.topic, qos_profile)
                  for e in config.pub_sub_dict.publishers} # Publishers
    callbacks = {
        # Vehicle data
        '/autodrive/nigel_1/throttle_command': callback_nigel_1_throttle_command,
        '/autodrive/nigel_1/steering_command': callback_nigel_1_steering_command,
        '/autodrive/nigel_1/headlights_command': callback_nigel_1_headlights_command,
        '/autodrive/nigel_1/indicators_command': callback_nigel_1_indicators_command,
        '/autodrive/nigel_1/reset_command': callback_nigel_1_reset_command,
        # Traffic light data
        '/autodrive/tl_1/command': callback_tl_1_command,
        '/autodrive/tl_2/command': callback_tl_2_command,
        '/autodrive/tl_3/command': callback_tl_3_command,
        '/autodrive/tl_4/command': callback_tl_4_command,
        # Environment data
        '/autodrive/environment/auto_time': callback_environment_auto_time,
        '/autodrive/environment/time_scale': callback_environment_time_scale,
        '/autodrive/environment/time_of_day': callback_environment_time_of_day,
        '/autodrive/environment/weather_id': callback_environment_weather_id,
        '/autodrive/environment/cloud_intensity': callback_environment_cloud_intensity,
        '/autodrive/environment/fog_intensity': callback_environment_fog_intensity,
        '/autodrive/environment/rain_intensity': callback_environment_rain_intensity,
        '/autodrive/environment/snow_intensity': callback_environment_snow_intensity
    } # Subscriber callback functions
    [autodrive_bridge.create_subscription(e.type, e.topic, callbacks[e.topic], qos_profile) for e in config.pub_sub_dict.subscribers] # Subscribers

    # If num_threads is not specified then num_threads will be multiprocessing.cpu_count() if it is implemented
    # Otherwise, it will use a single thread
    executor = rclpy.executors.MultiThreadedExecutor() # Create executor to control which thread callbacks get executed
    executor.add_node(autodrive_bridge) # Add node whose callbacks should be managed by this executor

    process = Thread(target=executor.spin, daemon=True) # Runs callbacks in the thread
    process.start() # Activate the thread as demon (background process) and prompt it to the target function (spin the executor)

    app = socketio.WSGIApp(sio) # Create socketio WSGI application
    pywsgi.WSGIServer(('', 4567), app, handler_class=WebSocketHandler).serve_forever() # Deploy as a gevent WSGI server
    
    # Cleanup
    executor.shutdown() # Executor shutdown
    autodrive_bridge.destroy_node() # Explicitly destroy the node
    rclpy.shutdown() # Shutdown this context

################################################################################

if __name__ == '__main__':
    main() # Call main function of AutoDRIVE ROS 2 bridge