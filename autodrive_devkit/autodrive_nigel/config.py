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
from std_msgs.msg import Int32, Float32, Bool # Int32, Float32 and Bool message classes
from geometry_msgs.msg import Point # Point message class
from sensor_msgs.msg import JointState, Imu, LaserScan, Image # JointState, Imu, LaserScan and Image message classes

# Python mudule imports
try: # Mapping objects that allow their elements to be accessed both as keys and as attributes
    from attrdict import AttrDict
except ImportError: # attrdict broken in Python 3.10 and not maintained
    # Monkey patch collections
    import collections
    import collections.abc
    for type_name in collections.abc.__all__:
        setattr(collections, type_name, getattr(collections.abc, type_name))
    from attrdict import AttrDict

################################################################################

# ROS 2 publishers and subscribers
pub_sub_dict = AttrDict({
    'subscribers': [
        # Vehicle data
        {'topic':'/autodrive/nigel_1/throttle_command', 'type': Float32, 'name': 'sub_nigel_1_throttle_command'},
        {'topic':'/autodrive/nigel_1/steering_command', 'type': Float32, 'name': 'sub_nigel_1_steering_command'},
        {'topic':'/autodrive/nigel_1/headlights_command', 'type': Int32, 'name': 'sub_nigel_1_headlights_command'},
        {'topic':'/autodrive/nigel_1/indicators_command', 'type': Int32, 'name': 'sub_nigel_1_indicators_command'},
        {'topic':'/autodrive/nigel_1/reset_command', 'type': Bool, 'name': 'sub_nigel_1_reset_command'},
        # Traffic light data
        {'topic':'/autodrive/tl_1/command', 'type': Int32, 'name': 'sub_tl_1_command'},
        {'topic':'/autodrive/tl_2/command', 'type': Int32, 'name': 'sub_tl_2_command'},
        {'topic':'/autodrive/tl_3/command', 'type': Int32, 'name': 'sub_tl_3_command'},
        {'topic':'/autodrive/tl_4/command', 'type': Int32, 'name': 'sub_tl_4_command'},
        # Environment data
        {'topic':'/autodrive/environment/auto_time', 'type': Bool, 'name': 'sub_environment_auto_time'},
        {'topic':'/autodrive/environment/time_scale', 'type': Float32, 'name': 'sub_environment_time_scale'},
        {'topic':'/autodrive/environment/time_of_day', 'type': Int32, 'name': 'sub_environment_time_of_day'},
        {'topic':'/autodrive/environment/weather_id', 'type': Int32, 'name': 'sub_environment_weather_id'},
        {'topic':'/autodrive/environment/cloud_intensity', 'type': Float32, 'name': 'sub_environment_cloud_intensity'},
        {'topic':'/autodrive/environment/fog_intensity', 'type': Float32, 'name': 'sub_environment_fog_intensity'},
        {'topic':'/autodrive/environment/rain_intensity', 'type': Float32, 'name': 'sub_environment_rain_intensity'},
        {'topic':'/autodrive/environment/snow_intensity', 'type': Float32, 'name': 'sub_environment_snow_intensity'}
    ],
    'publishers': [
        # Vehicle data
        {'topic': '/autodrive/nigel_1/throttle', 'type': Float32, 'name': 'pub_nigel_1_throttle'},
        {'topic': '/autodrive/nigel_1/steering', 'type': Float32, 'name': 'pub_nigel_1_steering'},
        {'topic': '/autodrive/nigel_1/speed', 'type': Float32, 'name': 'pub_nigel_1_speed'},
        {'topic': '/autodrive/nigel_1/left_encoder', 'type': JointState, 'name': 'pub_nigel_1_left_encoder'},
        {'topic': '/autodrive/nigel_1/right_encoder', 'type': JointState, 'name': 'pub_nigel_1_right_encoder'},
        {'topic': '/autodrive/nigel_1/ips', 'type': Point, 'name': 'pub_nigel_1_ips'},
        {'topic': '/autodrive/nigel_1/imu', 'type': Imu, 'name': 'pub_nigel_1_imu'},
        {'topic': '/autodrive/nigel_1/lidar', 'type': LaserScan, 'name': 'pub_nigel_1_lidar'},
        {'topic': '/autodrive/nigel_1/front_camera', 'type': Image, 'name': 'pub_nigel_1_front_camera'},
        {'topic': '/autodrive/nigel_1/rear_camera', 'type': Image, 'name': 'pub_nigel_1_rear_camera'},
        # Traffic light data
        {'topic': '/autodrive/tl_1/state', 'type': Int32, 'name': 'pub_tl_1_state'},
        {'topic': '/autodrive/tl_2/state', 'type': Int32, 'name': 'pub_tl_2_state'},
        {'topic': '/autodrive/tl_3/state', 'type': Int32, 'name': 'pub_tl_3_state'},
        {'topic': '/autodrive/tl_4/state', 'type': Int32, 'name': 'pub_tl_4_state'}
    ]
})