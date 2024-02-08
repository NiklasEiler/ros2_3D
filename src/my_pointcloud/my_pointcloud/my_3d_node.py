import sys
import os
import time

import rclpy 
import sensor_msgs.msg as sensor_msgs



from PyQt5.QtWidgets import QApplication, QWidget, QPushButton, QVBoxLayout
from threading import Thread
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
import pandas as pd
import open3d as o3d


class node_pointcloud(Node):

    def __init__(self):
        super().__init__('pcd_subsriber_node')
        self.cnt_3d_s=1
        self.i=0
        self.save_3d=0


        # Set up a subscription to the 'pcd' topic with a callback to the 
        # function `listener_callback`
        self.pcd_subscriber = self.create_subscription(
            sensor_msgs.PointCloud2,    # Msg type
            '/stereo/points2',                      # topic
            self.listener_callback,      # Function to call
            9000                          # QoS
        )


        self.subscription = self.create_subscription(
            String,
            '/gui/speichern',  
            self.button_callback,
            10  
        )
        
                
    def listener_callback(self, msg):
        self.i+=1
        pcd_as_numpy_array = np.array(list(read_points(msg)))
        pcd_as_numpy_array = pcd_as_numpy_array[:, :-1]
        pcd_as_numpy_array=pcd_as_numpy_array[~np.isnan(pcd_as_numpy_array).any(axis=1)] 
        
        #print(pcd_as_numpy_array)
        print('##########################################')
        
        
        if self.save_3d==1:
            path= "/home/stereocamera/Documents/stereocamera_mesurment/3d/"
            df = pd.DataFrame(pcd_as_numpy_array)
            df.to_csv(path + 'pc_'+ str((self.cnt_3d_s))+'.csv', index=False)
            self.save_3d=0
            self.cnt_3d_s+=1
    
    
    def button_callback(self, msg):
        self.save_3d=1
        print('save')



## The code below is "ported" from 
# https://github.com/ros/common_msgs/tree/noetic-devel/sensor_msgs/src/sensor_msgs
import sys
from collections import namedtuple
import ctypes
import math
import struct
from sensor_msgs.msg import PointCloud2, PointField

_DATATYPES = {}
_DATATYPES[PointField.INT8]    = ('b', 1)
_DATATYPES[PointField.UINT8]   = ('B', 1)
_DATATYPES[PointField.INT16]   = ('h', 2)
_DATATYPES[PointField.UINT16]  = ('H', 2)
_DATATYPES[PointField.INT32]   = ('i', 4)
_DATATYPES[PointField.UINT32]  = ('I', 4)
_DATATYPES[PointField.FLOAT32] = ('f', 4)
_DATATYPES[PointField.FLOAT64] = ('d', 8)

def read_points(cloud, field_names=None, skip_nans=False, uvs=[]):
    """
    Read points from a L{sensor_msgs.PointCloud2} message.
    @param cloud: The point cloud to read from.
    @type  cloud: L{sensor_msgs.PointCloud2}
    @param field_names: The names of fields to read. If None, read all fields. [default: None]
    @type  field_names: iterable
    @param skip_nans: If True, then don't return any point with a NaN value.
    @type  skip_nans: bool [default: False]
    @param uvs: If specified, then only return the points at the given coordinates. [default: empty list]
    @type  uvs: iterable
    @return: Generator which yields a list of values for each point.
    @rtype:  generator
    """
    assert isinstance(cloud, PointCloud2), 'cloud is not a sensor_msgs.msg.PointCloud2'
    fmt = _get_struct_fmt(cloud.is_bigendian, cloud.fields, field_names)
    width, height, point_step, row_step, data, isnan = cloud.width, cloud.height, cloud.point_step, cloud.row_step, cloud.data, math.isnan
    unpack_from = struct.Struct(fmt).unpack_from

    if skip_nans:
        if uvs:
            for u, v in uvs:
                p = unpack_from(data, (row_step * v) + (point_step * u))
                has_nan = False
                for pv in p:
                    if isnan(pv):
                        has_nan = True
                        break
                if not has_nan:
                    yield p
        else:
            for v in range(height):
                offset = row_step * v
                for u in range(width):
                    p = unpack_from(data, offset)
                    has_nan = False
                    for pv in p:
                        if isnan(pv):
                            has_nan = True
                            break
                    if not has_nan:
                        yield p
                    offset += point_step
    else:
        if uvs:
            for u, v in uvs:
                yield unpack_from(data, (row_step * v) + (point_step * u))
        else:
            for v in range(height):
                offset = row_step * v
                for u in range(width):
                    yield unpack_from(data, offset)
                    offset += point_step

def _get_struct_fmt(is_bigendian, fields, field_names=None):
    fmt = '>' if is_bigendian else '<'

    offset = 0
    for field in (f for f in sorted(fields, key=lambda f: f.offset) if field_names is None or f.name in field_names):
        if offset < field.offset:
            fmt += 'x' * (field.offset - offset)
            offset = field.offset
        if field.datatype not in _DATATYPES:
            print('Skipping unknown PointField datatype [%d]' % field.datatype, file=sys.stderr)
        else:
            datatype_fmt, datatype_length = _DATATYPES[field.datatype]
            fmt    += field.count * datatype_fmt
            offset += field.count * datatype_length

    return fmt


    

def main(args=None):
        
    rclpy.init(args=args)
    pointcloud = node_pointcloud()
    rclpy.spin(pointcloud)
    
    
    pointcloud.destroy_node()
    rclpy.shutdown()
    
if __name__ == '__main__':
    main()


'''
ros2 run my_pointcloud 3d_node
ros2 run rc_genicam_driver rc_genicam_driver --ros-args -p "device:=:02940786"
'''