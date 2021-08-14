#!/usr/bin/env python3

"""
RGBD camera sensor simulation for pybullet_ros base on pybullet.getCameraImage()
"""

import math
from os import cpu_count
import numpy as np

import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from sensor_msgs.msg import PointCloud2,PointField

##Imports for static transform
import tf
import tf2_ros
import geometry_msgs.msg


class RGBDCamera:
    def __init__(self, pybullet, robot, **kargs):
        # get "import pybullet as pb" and store in self.pb
        self.pb = pybullet
        # get robot from parent class
        self.robot = robot
        # create image msg placeholder for publication
        self.image_msg = Image()

        # get RGBD camera parameters from ROS param server
        self.image_msg.width = rospy.get_param('~rgbd_camera/resolution/width', 640)
        self.image_msg.height = rospy.get_param('~rgbd_camera/resolution/height', 480)
        assert(self.image_msg.width > 5)
        assert(self.image_msg.height > 5)
        cam_frame_id = rospy.get_param('~rgbd_camera/frame_id', None)
        if not cam_frame_id:
            rospy.logerr('Required parameter rgbd_camera/frame_id not set, will exit now...')
            rospy.signal_shutdown('Required parameter rgbd_camera/frame_id not set')
            return
        # get pybullet camera link id from its name
        link_names_to_ids_dic = kargs['link_ids']
        if not cam_frame_id in link_names_to_ids_dic:
            rospy.logerr('Camera reference frame "{}" not found in URDF model'.format(cam_frame_id))
            rospy.logwarn('Available frames are: {}'.format(link_names_to_ids_dic))
            rospy.signal_shutdown('required param rgbd_camera/frame_id not set properly')
            return
        self.pb_camera_link_id = link_names_to_ids_dic[cam_frame_id]
        self.image_msg.header.frame_id = cam_frame_id
        # create publisher
        self.pub_image = rospy.Publisher('rgb_image', Image, queue_size=1)
        self.image_msg.encoding = rospy.get_param('~rgbd_camera/resolution/encoding', 'rgb8')
        self.image_msg.is_bigendian = rospy.get_param('~rgbd_camera/resolution/encoding', 0)
        self.image_msg.step = rospy.get_param('~rgbd_camera/resolution/encoding', 1920)
        # projection matrix
        self.hfov = rospy.get_param('~rgbd_camera/hfov', 56.3)
        self.vfov = rospy.get_param('~rgbd_camera/vfov', 43.7)
        self.near_plane = rospy.get_param('~rgbd_camera/near_plane', 0.4)
        self.far_plane = rospy.get_param('~rgbd_camera/far_plane', 8)
        self.projection_matrix = self.compute_projection_matrix()
        # use cv_bridge ros to convert cv matrix to ros format
        self.image_bridge = CvBridge()
        # variable used to run this plugin at a lower frequency, HACK
        self.count = 0


        ## Setting TF static transform from camera to point cloud data in required direction
        point_cloud_frame="point_cloud_camera"        
        broadcaster = tf2_ros.StaticTransformBroadcaster()
        static_transformStamped = geometry_msgs.msg.TransformStamped()
        static_transformStamped.header.stamp = rospy.Time.now()
        static_transformStamped.header.frame_id = cam_frame_id
        static_transformStamped.child_frame_id = point_cloud_frame

        static_transformStamped.transform.translation.x = 0
        static_transformStamped.transform.translation.y = 0
        static_transformStamped.transform.translation.z = 0

        quat = tf.transformations.quaternion_from_euler(np.deg2rad(-90),0,0)
        static_transformStamped.transform.rotation.x = quat[0]
        static_transformStamped.transform.rotation.y = quat[1]
        static_transformStamped.transform.rotation.z = quat[2]
        static_transformStamped.transform.rotation.w = quat[3]       
        broadcaster.sendTransform(static_transformStamped)

        # publisher for depth image
        self.pub_depth_image = rospy.Publisher('depth_image', Image, queue_size=1)
        # image msg for depth image
        self.depth_image_msg = Image()
        self.depth_image_msg.width = rospy.get_param('~rgbd_camera/resolution/width', 640)
        self.depth_image_msg.height = rospy.get_param('~rgbd_camera/resolution/height', 480)
        self.depth_image_msg.encoding = rospy.get_param('~rgbd_camera/resolution/encoding', '32FC1')
        self.depth_image_msg.is_bigendian = rospy.get_param('~rgbd_camera/resolution/encoding', 0)
        self.depth_image_msg.step = rospy.get_param('~rgbd_camera/resolution/encoding', 2560)

        # publisher for point_cloud
        self.pub_point_cloud = rospy.Publisher('point_cloud', PointCloud2, queue_size=1)
        # point cloud msg 
        self.point_cloud_msg = PointCloud2()
        self.point_cloud_msg.header.frame_id = point_cloud_frame ## The new frame is the static frame 
        self.point_cloud_msg.width = rospy.get_param('~rgbd_camera/resolution/width',  640)
        self.point_cloud_msg.height = rospy.get_param('~rgbd_camera/resolution/height', 480)
        self.point_cloud_msg.fields = [PointField('x', 0, PointField.FLOAT32, 1),
                                       PointField('y', 4, PointField.FLOAT32, 1),
                                       PointField('z', 8, PointField.FLOAT32, 1)]
        self.point_cloud_msg.is_bigendian = rospy.get_param('~rgbd_camera/resolution/encoding', 0)
        self.point_cloud_msg.point_step = 12 
        self.point_cloud_msg.row_step = self.point_cloud_msg.width * self.point_cloud_msg.point_step
        self.point_cloud_msg.is_dense = True # organised point cloud
        

    def compute_projection_matrix(self):
        return self.pb.computeProjectionMatrix(
                    left=-math.tan(math.pi * self.hfov / 360.0) * self.near_plane,
                    right=math.tan(math.pi * self.hfov / 360.0) * self.near_plane,
                    bottom=-math.tan(math.pi * self.vfov / 360.0) * self.near_plane,
                    top=math.tan(math.pi * self.vfov / 360.0) * self.near_plane,
                    nearVal=self.near_plane,
                    farVal=self.far_plane)

    def extract_frame(self, camera_image):
        bgr_image = np.zeros((self.image_msg.height, self.image_msg.width, 3))

        camera_image = np.reshape(camera_image[2], (camera_image[1], camera_image[0], 4))

        bgr_image[:, :, 2] =\
            (1 - camera_image[:, :, 3]) * camera_image[:, :, 2] +\
            camera_image[:, :, 3] * camera_image[:, :, 2]

        bgr_image[:, :, 1] =\
            (1 - camera_image[:, :, 3]) * camera_image[:, :, 1] +\
            camera_image[:, :, 3] * camera_image[:, :, 1]

        bgr_image[:, :, 0] =\
            (1 - camera_image[:, :, 3]) * camera_image[:, :, 0] +\
            camera_image[:, :, 3] * camera_image[:, :, 0]

        # return frame
        return bgr_image.astype(np.uint8)

    def extract_depth_frame(self, camera_image):
        '''
        Inputs:
        -------
        camera_image: Obtained from getCameraImage()

        Returns:
        --------
        depth_image: Depth Image in cannonical form
        '''
        
        depth_image = camera_image[3]
        
        # Get true depth value (https://stackoverflow.com/questions/6652253/getting-the-true-z-value-from-the-depth-buffer)
        depth_image = self.far_plane * self.near_plane / ( self.far_plane - (self.far_plane - self.near_plane) * depth_image)

        # return depth_image 
        return depth_image.astype(np.float32) 


    def image_to_pointcloud(self, depth_image):
        '''
        Inputs:
        -------
        depth_image: Obtained from getCameraImage()

        Returns:
        --------
        point_cloud: point cloud data, sized (width x height x 3)
        '''
        
        # calculate the focal length
        y1_x = self.depth_image_msg.width / 2
        y1_y = self.depth_image_msg.height / 2
        focalLength_x = y1_x / np.tan(np.deg2rad(self.hfov)/2)
        focalLength_y = y1_y / np.tan(np.deg2rad(self.vfov)/2)

        # get the point cloud
        point_cloud_1 = []
        for v in range(depth_image.shape[0]):
            point_cloud_2 = []
            for u in range(depth_image.shape[1]):
                z = depth_image[v,u]
                x = (u - self.depth_image_msg.width / 2) * z / focalLength_x    
                y = (v - self.depth_image_msg.height / 2) * z / focalLength_y

                point_cloud_2.append([x, y, z]) 
            point_cloud_1.append(point_cloud_2) 
                    
        point_cloud = np.array(point_cloud_1)
        # print(point_cloud.shape)
        return point_cloud.astype(np.float32)

    def compute_camera_target(self, camera_position, camera_orientation):
        """
        camera target is a point 5m in front of the robot camera
        This method is used to tranform it to the world reference frame
        NOTE: this method uses pybullet functions and not tf
        """
        target_point = [5.0, 0, 0] # expressed w.r.t camera reference frame
        camera_position = [camera_position[0], camera_position[1], camera_position[2]]
        rm = self.pb.getMatrixFromQuaternion(camera_orientation)
        rotation_matrix = [[rm[0], rm[1], rm[2]],[rm[3], rm[4], rm[5]],[rm[6], rm[7], rm[8]]]
        return np.dot(rotation_matrix, target_point) + camera_position

    def execute(self):
        """this function gets called from pybullet ros main update loop"""

        # run at lower frequency, camera computations are expensive
        self.count += 1
        if self.count < 100:
            return
        self.count = 0 # reset count
        # get camera pose
        cam_state = self.pb.getLinkState(self.robot, self.pb_camera_link_id)
        # target is a point 5m ahead of the robot camera expressed w.r.t world reference frame
        target = self.compute_camera_target(cam_state[0], cam_state[1])
        view_matrix = self.pb.computeViewMatrix(cam_state[0], target, [0, 0, 1])
        # get camera image from pybullet
        pybullet_cam_resp = self.pb.getCameraImage(self.image_msg.width,
                                                   self.image_msg.height,
                                                   view_matrix,
                                                   self.projection_matrix,
                                                   renderer=self.pb.ER_BULLET_HARDWARE_OPENGL,
                                                   flags=self.pb.ER_NO_SEGMENTATION_MASK)
        # frame extraction function from qibullet
        frame = self.extract_frame(pybullet_cam_resp)
        # fill pixel data array
        self.image_msg.data = self.image_bridge.cv2_to_imgmsg(frame).data
        # update msg time stamp
        self.image_msg.header.stamp = rospy.Time.now()
        # publish camera image to ROS network
        self.pub_image.publish(self.image_msg)

        # Extract depth image/frame
        frame_depth = self.extract_depth_frame(pybullet_cam_resp)
        # fill pixel data array
        self.depth_image_msg.data = self.image_bridge.cv2_to_imgmsg(frame_depth, encoding="32FC1").data
        # update msg time stamp
        self.depth_image_msg.header.stamp = rospy.Time.now()
        # publish depth image to ROS network
        self.pub_depth_image.publish(self.depth_image_msg)

        # update msg time stamp
        self.point_cloud_msg.header.stamp = rospy.Time.now()
        # Get point cloud from depth image
        self.point_cloud_msg.data = self.image_to_pointcloud(frame_depth).tostring()
        # publish point cloud to ROS n/w
        self.pub_point_cloud.publish(self.point_cloud_msg)

