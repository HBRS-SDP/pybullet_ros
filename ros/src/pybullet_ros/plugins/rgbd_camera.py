#!/usr/bin/env python3

"""
RGBD camera sensor simulation for pybullet_ros base on pybullet.getCameraImage()
"""

import math
import numpy as np

import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

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
#!/usr/bin/env python3

"""
RGBD camera sensor simulation for pybullet_ros base on pybullet.getCameraImage()
"""

import math
import numpy as np

import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

import pybullet_data


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

    def point_cloud_from_cam(self):
        # self.pb.connect(self.pb.GUI)
        # self.pb.setAdditionalSearchPath(pybullet_data.getDataPath())
        # plane = self.pb.loadURDF("plane100.urdf")
        # cube = self.pb.loadURDF("cube.urdf", [0, 0, 1])


        def getRayFromTo(mouseX, mouseY):
            width, height, viewMat, projMat, cameraUp, camForward, horizon, vertical, _, _, dist, camTarget = self.pb.getDebugVisualizerCamera(
            )
            camPos = [
                camTarget[0] - dist * camForward[0], camTarget[1] - dist * camForward[1],
                camTarget[2] - dist * camForward[2]
            ]
            farPlane = 10000
            rayForward = [(camTarget[0] - camPos[0]), (camTarget[1] - camPos[1]), (camTarget[2] - camPos[2])]
            lenFwd = math.sqrt(rayForward[0] * rayForward[0] + rayForward[1] * rayForward[1] +
                                rayForward[2] * rayForward[2])
            invLen = farPlane * 1. / lenFwd
            rayForward = [invLen * rayForward[0], invLen * rayForward[1], invLen * rayForward[2]]
            rayFrom = camPos
            oneOverWidth = float(1) / float(width)
            oneOverHeight = float(1) / float(height)

            dHor = [horizon[0] * oneOverWidth, horizon[1] * oneOverWidth, horizon[2] * oneOverWidth]
            dVer = [vertical[0] * oneOverHeight, vertical[1] * oneOverHeight, vertical[2] * oneOverHeight]
            rayToCenter = [
                rayFrom[0] + rayForward[0], rayFrom[1] + rayForward[1], rayFrom[2] + rayForward[2]
            ]
            ortho = [
                -0.5 * horizon[0] + 0.5 * vertical[0] + float(mouseX) * dHor[0] - float(mouseY) * dVer[0],
                -0.5 * horizon[1] + 0.5 * vertical[1] + float(mouseX) * dHor[1] - float(mouseY) * dVer[1],
                -0.5 * horizon[2] + 0.5 * vertical[2] + float(mouseX) * dHor[2] - float(mouseY) * dVer[2]
            ]

            rayTo = [
                rayFrom[0] + rayForward[0] + ortho[0], rayFrom[1] + rayForward[1] + ortho[1],
                rayFrom[2] + rayForward[2] + ortho[2]
            ]
            lenOrtho = math.sqrt(ortho[0] * ortho[0] + ortho[1] * ortho[1] + ortho[2] * ortho[2])
            alpha = math.atan(lenOrtho / farPlane)
            return rayFrom, rayTo, alpha


        width, height, viewMat, projMat, cameraUp, camForward, horizon, vertical, _, _, dist, camTarget = self.pb.getDebugVisualizerCamera(
        )
        camPos = [
            camTarget[0] - dist * camForward[0], camTarget[1] - dist * camForward[1],
            camTarget[2] - dist * camForward[2]
        ]
        farPlane = 10000
        rayForward = [(camTarget[0] - camPos[0]), (camTarget[1] - camPos[1]), (camTarget[2] - camPos[2])]
        lenFwd = math.sqrt(rayForward[0] * rayForward[0] + rayForward[1] * rayForward[1] +
                        rayForward[2] * rayForward[2])
        oneOverWidth = float(1) / float(width)
        oneOverHeight = float(1) / float(height)
        dHor = [horizon[0] * oneOverWidth, horizon[1] * oneOverWidth, horizon[2] * oneOverWidth]
        dVer = [vertical[0] * oneOverHeight, vertical[1] * oneOverHeight, vertical[2] * oneOverHeight]

        lendHor = math.sqrt(dHor[0] * dHor[0] + dHor[1] * dHor[1] + dHor[2] * dHor[2])
        lendVer = math.sqrt(dVer[0] * dVer[0] + dVer[1] * dVer[1] + dVer[2] * dVer[2])

        cornersX = [0, width, width, 0]
        cornersY = [0, 0, height, height]
        corners3D = []

        imgW = int(width / 10)
        imgH = int(height / 10)

        img = self.pb.getCameraImage(imgW, imgH, renderer=self.pb.ER_BULLET_HARDWARE_OPENGL)
        rgbBuffer = img[2]
        depthBuffer = img[3]
        print("rgbBuffer.shape=", rgbBuffer.shape)
        print("depthBuffer.shape=", depthBuffer.shape)

        #disable rendering temporary makes adding objects faster
        self.pb.configureDebugVisualizer(self.pb.COV_ENABLE_RENDERING, 0)
        self.pb.configureDebugVisualizer(self.pb.COV_ENABLE_GUI, 0)
        self.pb.configureDebugVisualizer(self.pb.COV_ENABLE_TINY_RENDERER, 0)
        visualShapeId = self.pb.createVisualShape(shapeType=self.pb.GEOM_SPHERE, rgbaColor=[1, 1, 1, 1], radius=0.03)
        collisionShapeId = -1  #self.pb.createCollisionShape(shapeType=self.pb.GEOM_MESH, fileName="duck_vhacd.obj", collisionFramePosition=shift,meshScale=meshScale)

        for i in range(4):
            w = cornersX[i]
            h = cornersY[i]
            rayFrom, rayTo, _ = getRayFromTo(w, h)
            rf = np.array(rayFrom)
            rt = np.array(rayTo)
            vec = rt - rf
            l = np.sqrt(np.dot(vec, vec))
            newTo = (0.01 / l) * vec + rf
            #print("len vec=",np.sqrt(np.dot(vec,vec)))

            self.pb.addUserDebugLine(rayFrom, newTo, [1, 0, 0])
            corners3D.append(newTo)
        count = 0

        stepX = 5
        stepY = 5
        for w in range(0, imgW, stepX):
            for h in range(0, imgH, stepY):
                count += 1
                if ((count % 100) == 0):
                    print(count, "out of ", imgW * imgH / (stepX * stepY))
                rayFrom, rayTo, alpha = getRayFromTo(w * (width / imgW), h * (height / imgH))
                rf = np.array(rayFrom)
                rt = np.array(rayTo)
                vec = rt - rf
                l = np.sqrt(np.dot(vec, vec))
                depthImg = float(depthBuffer[h, w])
                far = 1000.
                near = 0.01
                depth = far * near / (far - (far - near) * depthImg)
                depth /= math.cos(alpha)
                newTo = (depth / l) * vec + rf
                self.pb.addUserDebugLine(rayFrom, newTo, [1, 0, 0])
                mb = self.pb.createMultiBody(baseMass=0,
                                    baseCollisionShapeIndex=collisionShapeId,
                                    baseVisualShapeIndex=visualShapeId,
                                    basePosition=newTo,
                                    useMaximalCoordinates=True)
                color = rgbBuffer[h, w]
                color = [color[0] / 255., color[1] / 255., color[2] / 255., 1]
                self.pb.changeVisualShape(mb, -1, rgbaColor=color)
        self.pb.addUserDebugLine(corners3D[0], corners3D[1], [1, 0, 0])
        self.pb.addUserDebugLine(corners3D[1], corners3D[2], [1, 0, 0])
        self.pb.addUserDebugLine(corners3D[2], corners3D[3], [1, 0, 0])
        self.pb.addUserDebugLine(corners3D[3], corners3D[0], [1, 0, 0])
        self.pb.configureDebugVisualizer(self.pb.COV_ENABLE_RENDERING, 1)
        print("ready\n")
        #self.pb.removeBody(plane)
        #self.pb.removeBody(cube)
        while (1):
            self.pb.setGravity(0, 0, -10)