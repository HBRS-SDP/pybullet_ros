#!/usr/bin/env python3

from pybullet_ros.plugins.environment import Environment as DefaultEnv
import rospy
import rospkg

class Environment(DefaultEnv):
    def __init__(self, pybullet, **kargs):
        super().__init__(pybullet)

    # def file_scan(self,world_name):
    #     parent_found=False
    #     counter=0
    #     current_directory =os.path.abspath(os.getcwd())
    #     while not parent_found and counter<10:    
    #         current_directory=os.path.abspath(os.path.join(current_directory,os.pardir))    
    #         foldername = os.path.basename(current_directory)

    #         if  foldername=='ros':
    #             parent_found=True   
    #             current_directory=os.path.abspath(os.path.join(current_directory,os.pardir))         

    #         counter+=1

        

    #     # world_file_name='test_world_v6.sdf'
    #     # relative_path=world_file_name
    #     world_folder=current_directory+'/common/test/sdf/worlds/'+world_name
    #     return world_folder

    # override parent method
    def load_environment_via_code(self):
        """
        This method provides the possibility for the user to define an environment via python code
        example:
        self.pb.loadURDF(...)
        self.pb.loadSDF(...)
        self.pb.loadSoftBody(...)
        self.pb.setTimeStep(0.001)
        self.pb.setPhysicsEngineParameter(sparseSdfVoxelSize=0.25) # ? related to soft bodies
        etc...
        """
       ## Name of the world file to use 
        world_file_name='test_world.sdf'


        ## including the file path to the world
        rospack = rospkg.RosPack()
        rospack_path=rospack.get_path('pybullet_ros')
        rospack_path=rospack_path+'/common/test/sdf/worlds/'+world_file_name       
        rospy.loginfo('Loading world file  : ' + rospack_path )         


        self.pb.loadSDF(rospack_path)

        rospy.logwarn('loading Pybullet world via code!')
