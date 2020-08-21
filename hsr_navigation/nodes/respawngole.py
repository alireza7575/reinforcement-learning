# -*- coding: utf-8 -*-

import rospy
import random
import time
import os

from gazebo_msgs.srv import SpawnModel, DeleteModel
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import Pose
import tf.transformations as tf

class Respawn():
    def __init__(self):
       
        
        self.modelPath  = '/home/upm/RL_ws(18-8-2020)/src/hsr_navigation/model/goal_box/model.sdf'
        self.humanmodelPath  = '/home/upm/RL_ws(18-8-2020)/src/hsr_simulation/models/citizen_extras_male_05/model.sdf'
        self.f_human = open(self.humanmodelPath, 'r')
        self.f = open(self.modelPath, 'r')

        self.model = self.f.read()
        self.human_model = self.f_human.read()

        self.stage = 1
        self.goal_position = Pose()
        self.human_goal_position = Pose()

        self.init_goal_x = 0.8
        self.init_goal_y = 0.0
        self.goal_position.position.x = self.init_goal_x
        self.goal_position.position.y = self.init_goal_y

        self.human_goal_position.position.x = self.goal_position.position.x + 0.7
        self.human_goal_position.position.y = self.goal_position.position.y 
        quaternion = tf.quaternion_from_euler(0, 0,-1.5 )

        self.human_goal_position.orientation.x  = quaternion[0]
        self.human_goal_position.orientation.y  = quaternion[1]
        self.human_goal_position.orientation.z  = quaternion[2]
        self.human_goal_position.orientation.w  = quaternion[3]

        self.modelName = 'goal'
        self.obstacle_1 = -0.5, 0
        self.obstacle_2 = -0.5, -0.5
        self.obstacle_3 = -0.5, 0.5
        self.obstacle_4 = -0.8, 0
        self.last_goal_x = self.init_goal_x
        self.last_goal_y = self.init_goal_y
        self.last_index = 0
        self.sub_model = rospy.Subscriber('gazebo/model_states', ModelStates, self.checkModel)
        self.check_model = False
        self.index = 0

    def checkModel(self, model):
        self.check_model = False
        for i in range(len(model.name)):
            if model.name[i] == "goal":
                
                self.check_model = True

    def respawnModel(self, human):
        while True:
            if not self.check_model:
                rospy.wait_for_service('gazebo/spawn_sdf_model')
                spawn_model_prox = rospy.ServiceProxy('gazebo/spawn_sdf_model', SpawnModel)
                spawn_model_prox(self.modelName, self.model, 'robotos_name_space', self.goal_position, "world")
                if human:
                    spawn_model_prox('human', self.human_model, 'robotos_name_space', self.human_goal_position, "world")
                rospy.loginfo("Goal position : %.1f, %.1f", self.goal_position.position.x,
                              self.goal_position.position.y)
                break
            else:
                pass

    def deleteModel(self):
        while True:
            rospy.loginfo("delete")
            if self.check_model:
                rospy.wait_for_service('gazebo/delete_model')
                del_model_prox = rospy.ServiceProxy('gazebo/delete_model', DeleteModel)

                del_model_prox('human')
                
                del_model_prox(self.modelName)
                rospy.loginfo("delete done")
                break
            else:
                pass

    def getPosition(self, position_check=False, delete=False,human=False):
        if delete:
            self.deleteModel()

        if self.stage != 4:
            while position_check:
                goal_x = random.randrange(-12, 13) / 10.0
                goal_y = random.randrange(-12, 13) / 10.0
                if abs(goal_x - self.obstacle_1[0]) <= 0.4 and abs(goal_y - self.obstacle_1[1]) <= 0.4:
                    position_check = True
                elif abs(goal_x - self.obstacle_2[0]) <= 0.4 and abs(goal_y - self.obstacle_2[1]) <= 0.4:
                    position_check = True
                elif abs(goal_x - self.obstacle_3[0]) <= 0.4 and abs(goal_y - self.obstacle_3[1]) <= 0.4:
                    position_check = True
                elif abs(goal_x - self.obstacle_4[0]) <= 0.4 and abs(goal_y - self.obstacle_4[1]) <= 0.4:
                    position_check = True
                elif abs(goal_x - 0.0) <= 0.4 and abs(goal_y - 0.0) <= 0.4:
                    position_check = True
                else:
                    position_check = False

                if abs(goal_x - self.last_goal_x) < 1 and abs(goal_y - self.last_goal_y) < 1:
                    position_check = True

                self.goal_position.position.x = goal_x
                self.goal_position.position.y = goal_y

                self.human_goal_position.position.x = self.goal_position.position.x + 0.7
                self.human_goal_position.position.y = self.goal_position.position.y 

        else:
            while position_check:
                goal_x_list = [0.8, 1.9, 0.5, 0.2, -0.8, -1, -1.9, 0.5, 2, 0.5, 0, -0.1, -2]
                goal_y_list = [0, -0.5, -1.9, 1.5, -0.9, 1, 1.1, -1.5, 1.5, 1.8, -1, 1.6, -0.8]

                self.index = random.randrange(0, 13)
                print(self.index, self.last_index)
                if self.last_index == self.index:
                    position_check = True
                else:
                    self.last_index = self.index
                    position_check = False

                self.goal_position.position.x = goal_x_list[self.index]
                self.goal_position.position.y = goal_y_list[self.index]

                self.human_goal_position.position.x = self.goal_position.position.x + 0.7
                self.human_goal_position.position.y = self.goal_position.position.y 

        time.sleep(0.5)
        self.respawnModel(human=human)

        self.last_goal_x = self.goal_position.position.x
        self.last_goal_y = self.goal_position.position.y

        return self.goal_position.position.x, self.goal_position.position.y
