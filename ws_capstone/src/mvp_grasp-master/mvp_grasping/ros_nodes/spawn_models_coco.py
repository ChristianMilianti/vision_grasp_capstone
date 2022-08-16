#!/usr/bin/env python2.7

import rospy
from gazebo_msgs.srv import SpawnModel, SpawnModelRequest, SpawnModelResponse
from copy import deepcopy
from tf.transformations import quaternion_from_euler

import random


sdf_bowl= """<?xml version="1.0" ?>
<sdf version="1.5">
  <model name="bowl">
    <link name="link">
      <inertial>
      <pose>0 0 0.0175 0 0 0</pose>
        <mass>0.1</mass>
	  <inertia>
	    <ixx>0.000250308</ixx>
	    <ixy>0.0</ixy>
	    <ixz>0.0</ixz>
	    <iyy>0.000250308</iyy>
	    <iyz>0.0</iyz>
	    <izz>0.0004802</izz>
	  </inertia>
      </inertial>
      <collision name="collision">
        <pose>0 0 0.0175 0 0 0</pose>
        <geometry>
          <cylinder>
            <radius>0.098</radius>
            <length>0.035</length>
          </cylinder>
        </geometry>
      </collision>
      <visual name="visual">
        <geometry>
          <mesh>
            <uri>model://bowl/meshes/bowl.dae</uri>
          </mesh>
        </geometry>
      </visual>
    </link>
  </model>
</sdf>
"""



sdf_cricket_ball= """<?xml version="1.0" ?>

<sdf version="1.5">
  <model name="cricket_ball">
    <link name="link">
      <pose>0 0 0.0375 0 0 0</pose>
      <inertial>
        <mass>0.1467</mass>
        <inertia>
          <ixx>8.251875e-05</ixx>
          <ixy>0</ixy>
          <ixz>0</ixz>
          <iyy>8.251875e-05</iyy>
          <iyz>0</iyz>
          <izz>8.251875e-05</izz>
        </inertia>
      </inertial>

      <collision name="collision">
        <geometry>
          <sphere>
            <radius>0.0375</radius>
          </sphere>
        </geometry>
        <surface>
          <contact>
            <!-- Red Pine coefficients for longitudinal axis of the wood
                 according to:
                 http://www.fpl.fs.fed.us/documnts/fplgtr/fplgtr113/ch04.pdf -->
            <poissons_ratio>0.347</poissons_ratio>
            <elastic_modulus>8.8e+09</elastic_modulus>
            <ode>
              <kp>100000</kp>
              <kd>100</kd>
              <max_vel>100.0</max_vel>
              <min_depth>0.001</min_depth>
            </ode>
          </contact>
          <friction>
            <torsional>
              <coefficient>1.0</coefficient>
              <use_patch_radius>0</use_patch_radius>
              <surface_radius>0.01</surface_radius>
            </torsional>
          </friction>
        </surface>
      </collision>

      <visual name="visual">
        <geometry>
          <sphere>
            <radius>0.0375</radius>
          </sphere>
        </geometry>
        <material>
          <script>
            <uri>file://media/materials/scripts/gazebo.material</uri>
            <name>Gazebo/Red</name>
          </script>
        </material>
      </visual>

      <!-- approximate rolling friction -->
      <velocity_decay>
        <linear>0.00</linear>
        <angular>0.005</angular>
      </velocity_decay>
    </link>
  </model>
</sdf>
"""


sdf_plastic_cup = """<?xml version="1.0" ?>
<sdf version="1.5">
  <model name="plastic_cup">
    <link name="link">
      <pose>0 0 0.065 0 0 0</pose>
      <inertial>
        <mass>0.0599</mass>
        <inertia>
          <ixx>0.0003028961527030333</ixx>
          <ixy>0</ixy>
          <ixz>0</ixz>
          <iyy>0.0003028961527030333</iyy>
          <iyz>0</iyz>
          <izz>3.2876352372798436e-05</izz>
        </inertia>
      </inertial>

      <collision name="collision">
        <geometry>
          <mesh>
            <uri>model://plastic_cup/meshes/plastic_cup.dae</uri>
          </mesh>
        </geometry>
        <surface>
          <contact>
            <!-- typical acrylic plastic material properties -->
            <poissons_ratio>0.35</poissons_ratio>
            <elastic_modulus>3.102640776e+09</elastic_modulus>
            <ode>
              <kp>100000</kp>
              <kd>100</kd>
              <max_vel>100.0</max_vel>
              <min_depth>0.001</min_depth>
            </ode>
          </contact>
          <friction>
            <torsional>
              <coefficient>1.0</coefficient>
              <use_patch_radius>0</use_patch_radius>
              <surface_radius>0.01</surface_radius>
            </torsional>
          </friction>
        </surface>
      </collision>

      <visual name="visual">
        <geometry>
          <mesh>
            <uri>model://plastic_cup/meshes/plastic_cup.dae</uri>
          </mesh>
        </geometry>
        <material>
          <script>
            <uri>file://media/materials/scripts/gazebo.material</uri>
            <name>Gazebo/GreyTransparent</name>
          </script>
        </material>
      </visual>

    </link>
  </model>
</sdf>
"""






def create_bowl_request(modelname, px, py, pz, rr, rp, ry, sx, sy, sz):
    """Create a SpawnModelRequest with the parameters of the bowl given.
    modelname: name of the model for gazebo
    px py pz: position of the bowl (and it's collision bowl)
    rr rp ry: rotation (roll, pitch, yaw) of the model
    sx sy sz: size of the bowl"""
    bowl = deepcopy(sdf_bowl)
    # Replace size of model
    size_str = str(round(sx, 3)) + " " + \
        str(round(sy, 3)) + " " + str(round(sz, 3))
    bowl = bowl.replace('SIZEXYZ', size_str)
    # Replace modelname
    bowl = bowl.replace('MODELNAME', str(modelname))

    req = SpawnModelRequest()
    req.model_name = modelname
    req.model_xml = bowl
    req.initial_pose.position.x = px
    req.initial_pose.position.y = py
    req.initial_pose.position.z = pz

    q = quaternion_from_euler(rr, rp, ry)
    req.initial_pose.orientation.x = q[0]
    req.initial_pose.orientation.y = q[1]
    req.initial_pose.orientation.z = q[2]
    req.initial_pose.orientation.w = q[3]

    return req


def create_cricket_ball_request(modelname, px, py, pz, rr, rp, ry, sx, sy, sz):
    """Create a SpawnModelRequest with the parameters of the ball given.
    modelname: name of the model for gazebo
    px py pz: position of the ball (and it's collision ball)
    rr rp ry: rotation (roll, pitch, yaw) of the model
    sx sy sz: size of the ball"""
    ball = deepcopy(sdf_cricket_ball)
    # Replace size of model
    size_str = str(round(sx, 3)) + " " + \
        str(round(sy, 3)) + " " + str(round(sz, 3))
    ball = ball.replace('SIZEXYZ', size_str)
    # Replace modelname
    ball = ball.replace('MODELNAME', str(modelname))

    req = SpawnModelRequest()
    req.model_name = modelname
    req.model_xml = ball
    req.initial_pose.position.x = px
    req.initial_pose.position.y = py
    req.initial_pose.position.z = pz

    q = quaternion_from_euler(rr, rp, ry)
    req.initial_pose.orientation.x = q[0]
    req.initial_pose.orientation.y = q[1]
    req.initial_pose.orientation.z = q[2]
    req.initial_pose.orientation.w = q[3]

    return req





def create_plastic_cup_request(modelname, px, py, pz, rr, rp, ry, sx, sy, sz):
    """Create a SpawnModelRequest with the parameters of the cup given.
    modelname: name of the model for gazebo
    px py pz: position of the cup (and it's collision cup)
    rr rp ry: rotation (roll, pitch, yaw) of the model
    sx sy sz: size of the cup"""
    cup = deepcopy(sdf_plastic_cup)
    # Replace size of model
    size_str = str(round(sx, 3)) + " " + \
        str(round(sy, 3)) + " " + str(round(sz, 3))
    cup = cup.replace('SIZEXYZ', size_str)
    # Replace modelname
    cup = cup.replace('MODELNAME', str(modelname))

    req = SpawnModelRequest()
    req.model_name = modelname
    req.model_xml = cup
    req.initial_pose.position.x = px
    req.initial_pose.position.y = py
    req.initial_pose.position.z = pz

    q = quaternion_from_euler(rr, rp, ry)
    req.initial_pose.orientation.x = q[0]
    req.initial_pose.orientation.y = q[1]
    req.initial_pose.orientation.z = q[2]
    req.initial_pose.orientation.w = q[3]

    return req



if __name__ == '__main__':
    rospy.init_node('spawn_models')
    spawn_srv = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
    rospy.loginfo("Waiting for /gazebo/spawn_sdf_model service...")
    spawn_srv.wait_for_service()
    rospy.loginfo("Connected to service!")

    x1= random.uniform(-0.4,0.2)
    y1= random.uniform(-0.37,0.2)

    x2= random.uniform(-0.4,0.2)
    y2= random.uniform(-0.37,0.2)

    x3= random.uniform(-0.4,0.2)
    y3= random.uniform(-0.37,0.2)

    x4= random.uniform(-0.4,0.2)
    y4= random.uniform(-0.37,0.2)

    x5= random.uniform(-0.4,0.2)
    y5= random.uniform(-0.37,0.2)

    x6= random.uniform(-0.4,0.2)
    y6= random.uniform(-0.37,0.2)

    x7= random.uniform(-0.4,0.2)
    y7= random.uniform(-0.37,0.2)

    x8= random.uniform(-0.4,0.2)
    y8= random.uniform(-0.37,0.2)

    
    # # Spawn Cube

    # rospy.loginfo("Spawning Cube")
    # rospy.loginfo(x1)
    # rospy.loginfo(y1)
    # req1 = create_cube_request("cube1",
    #                           0, 0, 0.05,  # position
    #                           0.0, 0.0, 0.0,  # rotation
    #                           0.05, 0.05, 0.05)  # size
    # spawn_srv.call(req1)
    # rospy.sleep(1.0)

    # # Spawn Bowl

    rospy.loginfo("Spawning Bowl")
    req2 = create_bowl_request("bowl1",
                              0, 0, 0.05,  # position
                              0.0, 0.0, 0.0,  # rotation
                              0.8, 0.8, 0.8)  # size
    spawn_srv.call(req2)
    rospy.sleep(1.0)

    # # Spawn Coke can

    # rospy.loginfo("Spawning Coke Can")
    # req2 = create_coke_can_request("coke_can1",
    #                           x3, y3, 0.05,  # position
    #                           0.0, 0.0, 0.0,  # rotation
    #                           0.8, 0.8, 0.8)  # size
    # spawn_srv.call(req2)
    # rospy.sleep(1.0)

    # # Spawn Cricket ball

    rospy.loginfo("Spawning Cricket ball")
    req2 = create_cricket_ball_request("cricket_ball1",
                              x4, y4, 0.05,  # position
                              0.0, 0.0, 0.0,  # rotation
                              0.8, 0.8, 0.8)  # size
    spawn_srv.call(req2)
    rospy.sleep(1.0)

    # # Spawn Door handle

    # rospy.loginfo("Spawning Door handle")
    # req2 = create_door_handle_request("door_handle1",
    #                           x5, y5, 0.05,  # position
    #                           0.0, 0.0, 0.0,  # rotation
    #                           0.8, 0.8, 0.8)  # size
    # spawn_srv.call(req2)
    # rospy.sleep(1.0)

    # # Spawn Hammer

    # rospy.loginfo("Spawning Hammer")
    # req2 = create_hammer_request("hammer1",
    #                           x6, y6, 0.05,  # position
    #                           0.0, 0.0, 0.0,  # rotation
    #                           0.8, 0.8, 0.8)  # size
    # spawn_srv.call(req2)
    # rospy.sleep(1.0)

    # # Spawn Wooden peg

    # rospy.loginfo("Spawning Wooden peg")
    # req2 = create_wooden_peg_request("wooden_peg1",
    #                           x7, y7, 0.05,  # position
    #                           0.0, 0.0, 0.0,  # rotation
    #                           0.8, 0.8, 0.8)  # size
    # spawn_srv.call(req2)
    # rospy.sleep(1.0)

    # # Plastic cup

    rospy.loginfo("Spawning Plastic cup")
    req2 = create_plastic_cup_request("plastic_cup1",
                              x8, x8, 0.05,  # position
                              0.0, 0.0, 0.0,  # rotation
                              0.8, 0.8, 0.8)  # size
    spawn_srv.call(req2)
    rospy.sleep(1.0)


