import pybullet as p
import pybullet_data
import time
import numpy as np

# Connect to PyBullet in GUI mode and set up paths
p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, -9.8)

# Load the ground plane
p.loadURDF("plane.urdf")

# Load a table
table_position = [0.5, 0, 0]
table_orientation = p.getQuaternionFromEuler([0, 0, 0])
tableId = p.loadURDF("table/table.urdf", basePosition=table_position, baseOrientation=table_orientation)

# Create a cup as a simple cylinder
cup_radius = 0.03
cup_height = 0.1
cup_collision = p.createCollisionShape(p.GEOM_CYLINDER, radius=cup_radius, height=cup_height)
cup_visual = p.createVisualShape(p.GEOM_CYLINDER, radius=cup_radius, length=cup_height, rgbaColor=[1, 0, 0, 1])
cup_initial_pos = [0.5, 0, 0.65]
cupId = p.createMultiBody(baseMass=0.1, baseCollisionShapeIndex=cup_collision, baseVisualShapeIndex=cup_visual, basePosition=cup_initial_pos)

# Load robotic hand (Franka Panda)
hand_start_pos = [0.45, -0.2, 0.8]
hand_start_orn = p.getQuaternionFromEuler([0, 0, 0])
handId = p.loadURDF("franka_panda/panda.urdf", basePosition=hand_start_pos, baseOrientation=hand_start_orn, useFixedBase=1)

num_joints = p.getNumJoints(handId)
end_effector_index = 11  # End effector index for Franka Panda

# Move end-effector to cup position using IK
def move_to(target_pos, target_orn, steps=100):
    for i in range(steps):
        joint_positions = p.calculateInverseKinematics(handId, end_effector_index, target_pos, target_orn)
        for j in range(len(joint_positions)):
            p.setJointMotorControl2(handId, j, p.POSITION_CONTROL, joint_positions[j])
        p.stepSimulation()
        time.sleep(1./240.)

# Close the gripper
def control_gripper(close=True):
    grip_joint_indices = [9, 10]  # Franka's gripper joints
    grip_value = 0.02 if close else 0.04
    for joint in grip_joint_indices:
        p.setJointMotorControl2(handId, joint, p.POSITION_CONTROL, grip_value, force=10)
    for _ in range(50):
        p.stepSimulation()
        time.sleep(1./240.)

# Approach cup
approach_pos = [0.5, 0, 0.75]
move_to(approach_pos, p.getQuaternionFromEuler([0, np.pi, 0]))

# Move down to cup
move_to(cup_initial_pos, p.getQuaternionFromEuler([0, np.pi, 0]))

# Close gripper to grasp cup
control_gripper(close=True)

# Lift cup
lift_pos = [0.5, 0, 0.85]
move_to(lift_pos, p.getQuaternionFromEuler([0, np.pi, 0]))

# Move to new location
new_pos = [0.3, -0.3, 0.85]
move_to(new_pos, p.getQuaternionFromEuler([0, np.pi, 0]))

# Open gripper to release cup
control_gripper(close=False)

# Lower hand slightly to detach
move_to([new_pos[0], new_pos[1], new_pos[2] - 0.05], p.getQuaternionFromEuler([0, np.pi, 0]))

# Run simulation for a bit longer
time.sleep(2)
p.disconnect()