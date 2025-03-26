import pybullet as p
import pybullet_data
import time
import numpy as np

# Connect to PyBullet
p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, -9.8)

# Load the environment
p.loadURDF("plane.urdf")

# Load a table
table_position = [0.5, 0, 0]
tableId = p.loadURDF("table/table.urdf", basePosition=table_position)

# Load a coaster
coaster_radius = 0.1  # Increased radius
coaster_height = 0.01
coaster_collision = p.createCollisionShape(p.GEOM_CYLINDER, radius=coaster_radius, height=coaster_height)
coaster_visual = p.createVisualShape(p.GEOM_CYLINDER, radius=coaster_radius, length=coaster_height, rgbaColor=[0, 0, 1, 1])
coaster_position = [0.5, 0.3, 0.61]  # Place coaster slightly above the table surface
coasterId = p.createMultiBody(baseMass=0.05, baseCollisionShapeIndex=coaster_collision, baseVisualShapeIndex=coaster_visual, basePosition=coaster_position)


# Load a cup
cup_radius = 0.03
cup_height = 0.1
cup_collision = p.createCollisionShape(p.GEOM_CYLINDER, radius=cup_radius, height=cup_height)
cup_visual = p.createVisualShape(p.GEOM_CYLINDER, radius=cup_radius, length=cup_height, rgbaColor=[1, 0, 0, 1])
cup_initial_pos = [0.5, 0, 0.65]  # Placed away from the coaster
cupId = p.createMultiBody(baseMass=0.1, baseCollisionShapeIndex=cup_collision, baseVisualShapeIndex=cup_visual, basePosition=cup_initial_pos)
p.changeDynamics(cupId, -1, angularDamping=0.25)

# Load robotic arm (Franka Panda)
table_height = 0.62  # Approximate height of the table
robot_start_pos = [0,0,table_height]  # Place the robot on top of the table
robotId = p.loadURDF("franka_panda/panda.urdf", basePosition=robot_start_pos, useFixedBase=1)

end_effector_index = 11  # End effector link index for Franka
suction_constraint = None

# Move robot end-effector to a target position
def move_to(target_pos, target_orn, steps=200):
    for _ in range(steps):
        joint_positions = p.calculateInverseKinematics(robotId, end_effector_index, target_pos, target_orn)
        for j in range(len(joint_positions)):
            p.setJointMotorControl2(robotId, j, p.POSITION_CONTROL, joint_positions[j])
        p.stepSimulation()
        time.sleep(1./240.)

# Weak suction: Gradually approach before attaching
def weak_suction_approach(target_pos, target_orn, threshold=0.005, step_size=0.01):
    global suction_constraint

    while True:
        ee_pos = np.array(p.getLinkState(robotId, end_effector_index)[0])  
        cup_pos = np.array(p.getBasePositionAndOrientation(cupId)[0])
        dist = np.linalg.norm(ee_pos - cup_pos)

        print(f"Distance to cup: {dist:.6f}m")  # Debugging info

        if dist <= threshold:
            print("Suction activated!")
            activate_suction()
            break

        # Move closer step-by-step
        new_pos = ee_pos + (cup_pos - ee_pos) * (step_size / dist)
        move_to(new_pos.tolist(), target_orn, steps=50)

# Attach the cup
def activate_suction():
    global suction_constraint
    suction_constraint = p.createConstraint(
        parentBodyUniqueId=robotId, parentLinkIndex=end_effector_index,
        childBodyUniqueId=cupId, childLinkIndex=-1,
        jointType=p.JOINT_FIXED,
        jointAxis=[0, 0, 0],
        parentFramePosition=[0, 0, 0],  # Keep the effector's frame position
        childFramePosition=[0, 0, -1.3*cup_height / 2]  # Offset downward to avoid overlap
    )
    print("Suction applied successfully!")

# Release the cup
def deactivate_suction():
    global suction_constraint
    if suction_constraint is not None:
        p.removeConstraint(suction_constraint)
        suction_constraint = None

move_to([-0.5, -0.5, 0.85], p.getQuaternionFromEuler([0, np.pi, 0]))

# Move to start position above the cup
approach_pos = [0.5, 0, cup_initial_pos[2] + 0.05]
move_to(approach_pos, p.getQuaternionFromEuler([0, np.pi, 0]))

# Approach cup with weak suction effect
weak_suction_approach([0.5, 0, cup_initial_pos[2] + cup_height / 2], p.getQuaternionFromEuler([0, np.pi, 0]), threshold=0.15)

# Lift cup
move_to([0.5, 0, 0.85], p.getQuaternionFromEuler([0, np.pi, 0]))

# Move to a new position
move_to([0.5, 0.3, 0.85], p.getQuaternionFromEuler([0, np.pi, 0]))

# Drop the cup
deactivate_suction()
# time.sleep(1)

# Lower hand slightly after releasing
move_to([0.3, -0.3, 0.8], p.getQuaternionFromEuler([0, np.pi, 0]))

# Run for a bit longer before disconnecting
time.sleep(2)
p.disconnect()
