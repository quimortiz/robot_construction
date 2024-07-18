# simple script to load the KUKA robot and a cube
# Here, we combine the robot and the cube in the same model.

import pinocchio as pin
from pinocchio.visualize import MeshcatVisualizer
import sys
from robot_descriptions.loaders.pinocchio import load_robot_description
import numpy as np
import hppfcl as fcl

# robot = load_robot_description("iiwa14_description")


def custom_configuration_vector(model: pin.Model, **kwargs) -> np.ndarray:
    """Generate a configuration vector where named joints have specific values.

    Args:
        robot: Robot model.
        kwargs: Custom values for joint coordinates.

    Returns:
        Configuration vector where named joints have the values specified in
        keyword arguments, and other joints have their neutral value.
    """
    q = pin.neutral(model)
    for joint_name, joint_value in kwargs.items():
        joint_id = model.getJointId(joint_name)
        joint = model.joints[joint_id]
        if type(joint_value) == np.ndarray:
            # assert len(joint_value) == joint.nq
            for i in range(joint.nq):
                q[joint.idx_q + i] = joint_value[i]
        else:
            q[joint.idx_q] = joint_value
    return q



base_path = "robot_models/"


urdf = base_path + "/kuka/urdf/iiwa_gripper_tamp.urdf"
meshes = base_path + "/kuka/"
package_dirs = [meshes]
robot = pin.RobotWrapper.BuildFromURDF(urdf, package_dirs=package_dirs)


urdf_cube = base_path + "/objects/cube.urdf"

empty_model = pin.Model()
empty_geom_model = pin.GeometryModel()
cube = pin.RobotWrapper.BuildFromURDF(urdf_cube)


# we need to rename the frames of the cube to avoid conflicts with the robot
for i in range(len(cube.model.frames)):
    cube.model.frames[i].name += "_c"


robot_with_cube, robot_with_cube_col = pin.appendModel(
    robot.model,
    cube.model,
    robot.collision_model,
    cube.collision_model,
    0,
    pin.SE3.Identity(),
)

_, robot_with_cube_visual = pin.appendModel(
    robot.model,
    cube.model,
    robot.visual_model,
    cube.visual_model,
    0,
    pin.SE3.Identity(),
)

robot_with_cube = pin.RobotWrapper(robot_with_cube, collision_model=robot_with_cube_col, visual_model=robot_with_cube_visual)

viz_base = MeshcatVisualizer()
try:
    viz_base.initViewer(open=True)
except ImportError as err:
    print(
        "Error while initializing the viewer. It seems you should install Python meshcat"
    )
    print(err)
    sys.exit(0)

viz = MeshcatVisualizer(
    robot_with_cube.model, robot_with_cube.collision_model, robot_with_cube.visual_model)

viz.initViewer(viz_base.viewer)
viz.loadViewerModel()

q0 = pin.neutral(robot_with_cube.model)
viz.display(q0)
viz.displayVisuals(True)



input("Press enter to exit")

q1 = custom_configuration_vector(robot_with_cube.model, 
                                 A1=1)
viz.display(q1)
input("Press enter to exit")
