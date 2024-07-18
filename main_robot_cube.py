# simple script to load the KUKA robot and a cube

import pinocchio as pin
from pinocchio.visualize import MeshcatVisualizer
import sys


base_path = "robot_models/"


urdf = base_path + "/kuka/urdf/iiwa_gripper_tamp.urdf"
meshes = base_path + "/kuka/"
package_dirs = [meshes]
robot = pin.RobotWrapper.BuildFromURDF(urdf, package_dirs=package_dirs)
urdf_cube = base_path + "/objects/cube.urdf"
# we do not need package_dirs because the cube does not have any mesh
cube = pin.RobotWrapper.BuildFromURDF(urdf_cube)


viz_base = MeshcatVisualizer()
try:
    viz_base.initViewer(open=True)
except ImportError as err:
    print(
        "Error while initializing the viewer. It seems you should install Python meshcat"
    )
    print(err)
    sys.exit(0)

viz = MeshcatVisualizer(robot.model, robot.collision_model, robot.visual_model)
viz.initViewer(viz_base.viewer)
viz.loadViewerModel()

q0 = pin.neutral(robot.model)
viz.display(q0)
viz.displayVisuals(True)


viz_cube = MeshcatVisualizer(cube.model, cube.collision_model, cube.visual_model)
viz_cube.initViewer(viz_base.viewer)
viz_cube.loadViewerModel()
viz_cube.displayVisuals(True)

input("Press enter to exit")
