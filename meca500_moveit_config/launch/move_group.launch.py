import os
import yaml
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory

def load_file(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)
    with open(absolute_file_path, 'r') as file:
        return file.read()

def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)
    with open(absolute_file_path, 'r') as file:
        return yaml.safe_load(file)

def generate_launch_description():
    
    # 1. Load the URDF from your description package
    robot_description_content = ParameterValue(
        Command([
            FindExecutable(name="xacro"), " ",
            PathJoinSubstitution([FindPackageShare("meca500_description"), "urdf", "meca500.urdf.xacro"])
        ]),
        value_type=str
    )
    robot_description = {"robot_description": robot_description_content}

    # 2. Load the SRDF
    robot_description_semantic = {
        "robot_description_semantic": load_file("meca500_moveit_config", "config/meca500.srdf")
    }

    # 3. Load Kinematics
    robot_description_kinematics = {
        "robot_description_kinematics": load_yaml("meca500_moveit_config", "config/kinematics.yaml")
    }
    
    # 3.5 Load Joint Limits (Crucial for time parameterization!)
    robot_description_planning = {
        "robot_description_planning": load_yaml("meca500_moveit_config", "config/joint_limits.yaml")
    }
    
    # 4. Load OMPL Planning Pipeline (This calculates the timestamps!)
    ompl_yaml = load_yaml("meca500_moveit_config", "config/ompl_planning.yaml")
    planning_pipeline = {
        "planning_pipelines": ["ompl"],
        "default_planning_pipeline": "ompl",
        "ompl": ompl_yaml
    }

    # 4. Load Controllers configuration
    controllers_yaml = load_yaml("meca500_moveit_config", "config/moveit_controllers.yaml")
    moveit_controllers = {
        "moveit_controller_manager": "moveit_simple_controller_manager/MoveItSimpleControllerManager",
        "moveit_simple_controller_manager": controllers_yaml["moveit_simple_controller_manager"],
    }

    # 5. Trajectory execution settings — prevent premature timeout on real hardware
    trajectory_execution = {
        "trajectory_execution": {
            "allowed_execution_duration_scaling": 1.5,
            "allowed_goal_duration_margin": 1.0,
            "execution_duration_monitoring": True,
        }
    }

    # 6. Define the Move Group Node
    run_move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            robot_description,
            robot_description_semantic,
            robot_description_kinematics,
            robot_description_planning,
            planning_pipeline,
            moveit_controllers,
            trajectory_execution,
            {"use_sim_time": False},
        ],
    )

    # 7. Start RViz2 with MoveIt parameters
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        parameters=[
            robot_description,
            robot_description_semantic,
            robot_description_kinematics,
        ],
    )

    return LaunchDescription([
        run_move_group_node,
        rviz_node
    ])