import os
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_path
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def add_sensors_to_sdf(robot_sdf_file, sensors_file):

    tree_robot = ET.parse(robot_sdf_file)
    root_robot = tree_robot.getroot()

    tree_sensors = ET.parse(sensors_file)
    root_sensors = tree_sensors.getroot()

    for link_robot in root_robot.iter('link'):
        for link_sensor in root_sensors.iter('link'):
            if(link_robot.attrib['name'] == link_sensor.attrib['name']):
                for sensor in link_sensor:
                    link_robot.append(sensor)

    tree_robot.write(robot_sdf_file)

def generate_launch_description():

    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    #pkg_project_gazebo = get_package_share_directory('rober_base_test1')

    rover_base_path = get_package_share_path('rover_base_test1')
    #rover_base_path = "/home/aymeric/Documents/Programmation/ROS/rover_ws/src/rover_base_test1"
    default_model_path = str(rover_base_path) + '/urdf/rover_base2.xacro'
    urdf_model_path = str(rover_base_path) + '/urdf/rover_base2.urdf'
    sdf_model_path = str(rover_base_path) + '/urdf/rover_base2.sdf'
    sensors_model_path = str(rover_base_path) + '/urdf/sensors.xml'
    default_rviz_config_path = str(rover_base_path) + '/rviz/urdf.rviz'

    #model_file = str(rover_base_path) + "/gazebo/models/rover_base2/rover_base2.sdf" #Valide
    #model_file = str(rover_base_path) + "/urdf/rover_base3.urdf"
    model_file = str(rover_base_path) + "/urdf/rover_base3.urdf"
    world_file = str(rover_base_path) + "/gazebo/world1.sdf"

    gui_arg = DeclareLaunchArgument(name='gui', default_value='true', choices=['true', 'false'],
                                    description='Flag to enable joint_state_publisher_gui')
    model_arg = DeclareLaunchArgument(name='model', default_value=str(model_file),
                                      description='Absolute path to robot urdf file')
    rviz_arg = DeclareLaunchArgument(name='rvizconfig', default_value=str(default_rviz_config_path),
                                     description='Absolute path to rviz config file')

    #Strategie pour obtenir un robot dans gazebo avec ses capteurs à partir d'un xacro: 
    # - convertir le xacro en urdf
    # - convertir l'urdf en sdf
    # - utiliser une fonction de parsing XML pour ajouter les capteurs désirés au fichier sdf(la conversion urdf->sdf les supprime automatiquement)
    # - inclure le fichier obtenu dans le monde sdf à charger et exécuter le noeud qui lance gazebo

    #Convertit le fichier xacro en fichier urdf
    #robot_description = ParameterValue(Command([LaunchConfiguration('model')]), value_type=str)
    desc = os.popen('xacro -o ' + str(urdf_model_path) + ' ' + str(default_model_path))
    robot_urdf = desc.read()
    #desc.read() : get the description ; ne fonctionne qu'une seule fois
    #Command(['xacro -o ', str(written_model_path), ' ', str(default_model_path)])
    robot_description = ParameterValue(Command(['xacro ', str(default_model_path)]))

    #Convertit l'urdf en sdf
    desc = os.popen('ign sdf -p ' + str(urdf_model_path))
    robot_sdf = desc.read()
    with open(sdf_model_path, 'w') as infp:
        infp.write(str(robot_sdf))

    #Ajout des capteurs
    add_sensors_to_sdf(sdf_model_path, sensors_model_path)

    #with open(written_model_path, 'w') as infp:
    #    infp.write(str(robot_desc))

    # Load the SDF file from "description" package
    #pkg_project_description = get_package_share_directory('ros_gz_example_description')
    #sdf_file  =  os.path.join(pkg_project_description, 'models', 'diff_drive', 'model.sdf')
    
    #with open(model_file, 'r') as infp:
    #    robot_description = infp.read()

    #print(robot_description)

    #with open(default_model_path, 'r') as infp:
    #    robot_description = infp.read()
    
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[
            {'use_sim_time': True},
            {'robot_description': robot_description},
            ]
    )

    # Depending on gui parameter, either launch joint_state_publisher or joint_state_publisher_gui
    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        condition=UnlessCondition(LaunchConfiguration('gui'))
    )

    joint_state_publisher_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        condition=IfCondition(LaunchConfiguration('gui'))
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rvizconfig')],
    )

    #gz_sim = ExecuteProcess(
    #    cmd=['gazebo', '--verbose', '-s', 'libgazebo_ros_factory.so',
    #     world_file], #autre extension : .world
    #    output='screen'
    #)

    gz_sim = ExecuteProcess(
        cmd=['ign', 'gazebo', '--force-version', '6', world_file], #autre extension : .world
        output='screen'
    ) 

    # Setup to launch the simulator and Gazebo world
    #gz_sim = IncludeLaunchDescription(
    #    PythonLaunchDescriptionSource(
    #        os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')),
    #    #launch_arguments={'gz_args': PathJoinSubstitution([
    #    #    pkg_project_gazebo,
    #    #    'gazebo',
    #    #    'world1.sdf'
    #    #])}.items(),
    #    launch_arguments={'gz_args': world_file}.items(),
    #)

    #Possibilité de remplacer 'gazebo' par gzserver (gzserver : calculs et gzclient : rendu)

    #spawn_entity = Node(package='gazebo_ros', node_executable='spawn_entity.py',
    #            arguments=['-entity', 'mulecar', '-file', 'gazebo/rover_base.sdf'],
    #            output='screen')
    
    #bridge = Node(
    #        package='ros_gz_bridge',
    #        #namespace='ros_gz_bridge',
    #        executable='parameter_bridge',
    #        name='bridge_keyboard',
    #        arguments=['/keyboard/keypress@std_msgs/msg/Int32@ignition.msgs.Int32']
    #)

    # Bridge ROS topics and Gazebo messages for establishing communication
    bridge_file = str(rover_base_path) + "/config/ros_gz_bridge.yaml"
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{
            'config_file': bridge_file, #os.path.join(pkg_project_bringup, 'config', 'ros_gz_example_bridge.yaml'),
            'qos_overrides./tf_static.publisher.durability': 'transient_local',
        }],
        output='screen'
    )

    teleop_node_play = IncludeLaunchDescription(
      PythonLaunchDescriptionSource([os.path.join(
         get_package_share_directory('teleop'), 'launch'),
         '/teleop_launch.py'])
      )

    return LaunchDescription([
        gui_arg,
        model_arg,
        rviz_arg,
        joint_state_publisher_node,
        joint_state_publisher_gui_node,
        robot_state_publisher_node,
        rviz_node,
        gz_sim,
        bridge,
        teleop_node_play,
    ])