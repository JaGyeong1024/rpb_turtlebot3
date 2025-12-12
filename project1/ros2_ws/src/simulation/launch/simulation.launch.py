import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable, TimerAction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, TextSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition, UnlessCondition # [중요] 조건부 실행을 위해 추가

def generate_launch_description():
    # 1. Launch Arguments 설정

    # 5) gui 인자: true/false (기본값 true)
    gui_arg = DeclareLaunchArgument(
        name='gui',
        default_value='true',
        description='Run Gazebo UI'
    )

    # 6) world 인자: 월드 파일 이름 입력 (기본값 default.sdf)
    world_arg = DeclareLaunchArgument(
        name='world',
        default_value='default.sdf',
        description='World file name'
    )

    # minibot 패턴 참고: prefix 인자 추가
    prefix_arg = DeclareLaunchArgument("prefix", default_value="")

    # 2. 경로 설정
    pkg_simulation = FindPackageShare('simulation')
    pkg_ros_gz_sim = FindPackageShare('ros_gz_sim')

    # 월드 파일 경로를 인자(world)에 따라 동적으로 설정
    world_file_path = PathJoinSubstitution([pkg_simulation, 'worlds', LaunchConfiguration('world')])

    server_config_path = PathJoinSubstitution([pkg_simulation, 'config', 'server.config'])
    server_config_env = SetEnvironmentVariable('GZ_SIM_SERVER_CONFIG_PATH', value=server_config_path)

    # GZ_SIM_RESOURCE_PATH를 올바르게 설정
    # 예제 코드 참고: 시스템 기본 경로와 우리 모델 경로를 모두 포함
    # pkg_simulation을 재사용하여 중복 호출 방지
    gz_resource_path = PathJoinSubstitution([pkg_simulation, 'models'])
    # SetEnvironmentVariable을 사용하여 명시적으로 모든 경로 설정
    # 시스템 기본 경로(/opt/ros/jazzy/share)와 우리 모델 경로를 모두 포함
    # run_docker.sh에서 설정된 값은 런치 파일 실행 시 덮어씀 (명시적 설정이 우선)
    gz_resource_env = SetEnvironmentVariable(
        'GZ_SIM_RESOURCE_PATH',
        [TextSubstitution(text='/opt/ros/jazzy/share'), TextSubstitution(text=':'), gz_resource_path]
    )
    
    # Gazebo Transport 환경변수 설정 (GUI-서버 통신용)
    gz_ip_env = SetEnvironmentVariable('GZ_IP', '127.0.0.1')
    gz_relay_env = SetEnvironmentVariable('GZ_RELAY', '127.0.0.1')
    
    # MESA/Vulkan 오류 방지 (Ubuntu 24.04 Docker 컨테이너에서 ZINK 오류 해결)
    # 참고: psd24_simulator는 MESA 환경변수를 전혀 설정하지 않음
    # 많은 MESA 환경변수는 OpenGL 컨텍스트 문제를 유발할 수 있으므로 최소화
    # Vulkan만 비활성화 (ZINK 오류 방지)
    vulkan_icd_env = SetEnvironmentVariable('VK_ICD_FILENAMES', '')

    bridge_config_path = PathJoinSubstitution([pkg_simulation, 'config', 'bridge.yaml'])
    # 3. Gazebo 실행 로직 (GUI 조건부 분기)

    # Case A: gui가 true일 때 (화면 켜기) -> -r (run) 옵션만 사용
    # GUI 설정 파일을 명시적으로 지정하여 사용자가 수정한 설정이 적용되지 않도록 함
    # 기본 GUI 설정 파일 사용 (model.config 참조 없음)
    # 렌더링 엔진을 OpenGL로 명시적으로 지정 (MESA/Vulkan 오류 방지)
    default_gui_config = '/usr/share/gz/gz-sim8/gui/gui.config'
    # gz_args는 리스트로 전달되면 각 요소가 공백으로 결합됨
    # 각 옵션과 값 사이에 공백이 필요하므로 명시적으로 포함
    gazebo_gui = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py'])),
        launch_arguments={
            'gz_args': [
                TextSubstitution(text='-r -v 4 '),
                TextSubstitution(text='--render-engine-gui-api-backend '),
                TextSubstitution(text='opengl '),
                TextSubstitution(text='--gui-config '),
                TextSubstitution(text=default_gui_config),
                TextSubstitution(text=' '),
                world_file_path
            ],
            'on_exit_shutdown': 'true'
        }.items(),
        condition=IfCondition(LaunchConfiguration('gui'))
    )

    # Case B: gui가 false일 때 (화면 끄기/Headless) -> -s (server only) -r (run) 옵션 사용
    gazebo_headless = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py'])),
        launch_arguments={
            'gz_args': [TextSubstitution(text='-s -r -v 4 '), world_file_path],
            'on_exit_shutdown': 'true'
        }.items(),
        condition=UnlessCondition(LaunchConfiguration('gui'))
    )

    # 4. 로봇 Description 및 Spawner
    # minibot 패턴 참고: upload_robot을 별도 launch로 분리
    upload_robot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([pkg_simulation, 'launch', 'upload_robot.launch.py'])),
        launch_arguments={
            'prefix': LaunchConfiguration('prefix')
        }.items()
    )

    # 로봇 스폰 (robot_description 토픽이 발행된 후 실행되도록 지연)
    # ros_gz_sim의 create 노드를 직접 사용 (gz_spawn_model.launch.py가 없을 수 있음)
    urdf_spawner = TimerAction(
        period=3.0,
        actions=[
            Node(
                package='ros_gz_sim',
                executable='create',
                name='spawn_robot',
                arguments=[
                    '-world', 'default',
                    '-topic', '/robot_description',
                    '-name', 'robot',
                    '-x', '1.2',
                    '-y', '-28.5',
                    '-z', '1.5',
                    '-Y', '1.5708',
                ],
                output='screen',
            )
        ]
    )

    bridge_node = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{
            'config_file': bridge_config_path
        }],
        output='screen',
    )

    return LaunchDescription([
        gui_arg,
        world_arg,
        prefix_arg,
        server_config_env,
        gz_resource_env,
        gz_ip_env,
        gz_relay_env,
        vulkan_icd_env,
        gazebo_gui,
        gazebo_headless,
        upload_robot,
        urdf_spawner,
        bridge_node,
    ])
