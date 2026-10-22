"""Launch owns the scene, PX4, Agent and the real mission nodes."""
import os
import math
from pathlib import Path
import socket
import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess, OpaqueFunction,
    RegisterEventHandler, EmitEvent, LogInfo, IncludeLaunchDescription, SetEnvironmentVariable)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def setup(context):
    value = lambda name: LaunchConfiguration(name).perform(context)
    yes = lambda name: value(name).lower() == 'true'
    share = Path(get_package_share_directory('uav_competition'))
    project = Path(value('project_dir')).expanduser().resolve()
    site_file = Path(value('site_file')).expanduser()
    site = yaml.safe_load(site_file.read_text()) or {}
    for name in ('start_sim', 'start_agent', 'headless', 'build', 'enable_tests'):
        if value(name).lower() not in ('true', 'false'):
            raise ValueError(f'{name} must be true or false')
    if value('profile') not in ('standard', 'vm') or int(value('build_jobs')) < 1:
        raise ValueError('profile must be standard/vm; build_jobs must be positive')

    def path(setting):
        raw = value(setting) or site[setting]
        raw = str(raw).replace('@project', str(project)).replace('@package', str(share))
        result = Path(raw).expanduser()
        return result.resolve() if result.is_absolute() else (site_file.parent/result).resolve()

    world, config, interfaces = (path(k) for k in ('world_file','config_file','interface_file'))
    cfg = yaml.safe_load(config.read_text())
    if value('route') not in cfg['route_profiles']:
        raise ValueError('Selected route missing from config_file')
    if not world.is_file() or not interfaces.is_file():
        raise ValueError('world_file or interface_file does not exist')
    interface_cfg = yaml.safe_load(interfaces.read_text()) or {}
    if not isinstance(interface_cfg.get('remappings', {}), dict):
        raise ValueError('interfaces.remappings must be a mapping of ROS names')
    trajectory = cfg.get('trajectory', {})
    for key in ('corner_radius', 'sample_spacing', 'lookahead', 'max_xy_acceleration', 'corner_speed'):
        if key in trajectory and (not math.isfinite(float(trajectory[key])) or float(trajectory[key]) <= 0):
            raise ValueError(f'trajectory.{key} must be positive and finite')
    logs = Path(value('log_dir') or project/'logs/competition').expanduser().resolve()
    logs.mkdir(parents=True, exist_ok=True)
    px4, agent = path('px4_dir'), path('agent_prefix')
    model_paths = []
    for raw in site.get('model_paths', []):
        raw = str(raw).replace('@project', str(project)).replace('@package', str(share))
        p = Path(raw).expanduser()
        model_paths.append(str(p if p.is_absolute() else site_file.parent/p))
    for raw in cfg.get('map',{}).get('model_paths',[]):
        p = Path(raw).expanduser()
        model_paths.append(str(p if p.is_absolute() else config.parent/p))
    model_paths += ['/usr/share/gazebo-11/models', str(Path.home()/'.gazebo/models')]
    build = px4/'build/px4_sitl_default'
    env = dict(os.environ)
    env.update(GAZEBO_MASTER_URI=str(site.get('gazebo_master_uri','http://127.0.0.1:11345')),
        GAZEBO_IP=str(site.get('gazebo_ip','127.0.0.1')), GZ_IP=str(site.get('gazebo_ip','127.0.0.1')),
        GAZEBO_MODEL_DATABASE_URI='http://127.0.0.1:9',
        GAZEBO_MODEL_PATH=':'.join(model_paths+[env.get('GAZEBO_MODEL_PATH','')]),
        GAZEBO_RESOURCE_PATH=':'.join(model_paths+['/usr/share/gazebo-11','/usr/share/gazebo-11/media',env.get('GAZEBO_RESOURCE_PATH','')]),
        GAZEBO_PLUGIN_PATH=f"{build}/build_gazebo-classic:{env.get('GAZEBO_PLUGIN_PATH','')}",
        LD_LIBRARY_PATH=f"{build}/build_gazebo-classic:{env.get('LD_LIBRARY_PATH','')}",
        PX4_SIM_MODEL=str(site.get('sim_model','gazebo-classic_iris')),
        PX4_UXRCE_DDS_PORT=str(site.get('agent_port',8888)),
        PATH=f"{path('parameter_dir')}:{env.get('PATH','')}", QT_X11_NO_MITSHM='1')
    if value('profile') == 'vm':
        env.update(PX4_SIM_SPEED_FACTOR='1', OMP_NUM_THREADS='1', OPENBLAS_NUM_THREADS='1',
                   OPENCV_FOR_THREADS_NUM='1', MKL_NUM_THREADS='1')
    runtime = ['python3', '-m', 'uav_competition.runtime']
    actions = [SetEnvironmentVariable(k,v) for k,v in env.items() if os.environ.get(k) != v]

    def fatal(process, label):
        return RegisterEventHandler(OnProcessExit(target_action=process,
            on_exit=lambda event,ctx: [EmitEvent(event=Shutdown(reason=f'{label} exited ({event.returncode})'))]))

    def after_success(process, following):
        return RegisterEventHandler(OnProcessExit(target_action=process,
            on_exit=lambda event,ctx: following if event.returncode == 0 else [
                EmitEvent(event=Shutdown(reason=f'Preparation failed ({event.returncode})'))]))

    launch_args = {'route':value('route'), 'enable_tests':value('enable_tests'),
        'config_file':str(config), 'world_file':str(world), 'interface_file':str(interfaces), 'log_dir':str(logs/'flight')}
    business = IncludeLaunchDescription(PythonLaunchDescriptionSource(str(share/'launch/px4_regions.launch.py')),
                                       launch_arguments=launch_args.items())
    readiness = ExecuteProcess(cmd=runtime+['ready', '--interfaces', str(interfaces)], additional_env=env, output='screen')
    actions.append(RegisterEventHandler(OnProcessExit(target_action=readiness,
        on_exit=lambda event,ctx: [] if event.returncode == 0 else [
            LogInfo(msg='Readiness failed: inspect sensor/flight logs. No mission was requested.')])))
    actions.append(LogInfo(msg=f'COMPETITION config={config}; world={world}; route={value("route")}; logs={logs}'))
    if yes('start_agent'):
        executable = agent/'bin/MicroXRCEAgent'
        if not executable.is_file():
            raise ValueError(f'Agent missing: {executable}; set start_agent:=false to use an existing agent')
        # No global pkill: do not replace an unrelated instance silently.
        probe = socket.socket(socket.AF_INET,socket.SOCK_DGRAM)
        try:
            probe.bind(('0.0.0.0',int(site.get('agent_port',8888))))
        finally:
            probe.close()
        agent_env = dict(env, LD_LIBRARY_PATH=f'{agent}/lib:{os.environ.get("LD_LIBRARY_PATH","")}')
        agent_process = ExecuteProcess(cmd=[str(executable),'udp4','-p',str(site.get('agent_port',8888))],
                                       additional_env=agent_env, output='log')
        actions.extend([fatal(agent_process,'Agent'),agent_process])
    if not yes('start_sim'):
        actions.extend([business, readiness])
        return actions
    from urllib.parse import urlparse
    uri = urlparse(env['GAZEBO_MASTER_URI'])
    with socket.socket() as probe:
        if probe.connect_ex((uri.hostname or '127.0.0.1',uri.port or 11345)) == 0:
            raise ValueError('Gazebo master already running; stop the previous simulation or set start_sim:=false')
    generated = logs/'vm_course.world'
    actual_world = generated if value('profile') == 'vm' else world
    if value('profile') == 'vm' and not yes('headless'):
        gui_ini = logs/'vm-gui.ini'
        gui_ini.write_text('[geometry]\nx=0\ny=0\nwidth=960\nheight=640\n[rendering]\nstereo=0\n')
        env['GAZEBO_GUI_INI_FILE'] = str(gui_ini)
        # This variable is assigned after the common environment action list
        # is built, so add it explicitly for the current Launch process too.
        actions.append(SetEnvironmentVariable('GAZEBO_GUI_INI_FILE', str(gui_ini)))
    prepare_cmd = runtime+['prepare','--project',str(project),'--px4',str(px4),'--world',str(world),
        '--output',str(generated),'--profile',value('profile'),'--jobs',value('build_jobs')]
    if yes('build'):
        prepare_cmd.append('--build')
    if yes('headless'):
        prepare_cmd.append('--headless')
    prepare = ExecuteProcess(cmd=prepare_cmd, additional_env=env, output='screen')
    server = ExecuteProcess(cmd=['gzserver','--verbose',str(actual_world),'-s','libgazebo_ros_init.so',
        '-s','libgazebo_ros_factory.so'], additional_env=env, output='log', sigterm_timeout='5', sigkill_timeout='3')
    px4_process = ExecuteProcess(cmd=runtime+['px4','--px4',str(px4)], additional_env=env,
                                output='log', sigterm_timeout='5', sigkill_timeout='3')
    following = [fatal(server,'Gazebo server'),fatal(px4_process,'PX4'),server,px4_process,business,readiness]
    if not yes('headless'):
        following.append(ExecuteProcess(cmd=['gzclient','--verbose'], additional_env=env, output='log'))
    actions.extend([after_success(prepare,following),prepare])
    return actions


def generate_launch_description():
    share = Path(get_package_share_directory('uav_competition'))
    # Checkout wrappers pass project_dir explicitly; also discover standard
    # colcon install layouts without embedding a username.
    project = next((p for p in share.parents if (p/'PX4-Autopilot').is_dir()), Path.cwd())
    arguments = [
        ('project_dir',str(project)),('site_file',str(share/'config/site.yaml')),
        ('world_file',''),('config_file',''),('interface_file',''),('px4_dir',''),('agent_prefix',''),
        ('parameter_dir',''),('route','right'),('profile','standard'),('log_dir',''),
        ('start_sim','true'),('start_agent','true'),('headless','false'),('build','false'),
        ('build_jobs','1'),('enable_tests','false'),
    ]
    return LaunchDescription([*(DeclareLaunchArgument(k,default_value=v) for k,v in arguments),
                              OpaqueFunction(function=setup)])
