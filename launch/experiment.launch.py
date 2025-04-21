
from launch import Action, LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from mess2_logger.trial import get_trial_dir_path


launch_args = [
    DeclareLaunchArgument(
        'namespace',
        default_value='mess2',
        description='The namespace of the topics that are to be logged.'
    ),
    DeclareLaunchArgument(
        'log_dir_path',
        default_value='~/Projets/mess2/logs',
        description='The relative path to the directory where the topics are to be logged.'
    ),
    DeclareLaunchArgument(
        'topic_names',
        default_value='test1, test2',
        description='A list of the topics that are to be logged.'
    ),
    DeclareLaunchArgument(
        'period',
        default_value='5.0',
        description='The period of the timer that checks whether a topic is advertised or not.'
    )
]


def launch_setup(context) -> list[Action]:
    """
    Performs launch setup including namespace substitution into topic list.
    """
    namespace_arg = LaunchConfiguration('namespace').perform(context).strip()
    topic_names_arg = LaunchConfiguration('topic_names').perform(context)
    topic_names = [
        f"/{namespace_arg}/{topic_name.strip()}" if namespace_arg else f"/{topic_name.strip()}"
        for topic_name in topic_names_arg.split(',')
        if topic_name.strip()
    ]
    trial_dir_path = get_trial_dir_path(log_dir_path=LaunchConfiguration('log_dir_path').perform(context).strip())

    node = Node(
        package='mess2_logger',
        executable='main.py',
        namespace=LaunchConfiguration('namespace'),
        parameters=[
            {'log_dir_path': trial_dir_path},
            {'topic_names': topic_names},
            {'period': LaunchConfiguration('period')},
        ]
    )
    return [node]


def generate_launch_description() -> LaunchDescription:
    """
    Generates the launch description for the mess2_logger initializer node with parameters.
    """
    opfunc = OpaqueFunction(function=launch_setup)
    ld = LaunchDescription(launch_args)
    ld.add_action(opfunc)
    return ld
