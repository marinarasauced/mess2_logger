
from launch import Action, LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from mess2_logger.trial import get_trial_dir_path


launch_args = [
    DeclareLaunchArgument(
        'namespace',
        default_value='',
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
    topic_names = []
    for value in topic_names_arg.split(","):
        topic_name: str = value.strip()
        b0: bool = namespace_arg == ''
        b1: bool = namespace_arg[0] == "/" if not b0 else False
        b2: bool = topic_name[0] == "/"
        b3: bool = topic_name.count("/") == 0
        b4: bool = topic_name.count("/") == 1
        b5: bool = topic_name.count("/") == 2

        # topic
        if b0 and not b2 and b3:
            topic_names.append(f"/{topic_name}")

        # /topic
        elif b0 and b2 and b4:
            topic_names.append(f"{topic_name}")

        # /ns + topic
        elif not b0 and b1 and not b2 and b3:
            topic_names.append(f"{namespace_arg}/{topic_name}")

        # /ns + /topic
        elif not b0 and b1 and b2 and b4:
            topic_names.append(f"{namespace_arg}{topic_name}")

        # ns + topic
        elif not b0 and not b1 and not b2 and b3:
            topic_names.append(f"/{namespace_arg}/{topic_name}")

        # ns + /topic
        elif not b0 and not b1 and b2 and b4:
            print(f"/{namespace_arg}{topic_name}")
            topic_names.append(f"/{namespace_arg}{topic_name}")

        # /ns/topic
        elif b2 and b5:
            topic_names.append(f"{topic_name}")

        # ns/topic
        elif not b2 and b4:
            topic_names.append(f"/{topic_name}")
        
        # error
        else:
            raise RuntimeError(f"Encountered unexpected topic name : {topic_name}")

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
