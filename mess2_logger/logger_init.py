#!/usr/bin/env python3

from os import makedirs, path
from rclpy.node import Node


class LoggerInitializer(Node):
    """
    """
    def __init__(self):
        """
        """
        super().__init__("mess2_logger_initializer")

        self.declare_parameter("log_dir_path", "~/Projets/mess2/logs")
        self.declare_parameter("topic_names", ["/ns/test1", "/ns/test2"])
        self.declare_parameter("period", 2.0)   # seconds

        self.log_dir_path: str = path.expanduser(self.get_parameter("log_dir_path").get_parameter_value().string_value)
        self.topic_names: list[str] = self.get_parameter("topic_names").get_parameter_value().string_array_value
        self.period: float = self.get_parameter("period").get_parameter_value().double_value

        if not path.exists(self.log_dir_path):
            makedirs(self.log_dir_path)
