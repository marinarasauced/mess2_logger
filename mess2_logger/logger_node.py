#!/usr/bin/env python3

import contextlib
import io
from os import path

from mess2_logger.msg_logger import Logger
from mess2_logger.msg_parser import parse_msg
from rclpy.node import Node
from ros2topic.api import get_msg_class


def suppress_ros2topic_api_warning():
    """
    """
    with contextlib.redirect_stderr(io.StringIO()):
        yield


class LoggerNode(Node):
    """
    """
    def __init__(self, log_dir_path: str, node_name: str, topic_name: str, period: float = 2.0):
        """
        """
        super().__init__(node_name)
        self.log_dir_path = log_dir_path
        self.topic_name = topic_name
        self.period = period

        self.log_file_path = path.abspath(path.join(self.log_dir_path, f"{self.topic_name[1:].replace("/", "_")}.msg"))
        self.topic_type = get_msg_class(self, self.topic_name)
        if self.topic_type is not None:
            self.is_advertised()
        else:
            self.timer = self.create_timer(self.period, self.is_not_advertised)


    def is_not_advertised(self) -> None:
        """
        """
        self.topic_type = get_msg_class(self, self.topic_name)
        if self.topic_type is not None:
            self.timer.destroy()
            self.is_advertised()


    def is_advertised(self) -> None:
        """
        """
        modules = self.topic_type.__module__.split(".")
        msg_pkg = modules[0]
        msg_type = self.topic_type.__name__
        self.topic_map: list[str] = parse_msg("msg", f"{msg_pkg}/msg/{msg_type}")

        self.logger = Logger(self.log_file_path)
        self.logger.log(self.topic_map)
        self.subscription = self.create_subscription(
            msg_type=self.topic_type,
            topic=self.topic_name,
            qos_profile=10,
            callback=self.callback
        )


    def callback(self, msg: any) -> None:
        """
        """
        data = []
        for field in self.topic_map:
            try:
                value = str(eval(field))
                if value.startswith("(") and value.endswith(")"):
                    value.replace(",", "")
                data.append(value)
            except Exception as e:
                data.append("")
                self.get_logger().info(data)
        self.logger.log(data)



    def destroy_node(self) -> None:
        """
        """
