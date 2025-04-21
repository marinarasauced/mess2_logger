#!/usr/bin/env python3

from os import path

from mess2_logger.msg_logger import Logger
from mess2_logger.msg_parser import parse_msg
from rclpy.node import Node
from ros2topic.api import get_msg_class


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
        self.logger = Logger(self.log_file_path)
        
        self.get_logger().info(f"Checking if topic {self.topic_name} is advertised")

        self.topic_type = get_msg_class(self, self.topic_name)
        if self.topic_type is not None:
            self.is_advertised()
        else:
            self.get_logger().info(f"Topic {self.topic_name} is not advertised, checking again in {self.period} seconds")
            self.timer = self.create_timer(self.period, self.is_not_advertised)


    def is_not_advertised(self) -> None:
        """
        """
        self.get_logger().info(f"Topic {self.topic_name} is not advertised, checking again in {self.period} seconds")

        self.topic_type = get_msg_class(self, self.topic_name)
        if self.topic_type is not None:
            self.timer.destroy()
            self.is_advertised()


    def is_advertised(self) -> None:
        """
        """
        self.get_logger().info(f"Topic {self.topic_name} is advertised, parsing message")

        modules = self.topic_type.__module__.split(".")
        msg_pkg = modules[0]
        msg_type = self.topic_type.__name__
        self.topic_map: list[str] = parse_msg("msg", f"{msg_pkg}/msg/{msg_type}")

        self.get_logger().info(f"Parsed {msg_pkg}/msg/{msg_type}, creating subscription to {self.topic_name}")

        self.logger.log(self.topic_map)
        self.subscription = self.create_subscription(
            msg_type=self.topic_type,
            topic=self.topic_name,
            qos_profile=10,
            callback=self.callback
        )

        self.get_logger().info(f"Created subscription to {self.topic_name}")


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
        if self.logger:
            del self.logger
