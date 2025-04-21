#!/usr/bin/env python3

import rclpy
from rclpy.executors import MultiThreadedExecutor

from mess2_logger.logger_init import LoggerInitializer
from mess2_logger.logger_node import LoggerNode


def main(args=None):
    """
    """
    rclpy.init(args=args)
    initializer = LoggerInitializer()

    container: list[LoggerNode] = []
    for topic_name in initializer.topic_names:
        clean_name = topic_name.strip("/").replace("/", "_")
        node_name = f"mess2_logger_{clean_name}"
        container.append(LoggerNode(
            log_dir_path=initializer.log_dir_path, 
            node_name=node_name,
            topic_name=topic_name,
            period=initializer.period
        ))
    
    initializer.destroy_node()

    executor = MultiThreadedExecutor(num_threads=2)
    for node in container:
        executor.add_node(node)
    
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        for node in container:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__=="__main__":
    main()
