import rclpy
from rclpy.node import Node
from rclpy.logging import get_logger

class Planner(Node):
    def __init__(self):
        super().__init__('planner')

def main(args=None):
    logger = get_logger("logger")
    logger.info("Planner started")
    rclpy.init(args=args)
    rclpy.spin(Planner())
    rclpy.shutdown()
