#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from asyncua import ua, uamethod, Server


class BascOPCUA(Node):
    def __init__(self):
        super().__init__('basic_opcua_server')



def main(args=None):
    rclpy.init(args=args)

    basic_opcua_server = BascOPCUA()

    rclpy.spin(basic_opcua_server)

    basic_opcua_server.destroy_node()
    rclpy.shutdown()