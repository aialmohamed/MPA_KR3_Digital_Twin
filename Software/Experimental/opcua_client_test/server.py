from asyncua import Server, ua, uamethod
import rclpy
from rclpy.node import Node
import asyncio
from std_msgs.msg import String


rclpy.init()
ros2_publisher_node = Node("ros2_publisher")
publisher_ = ros2_publisher_node.create_publisher(String, 'example_topic', 10)

@uamethod
def publish_opcua_message(parent, message):

    msg = String()
    msg.data = message
    publisher_.publish(msg)
    ros2_publisher_node.get_logger().info(f"Published message on /example_topic: {msg.data}")
    return True  


async def start_opcua_server():
    server = Server()
    await server.init()
    server.set_endpoint("opc.tcp://localhost:4840")
    server.set_security_policy([ua.SecurityPolicyType.NoSecurity])


    uri = "http://your-opcua-namespace.com"
    idx = await server.register_namespace(uri)
    objects_node = server.nodes.objects
    ros_interface = await objects_node.add_object(idx, "ROS_Interface")


    await ros_interface.add_method(
        idx, "PublishMessage", publish_opcua_message, [ua.VariantType.String], [ua.VariantType.Boolean]
    )

    async with server:
        print("OPC UA Server with ROS 2 integration running.")
        while rclpy.ok():
            await asyncio.sleep(1)
try:
    asyncio.run(start_opcua_server())
finally:
    rclpy.shutdown()
