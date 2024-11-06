from asyncua import Server, ua
import rclpy
from rclpy.node import Node
import asyncio
from std_msgs.msg import Float32

class ROS2Subscriber(Node):
    def __init__(self, server_variable):
        super().__init__('ros2_subscriber')
        self.server_variable = server_variable
        self.subscription = self.create_subscription(
            Float32, 'random_number', self.listener_callback, 10
        )

    async def listener_callback(self, msg):
        # Update the OPC UA variable with the new value from ROS 2
        self.get_logger().info(f"Received data: {msg.data}")
        await self.server_variable.write_value(msg.data)

async def start_opcua_server():
    # Initialize the OPC UA server
    server = Server()
    await server.init()
    server.set_endpoint("opc.tcp://localhost:4840")
    server.set_security_policy([ua.SecurityPolicyType.NoSecurity])

    # Register namespace and create OPC UA variable
    uri = "http://your-opcua-namespace.com"
    idx = await server.register_namespace(uri)
    objects_node = server.nodes.objects
    ros_interface = await objects_node.add_object(idx, "ROS_Interface")

    # Create a variable to hold the random number received from ROS 2
    random_number_var = await ros_interface.add_variable(idx, "RandomNumber", 0.0)
    await random_number_var.set_writable()  # Make it writable so we can update it

    # Initialize ROS 2 and create the ROS2Subscriber node
    rclpy.init()
    ros2_subscriber_node = ROS2Subscriber(random_number_var)

    # Run the OPC UA server and ROS 2 node in parallel
    async with server:
        print("OPC UA Server with ROS 2 integration running.")
        while rclpy.ok():
            rclpy.spin_once(ros2_subscriber_node, timeout_sec=0.1)
            await asyncio.sleep(0.1)  # Allow the server to process other tasks

    # Shutdown ROS 2 when the server stops
    ros2_subscriber_node.destroy_node()
    rclpy.shutdown()

# Run the OPC UA server
try:
    asyncio.run(start_opcua_server())
except KeyboardInterrupt:
    print("Server stopped.")
