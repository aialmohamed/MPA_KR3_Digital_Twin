import asyncio
import random
from asyncua import Server, ua

async def main():
    # Create the server
    server = Server()
    await server.init()
    server.set_endpoint("opc.tcp://0.0.0.0:4840")

    # Set server namespace
    uri = "http://example.org"
    idx = await server.register_namespace(uri)

    # Create a "Temperature" variable
    temperature_node = await server.nodes.objects.add_variable(
        ua.NodeId("Temperature", idx), "Temperature", 0.0
    )
    await temperature_node.set_writable()  # Allow clients to write to this variable

    async def square_method(parent, value):
        try:
            # Extract the actual value from the Variant object
            actual_value = value.Value
            if isinstance(actual_value, (int, float)):
                result = actual_value ** 2
                return [ua.Variant(result, ua.VariantType.Double)]  # OPC UA expects a list of return values
            else:
                raise ValueError("Input value must be a number")
        except Exception as e:
            print(f"Error in square_method: {e}")
            raise

    # Add method to the server
    await server.nodes.objects.add_method(
        ua.NodeId("Square", idx), "Square", square_method, [ua.VariantType.Double], [ua.VariantType.Double]
    )

    # Start the server
    async with server:
        print("Server is running at opc.tcp://0.0.0.0:4840/freeopcua/server/")
        while True:
            # Randomly update the temperature variable
            new_temperature = round(random.uniform(20.0, 30.0), 2)
            await temperature_node.write_value(new_temperature)
            print(f"Updated Temperature: {new_temperature}")
            await asyncio.sleep(2)  # Update every 2 seconds

# Run the server
if __name__ == "__main__":
    asyncio.run(main())
