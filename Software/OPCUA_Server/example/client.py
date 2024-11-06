from asyncua import Client
import asyncio

async def test_connection():
    url = "opc.tcp://0.0.0.0:4840"  # Adjust if your server has a specific path
    client = Client(url)
    try:
        await client.connect()
        print("Connection successful!")
    except Exception as e:
        print(f"Connection failed: {e}")
    finally:
        await client.disconnect()

asyncio.run(test_connection())
