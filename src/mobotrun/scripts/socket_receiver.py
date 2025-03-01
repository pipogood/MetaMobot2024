import socketio
import asyncio
import time

sio = socketio.AsyncClient(
    reconnection=True, 
    reconnection_attempts=10000, 
    reconnection_delay=0.1,
    reconnection_delay_max = 5.0, 
    ssl_verify=False,
    logger = True)

@sio.event()
async def connect():
    print("Connected to the server")

@sio.on('chat_message')
async def chat_message(data):
    print("Chat message received:", data)

@sio.event
async def disconnect():
    print("Disconnected from the server")

async def main():
    try:
        await sio.connect("https://34.126.126.56:8002/")
        await sio.wait()
    except Exception as e:  
        print(f"Connection failed: {e}")
    finally:
        await sio.disconnect()

if __name__ == "__main__":
    asyncio.run(main())