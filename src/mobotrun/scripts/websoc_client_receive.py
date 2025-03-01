#!/usr/bin/python3

import asyncio
import websockets

async def receive_message():
    async with websockets.connect("ws://192.168.137.1:8765") as websocket:
        message = await websocket.recv()
        print(f"Received: {message}")

if __name__ == "__main__":
    asyncio.run(receive_message())
