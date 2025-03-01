#!/usr/bin/python3

import webbrowser
import paho.mqtt.client as mqtt

# MQTT Broker configuration
BROKER = "broker.hivemq.com"  # Replace with your MQTT broker address
PORT = 1883                    # Use 8883 if using TLS
TOPIC = "trigger/open_browser" # Replace with your topic name

# Callback function when connected to the MQTT broker
def on_connect(client, userdata, flags, rc):
    if rc == 0:
        print("Connected to MQTT Broker!")
        # Subscribe to the topic
        client.subscribe(TOPIC)
    else:
        print("Failed to connect, return code:", rc)

# Callback function when a message is received
def on_message(client, userdata, msg):
    print(f"Message received on topic {msg.topic}: {msg.payload.decode()}")
        # Open the web browser
    webbrowser.open(msg.payload.decode())
    # if msg.payload.decode().strip().lower() == "open":
    #     # Open the web browser
    #     webbrowser.open("https://34.126.126.56:8000/")

# Set up the MQTT client
client = mqtt.Client(client_id = 'fibo1234')
client.on_connect = on_connect
client.on_message = on_message

# Connect to the MQTT broker
try:
    client.connect(BROKER, PORT)
except Exception as e:
    print(f"Error connecting to MQTT Broker: {e}")
    exit(1)

# Start the MQTT loop
print("Listening for messages...")
client.loop_forever()
