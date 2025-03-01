import paho.mqtt.client as mqtt
import time
import ssl

def my_on_connect(client:mqtt.Client, userdata, connect_flags, reason_code, properties):

    print(f"my_on_connect({client}, {userdata}, {connect_flags}, {reason_code}, {properties})")

    client.subscribe(topic="fibo/log", qos=2)

def my_on_message(client, userdata, message:mqtt.MQTTMessage):

    print("my_on_message({client}, {userdata}, {message})")

    print(f"=>{message.topic} => {str(message.payload)}")

client = mqtt.Client(callback_api_version=mqtt.CallbackAPIVersion.VERSION2, client_id=f"TEST-ID-{time.time}", transport="websockets")
client.ws_set_options(path="/mqtt")
client.tls_set(ca_certs=None, cert_reqs=ssl.CERT_REQUIRED)

client.on_connect = my_on_connect
client.on_message = my_on_message

client.connect_async(host="test.mosquitto.org", port=8081, keepalive=5)
client.loop_start()

while(True):

    print(".", end="")
    time.sleep(1)
