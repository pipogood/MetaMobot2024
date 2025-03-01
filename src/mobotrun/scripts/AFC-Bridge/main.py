import time
import os
import sys
import asyncio
import signal
import paho.mqtt.client
import my.mqtt
import random
from dotenv import load_dotenv
import my.udp
import queue


# === Init ===

load_dotenv()

_APP_FCY_TIME_      = 1 / int(os.environ["APP_FCY"])

_SHUTDOWN_EVENT_    = None

_DATA_CHANNEL_2_    = my.mqtt.MySecondaryDataChannel()
_COMM_INTERNAL_     = my.udp.MyInternalCommunication()

_QUEUE_MSG_         = queue.Queue()

ROBOT_ID            = None
ROBOT_MODE          = None

# === Event ===


# === Helper ===

async def my_task_mqtt():

    global _APP_FCY_TIME_, _SHUTDOWN_EVENT_, _DATA_CHANNEL_2_, _QUEUE_MSG_, ROBOT_ID, ROBOT_MODE

    TIME_UPDATE_LATE = time.time()

    try:

        # --- Event ---

        def my_event_mqtt_on_connect(client:paho.mqtt.client.Client, userdata, connect_flags, reason_code, properties):

            print("MQTT Connected")

            TOPIC_PREFIX = os.environ["MQTT_TOPIC_PREFIX"]

            client.subscribe(f"{TOPIC_PREFIX}/{ROBOT_MODE}/{ROBOT_ID}-{ROBOT_MODE}")

        def my_event_mqtt_on_message(client:paho.mqtt.client.Client, userdata, message:paho.mqtt.client.MQTTMessage):

            print(f"=>{message.topic} => {str(message.payload)}")

            try:

                _QUEUE_MSG_.put_nowait(message.payload)

            except queue.Full as e:

                print(f"! queue.Full : {e}")

        # --- Init ---

        print("MQTT ...[START]")

        ID = f"{ROBOT_ID}-{ROBOT_MODE}"
        HOST =os.environ["MQTT_HOST"]
        PORT = int(os.environ["MQTT_PORT"])

        _DATA_CHANNEL_2_.my_init(id=ID, callback_on_connect=my_event_mqtt_on_connect, callback_on_message=my_event_mqtt_on_message)
        
        print(f"MQTT - Connecting to {HOST}:{PORT} using ID={ID}...")

        _DATA_CHANNEL_2_.my_connect(host=HOST, port=PORT)

        while (_SHUTDOWN_EVENT_ != True):

            await asyncio.sleep(_APP_FCY_TIME_)

            if time.time() - TIME_UPDATE_LATE > 1 :

                print("Running ...[MQTT]")
                TIME_UPDATE_LATE = time.time()

        print("MQTT - Shutting down...")

        _DATA_CHANNEL_2_.my_release()

    except asyncio.CancelledError as e:

        print(f"\n! asyncio.CancelledError (MQTT) : {e}")

    except BaseException as e:

        print(f"\n! BaseException (MQTT) : {e}")

    finally:

        print("MQTT ...[END]")

    return None

async def my_task_udp():

    global _APP_FCY_TIME_, _SHUTDOWN_EVENT_, _COMM_INTERNAL_, _QUEUE_MSG_

    TIME_DISPLAY_LAST = time.time()

    try:

        # --- Init ---

        print("UDP ...[START]")

        HOST = os.environ["UDP_HOST"]
        PORT = int(os.environ["UDP_PORT"])
        

        print(f"UDP - Internal destination is {HOST}:{PORT}")

        _COMM_INTERNAL_.my_init(host=HOST, port=PORT)

        while (_SHUTDOWN_EVENT_ != True):

            time_begin = time.time()

            try:

                data = _QUEUE_MSG_.get_nowait()
                size = _QUEUE_MSG_.qsize()

                print(f"<=({size}) <= {data}")

                _COMM_INTERNAL_.my_send(data)

            except queue.Empty as e:

                pass

            time_end = time.time()
            time_dv = _APP_FCY_TIME_ - (time_end - time_begin)

            await asyncio.sleep(0 if time_dv < 0 else time_dv)

            if time.time() - TIME_DISPLAY_LAST > 1 :

                print(f"Running ({time_dv}) ...[UDP]")

                TIME_DISPLAY_LAST = time.time()

        print("UDP - Shutting down...")
        
        _DATA_CHANNEL_2_.my_release()

    except asyncio.CancelledError as e:

        print(f"\n! asyncio.CancelledError (UDP) : {e}")

    except BaseException as e:

        print(f"\n! BaseException (UDP) : {e}")

    finally:

        print("UDP ...[END]")

    return None

# === Main ===

async def main():

    results = None

    # --- Event ---

    def my_event_signal_terminate(signum, frame):

        global _SHUTDOWN_EVENT_

        print(f"Receive terminate signal ({signum}, {frame})")

        _SHUTDOWN_EVENT_ = True

    # --- Init ---

    print("Main program ...[START]")

    signal.signal(signal.SIGTERM, my_event_signal_terminate)
    signal.signal(signal.SIGINT, my_event_signal_terminate)

    results = asyncio.gather(my_task_mqtt(),my_task_udp())

    try:

        # Block
        await results

    except asyncio.CancelledError as e:

        print(f"\n! asyncio.CancelledError (Main) : {e}")

    except BaseException as e:

        print(f"\n! BaseException (Main) : {e}")

    print("Main program ...[END]")

if __name__ == "__main__" :

    if len(sys.argv) < 3 :

        print("\nUsage: python main.py <Robot ID> <MODE>\n")
        exit()

    ROBOT_ID = sys.argv[1]
    ROBOT_MODE = sys.argv[2]

    if(ROBOT_MODE != "avatar" and ROBOT_MODE != "control"):

        print(f"\nNot found {ROBOT_MODE} mode\n")
        exit()

    asyncio.run(main())
