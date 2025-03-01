import paho.mqtt.client as mqtt
import time
import ssl

class MySecondaryDataChannel(mqtt.Client):

    def __init__(self):

        pass

    # === Init ===

    def my_init(self, id, callback_on_connect, callback_on_message) -> None:

        super().__init__(callback_api_version=mqtt.CallbackAPIVersion.VERSION2, client_id=f"{id}-control-{time.time()}", clean_session=True, transport="websockets")
        self.ws_set_options(path="/mqtt")
        self.tls_set(ca_certs=None, cert_reqs=ssl.CERT_REQUIRED)

        self.on_connect = callback_on_connect
        self.on_message = callback_on_message

    def my_connect(self, host:str, port:int) -> None:

        self.connect_async(host=host, port=port, keepalive=5)
        self.loop_start()

    def my_release(self) -> None:

        self.loop_stop()
