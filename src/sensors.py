import paho.mqtt.client as mqtt
import threading
import json

class Sensor():
    def __init__(self, name="Sibiu", addr='127.0.0.1', port=1883, timeout=60, topics2suscribe="sensor"):
        self.client = mqtt.Client(name)
        self.topics2suscribe=topics2suscribe
        self._mqtt_thread = threading.Thread(target=self.mqtt_thread, args=(addr, port, timeout,))
        self._mqtt_thread.start()
    
    def mqtt_thread(self, addr='127.0.0.1', port=1883, timeout=60):
        self.client.on_connect = self.on_connect
        self.client.on_message = self.onmessage
        self.client.on_disconnect = self.on_disconnect
        self.client.connect(addr, port, timeout)
        self.client.loop_forever()

    def on_connect(self, _client, _, __, rc):
        print("Connected to MQTT server")
        for topic in self.topics2suscribe:
            self.client.subscribe(topic)

    def on_disconnect(self, client, userdata,_, rc):
        print("client disconnected ok")

    def on_message(self, _client, user_data, msg):
        message = json.loads(msg.payload.decode('utf-8'))  # Decode the msg into UTF-8
        if "ph" in message:
            print(f"ph is {message['ph']}")
        else:
            print(f"unknown message {message}")

    def close(self):
        self.client.loop_stop()
        self.client.disconnect()

    
    def take_sample(self):
        time.sleep(1)

if __name__ == '__main__':
    pass