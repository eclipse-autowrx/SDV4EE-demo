import json
import time
from kuksa_client.grpc import VSSClient, Datapoint
import paho.mqtt.client as mqtt

# MQTT broker configuration
MQTT_BROKER = "192.168.88.10"
MQTT_PORT = 1883
MQTT_USERNAME = "app1"
MQTT_PASSWORD = "app1password"

# Kuksa client configuration
KUKSA_HOST = '127.0.0.1'
KUKSA_PORT = 55555

# The specific MQTT topic to monitor
WHEEL_ANGLE_TOPIC = "Vehicle/ADAS/LaneAssist/TargetSteeringWheelAngle"

# The corresponding VSS path
WHEEL_ANGLE_VSS_PATH = "Vehicle.ADAS.LaneAssist.TargetSteeringWheelAngle"

def on_connect(client, userdata, flags, rc):
    """Callback when connected to MQTT broker"""
    print(f"Connected to MQTT broker with result code {rc}")
    # Subscribe to the specific wheel angle topic
    client.subscribe(WHEEL_ANGLE_TOPIC)
    print(f"Subscribed to {WHEEL_ANGLE_TOPIC}")

def on_message(client, userdata, msg):
    """Callback when message is received from MQTT broker"""
    if msg.topic != WHEEL_ANGLE_TOPIC:
        return  # Ignore other topics

    try:
        # Try to parse payload
        value = json.loads(msg.payload.decode())

        # Set the value in Kuksa VSS
        with VSSClient(KUKSA_HOST, KUKSA_PORT) as client:
            client.set_current_values({
                WHEEL_ANGLE_VSS_PATH: Datapoint(value),
            })
            print(f"Updated {WHEEL_ANGLE_VSS_PATH} to {value}")

    except Exception as e:
        print(f"Error processing wheel angle message: {e}")

def setup_mqtt():
    """Set up and connect the MQTT client"""
    client = mqtt.Client()

    # Set callbacks
    client.on_connect = on_connect
    client.on_message = on_message

    # Set credentials
    client.username_pw_set(MQTT_USERNAME, MQTT_PASSWORD)

    # Connect to broker
    client.connect(MQTT_BROKER, MQTT_PORT, 60)

    return client

def main():
    mqtt_client = None

    # Try to set up and connect the MQTT client, retrying on failure.
    while mqtt_client is None:
        try:
            mqtt_client = setup_mqtt()

            print("Connected to MQTT broker.")
        except OSError as e:
            print(f"Error connecting to MQTT broker: {e}. Retrying in 5 seconds...")
            time.sleep(10)

    try:
        mqtt_client.loop_forever()
    except KeyboardInterrupt:
        print("Interrupted by user, shutting down.")
    finally:
        mqtt_client.disconnect()

if __name__ == '__main__':
    main()
