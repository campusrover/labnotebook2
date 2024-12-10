# Controlling an External Actuator with a ROS Publisher

## Requirements
- Arduino (optional)
- Raspberry Pi
- Relay
- External actuator (such as a battery-powered sprayer)

## How It Works
To control an actuator, a relay is used to handle the switching of the circuit loop, enabling it to turn on and off.  
A logic board, such as a microcontroller, is required to control the relay, providing the necessary signals to power the actuator on and off.

There are two ways to control the relay with the logic board:  

### 1. Controlling the Relay Directly with a Raspberry Pi
In this approach, the relay is connected directly to the GPIO pins of the Raspberry Pi, which controls the relay based on ROS messages received.

#### Pros:
- **Simplicity**: Fewer components are needed since the relay is connected directly to the Raspberry Pi.
- **Reduced Latency**: Direct control minimizes delay between receiving a message and activating the actuator.
- **Compact Design**: Eliminates the need for additional hardware like an Arduino.

#### Cons:
- **Limited GPIO Pins**: The Raspberry Pi has a limited number of GPIO pins, which may restrict additional functionality.
- **Current and Voltage Handling**: GPIO pins on the Raspberry Pi may not handle higher current/voltage requirements for some relays, necessitating the use of relay modules with optoisolation.

### 2. Controlling the Relay with an Arduino
In this approach, the relay is connected to an Arduino, which receives commands from the Raspberry Pi via serial communication. The Arduino processes the commands and controls the relay accordingly.

#### Pros:
- **Increased Flexibility**: The Arduino can offload tasks from the Raspberry Pi, allowing it to handle other processes efficiently.
- **Higher Voltage/Current Capability**: The Arduino can interface more easily with relays that require higher voltage or current than the Raspberry Pi GPIO pins can provide.
- **Modularity**: Easier to troubleshoot or replace components since the Raspberry Pi and Arduino work independently.

#### Cons:
- **Increased Complexity**: Adds another component to the system, requiring more programming and wiring.
- **Latency**: Introduces a small delay due to the additional communication step between the Raspberry Pi and Arduino.
- **Cost and Size**: Slightly increases the overall cost and size of the setup.

---

## Which Option to Choose?

- Use **Option 1 (Direct Control with Raspberry Pi)** if simplicity, compactness, and minimal cost are priorities, and the relay can operate within the Raspberry Pi's GPIO pin specifications.  
- Use **Option 2 (Control with Arduino)** if you require additional flexibility, need to handle higher power relays, or want to distribute the control logic between the Raspberry Pi and Arduino.

---

## Implementation Examples

Control with Arduino
1. Connect the relay module to the Arduino digital pin (e.g., pin 7).
2. Program the Arduino to receive serial commands (`ACTUATOR_ON`/`ACTUATOR_OFF`) from the Raspberry Pi and toggle the relay.
```
const int relayPin = 7;  // Pin connected to the relay
bool isRelayOn = false; 

void setup() {
  pinMode(relayPin, OUTPUT);       // Set relay pin as output
  digitalWrite(relayPin, HIGH);     // Ensure relay starts OFF
  Serial.begin(9600);             // Start serial communication
}

void loop() {
  if (Serial.available() > 0) {  // Check if data is available to read
    String command = Serial.readStringUntil('\n');  // Read the incoming message
    command.trim();  // Remove any extra whitespace or newline characters

    if (command == "RELAY_ON" && !isRelayOn) {
      digitalWrite(relayPin, LOW);  // Turn the relay ON
      isRelayOn = true;
      Serial.println("Relay is ON");
    } else if (command == "RELAY_OFF" && isRelayOn) {
      digitalWrite(relayPin, HIGH);   // Turn the relay OFF
      isRelayOn = false;
      Serial.println("Relay is OFF");
    } else {
      Serial.println("Unknown command received");
    }
  }
}

```
4. Write a ROS subscriber node on the Raspberry Pi that sends these serial commands to the Arduino.
```
#!/usr/bin/env python3
import rospy
from std_msgs.msg import String
import serial
import time

class ArduinoController:
    def __init__(self):
        # Set up the serial connection to the Arduino
        self.arduino = serial.Serial('/dev/arduino-uno', 9600, timeout=1)  # Replace with your port
        time.sleep(2)  # Allow time for the connection to initialize

        # Initialize the ROS subscriber
        rospy.init_node('relay_control_subscriber', anonymous=True)
        rospy.Subscriber('/relay_control', String, self.callback)
        rospy.loginfo("Subscriber initialized and listening to /relay_control")

    def callback(self, msg):
        # Send the received command to the Arduino
        command = msg.data.strip()
        rospy.loginfo(f"Received: {command}")
        if command in ["RELAY_ON", "RELAY_OFF"]:
            self.arduino.write(f"{command}\n".encode())
            rospy.loginfo(f"Sent to Arduino: {command}")
        else:
            rospy.logwarn(f"Unknown command received: {command}")

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        controller = ArduinoController()
        controller.run()
    except rospy.ROSInterruptException:
        pass
```
---

