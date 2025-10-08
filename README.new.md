# Docker on Windows 

### Install Arduino IDE
```web
https://www.arduino.cc/en/software/
```
- Choose the correct download file depending on your **CPU**
- You can check which type of CPU you have by going to the top left of your screen and clicking the '**about**' on your mac book

### Upload arduino code
```c
int ledPin = 13; // or whatever pin you’re using
String cmd = "";

void setup() {
  pinMode(ledPin, OUTPUT);
  Serial.begin(9600);
}

void loop() {
  if (Serial.available()) {
    cmd = Serial.readStringUntil('\n');
    cmd.trim();
    if (cmd == "on") {
      digitalWrite(ledPin, HIGH);
      Serial.println("LED ON");
    } else if (cmd == "off") {
      digitalWrite(ledPin, LOW);
      Serial.println("LED OFF");
    }
  }
}
```

- Make a directory for ROS2
```bash
mkdir C:\ros2_led_project
cd C:\ros2_led_project
```

- check what port your arduino is located
  - Open "Device Manager" and look for what "COM" port the arduino is connected to

- Check if we can see the arduino port (connect the arduino to your computer[might have to run next command in wsl])
```bash
wsl ls /dev/tty*
```
---
### Docker section
- Create a docker network
```bash
docker network create ros2-network
```
-  Run Docker Container
```bash
docker run -it --rm ^
  --network ros2-net ^
  --device /dev/ttyS4:/dev/ttyACM0 ^
  --shm-size=512m ^
  -v /mnt/c/Users/<YourName>/ros2_ws:/root/ros2_ws ^
  utsarobotics/ros2-humble:1.1.0 ^
  bash
```




- Move into the ros-workspace directory
```bash
cd /root/ros2_ws
mkdir -p src
cd src

```

- Create a new ROS2 node package
```bash
ros2 pkg create --build-type ament_python led_controller
```

- Create a python node to read from the serial-port
```bash
nano /root/ros2_ws/src/led_controller/led_controller/led_publisher.py
```

- Paste this python code into the file
```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class LEDPublisher(Node):
    def __init__(self):
        super().__init__('led_publisher')
        self.publisher_ = self.create_publisher(String, 'led_command', 10)
        self.state = False
        self.timer = self.create_timer(2.0, self.timer_callback)
        self.get_logger().info("LED Publisher started — toggling every 2 seconds")

    def timer_callback(self):
        msg = String()
        msg.data = "on" if self.state else "off"
        self.publisher_.publish(msg)
        self.get_logger().info(f"Publishing: {msg.data}")
        self.state = not self.state

def main(args=None):
    rclpy.init(args=args)
    node = LEDPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
```

- Create a different node for controlling the LED signal
```bash
nano /root/ros2_ws/src/led_controller/led_controller/led_keyboard_publisher.py
```

- Paste the second python code into the node just created
```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import sys
import termios
import tty
import select

class LEDKeyboardPublisher(Node):
    def __init__(self):
        super().__init__('led_keyboard_publisher')
        self.publisher = self.create_publisher(String, 'led_command', 10)
        self.get_logger().info("Press '1' to turn LED ON, '2' to turn LED OFF, 'q' to quit.")

    def get_key(self):
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setraw(fd)
            rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
            if rlist:
                key = sys.stdin.read(1)
            else:
                key = None
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return key

    def run(self):
        while rclpy.ok():
            key = self.get_key()
            if key == '1':
                msg = String()
                msg.data = 'on'
                self.publisher.publish(msg)
                self.get_logger().info("🔆 LED ON command sent")
            elif key == '2':
                msg = String()
                msg.data = 'off'
                self.publisher.publish(msg)
                self.get_logger().info("🌑 LED OFF command sent")
            elif key == 'q':
                self.get_logger().info("Exiting keyboard control...")
                break

def main(args=None):
    rclpy.init(args=args)
    node = LEDKeyboardPublisher()
    node.run()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

- Edit the python setup.py code to include the 2 new nodes
```bash
nano /root/ros2_ws/srcled_controller/setup.py
```

- Replace the Setup code with this code
```python
from setuptools import setup

package_name = 'led_controller'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='you@example.com',
    description='LED control using ROS2 and Arduino',
    license='MIT',
    entry_points={
        'console_scripts': [
            'led_publisher = led_controller.led_publisher:main',
            'led_keyboard_publisher = led_controller.led_keyboard_publisher:main',
        ],
    },
)
```

- Build the new nodes we created (do this everytime you change the code for the package)
```bash
cd /root/ros2_ws
source /opt/ros/humble/setup.bash
colcon build
source install/setup.bash
```

- Then run the first lisening node on the current terminal
```bash
source /root/ros2_ws/install/setup.bash
ros2 run led_controller led_serial_node
```

### Second terminal

- Open another terminal and run the docker container again
```bash

docker run -it --rm ^
  --network ros2-net ^
  --device /dev/ttyS4:/dev/ttyACM0 ^
  --shm-size=512m ^
  -v /mnt/c/Users/<YourName>/ros2_ws:/root/ros2_ws ^
  utsarobotics/ros2-humble:1.1.0 ^
  bash

```

- Run the second node we made to manage the LED
```bash
source /root/ros2_ws/install/setup.bash
ros2 run led_controller led_keyboard_publisher
```

---

##  Servo Control

- upload this servo arduino code into your arduino
```c
#include <Servo.h>

Servo servo;
int servoPin = 9;  // Change as needed
int pos = 0;

void setup() {
  Serial.begin(9600);
  servo.attach(servoPin);
  servo.write(90);
}

void loop() {
  if (Serial.available()) {
    int angle = Serial.parseInt();
    if (angle >= 0 && angle <= 180) {
      servo.write(angle);
      Serial.print("Moved to: ");
      Serial.println(angle);
    }
  }
}
```

- Inside the Docker Container go into this directory
```bash
source /opt/ros/humble/setup.bash
cd /root/ros2_ws
mkdir -p src
cd src
```
- Create a new node package for ROS2
```bash
ros2 pkg create --build-type ament_python servo_controller
```

- Create a servo_publisher.py node
```bash
nano /root/ros2_ws/src/servo_controller/servo_controller/servo_publisher.py
```
- Paste this python code into the servo_publisher.py file
```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64

class ServoPublisher(Node):
    def __init__(self):
        super().__init__('servo_publisher')
        self.publisher = self.create_publisher(Float64, 'servo_angle', 10)
        self.angle = 0
        self.create_timer(1.0, self.timer_callback)

    def timer_callback(self):
        msg = Float64()
        msg.data = float(self.angle)
        self.publisher.publish(msg)
        self.get_logger().info(f'Publishing: {self.angle}')
        self.angle = (self.angle + 10) % 180

def main(args=None):
    rclpy.init(args=args)
    node = ServoPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
```

- Create a servo_node.py node
```bash
nano /root/ros2_ws/src/servo_controller/servo_controller/servo_node.py
```

- Paste this python code into the servo_node.py file
```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
import serial

class ServoNode(Node):
    def __init__(self):
        super().__init__('servo_node')
        self.subscription = self.create_subscription(
            Float64,
            'servo_angle',
            self.listener_callback,
            10)
        self.serial = serial.Serial('/dev/ttyACM0', 9600, timeout=1)
        self.get_logger().info("Connected to Arduino on /dev/ttyACM0")

    def listener_callback(self, msg):
        angle = int(msg.data)
        self.serial.write(f"{angle}\n".encode())
        self.get_logger().info(f"Sent angle: {angle}")

def main(args=None):
    rclpy.init(args=args)
    node = ServoNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
```

- Edit the setup.py file to include the 2 new nodes
```bash
nano /root/ros2_ws/src/servo_controller/setup.py
```

- Replace the setup.py code with this None
```python
from setuptools import setup

package_name = 'servo_controller'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='your_name',
    maintainer_email='your_email@example.com',
    description='Servo control via Arduino over ROS 2',
    license='MIT',
    entry_points={
        'console_scripts': [
            'servo_publisher = servo_controller.servo_publisher:main',
            'servo_node = servo_controller.servo_node:main',
        ],
    },
)
```

- Build the ros-workspace
```bash
cd /root/ros2_ws
source /opt/ros/humble/setup.bash
colcon build
source install/setup.bash
```

- Run the first listening node
```bash
ros2 run servo_controller servo_publisher
```

- Open a new Docker container terminal
```bash
docker run -it --rm ^
  --network ros2-net ^
  --device /dev/ttyS4:/dev/ttyACM0 ^
  --shm-size=512m ^
  -v /mnt/c/Users/<YourName>/ros2_ws:/root/ros2_ws ^
  utsarobotics/ros2-humble:1.1.0 ^
  bash
```

-- Run the Second Servo angle node
```bash
ros2 run servo_controller servo_node
```
