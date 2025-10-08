# Docker on UNIX

## MacOS Installation Process
### Install Arduino IDE
```
https://www.arduino.cc/en/software/
```
- Choose the correct download file depending on your **CPU**
- You can check which type of CPU you have by going to the top left of your screen and clicking the '**about**' on your mac book

### Upload arduino code
```cpp
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
### Docker section
- Create a docker network
```bash
docker network create ros2-network
```
-  Run Docker Container
```bash
docker run -it --rm \
  --network ros2-net \
  --device /dev/ttyACM0:/dev/ttyACM0 \
  --group-add dialout \
  --shm-size=512m \
  -e DISPLAY=$DISPLAY \
  -e QT_X11_NO_MITSHM=1 \
  -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
  -v ~/ros2_ws:/root/ros2_ws \
  utsarobotics/ros2-humble:1.1.0 \
  bash
```

- Move into the ros-workspace directory
```bash
cd /root/ros2_ws/src
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
docker run -it --rm \
  --network ros2-net \
  --device /dev/ttyACM0:/dev/ttyACM0 \
  --group-add dialout \
  --shm-size=512m \
  -e DISPLAY=$DISPLAY \
  -e QT_X11_NO_MITSHM=1 \
  -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
  -v ~/ros2_ws:/root/ros2_ws \
  utsarobotics/ros2-humble:1.1.0 \
  bash
```

- Run the second node we made to manage the LED
```bash
source /root/ros2_ws/install/setup.bash
ros2 run led_controller led_serial_node
```
