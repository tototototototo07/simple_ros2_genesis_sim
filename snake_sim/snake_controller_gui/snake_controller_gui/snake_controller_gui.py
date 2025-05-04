import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from std_msgs.msg import String
from std_msgs.msg import Int8MultiArray
import tkinter as tk
import threading

class SnakeControllerGUI(Node):
    def __init__(self):
        super().__init__('snake_controller_gui')
        self.gait_pub = self.create_publisher(String, 'gait_cmd', 10)
        self.motion_pub = self.create_publisher(Int8MultiArray, 'motion_cmd', 10)
        self.current_motion = [0, 0, 0]  # [forward/back, roll, radius]

    def publish_cmd(self, category, msg_str):
        if category == 'gait':
            # Before switching gait, reset motion to [0,0,0]
            reset_msg = Int8MultiArray()
            reset_msg.data = [0, 0, 0]
            self.motion_pub.publish(reset_msg)
            self.current_motion = [0, 0, 0]

            msg = String()
            msg.data = msg_str
            self.get_logger().info(f'Gait: "{msg_str}"')
            self.gait_pub.publish(msg)
        elif category == 'motion':
            self.get_logger().info(f'Motion: "{msg_str}"')
            if msg_str == "Forward":
                self.current_motion[0] = 1
            elif msg_str == "Back":
                self.current_motion[0] = -1
            elif msg_str == "Roll L":
                self.current_motion[1] = 1
            elif msg_str == "Roll R":
                self.current_motion[1] = -1
            elif msg_str == "STOP":
                self.current_motion[0] = 0
                self.current_motion[1] = 0
            msg = Int8MultiArray()
            msg.data = self.current_motion
            self.motion_pub.publish(msg)
        elif category == 'radius':
            self.get_logger().info(f'Radius: "{msg_str}"')
            if msg_str == "Increase":
                self.current_motion[2] = 1
            elif msg_str == "Decrease":
                self.current_motion[2] = -1
            elif msg_str == "STOP":
                self.current_motion[2] = 0
            msg = Int8MultiArray()
            msg.data = self.current_motion
            self.motion_pub.publish(msg)

def create_button(master, text, command, width=10, height=2):
    return tk.Button(master, text=text, width=width, height=height, command=command)

def start_gui(node):
    root = tk.Tk()
    root.title("ROS2 GUI Controller")

    def make_command(category, cmd):
        def command():
            node.publish_cmd(category, cmd)
        return command

    # Left side - Gaits
    left_frame = tk.Frame(root)
    left_frame.grid(row=0, column=0, padx=10)
    tk.Label(left_frame, text="Gait Mode").pack()
    for mode in ["Pedal", "Lateral", "Helical"]:
        def make_gait_command(m=mode):
            def cmd():
                node.publish_cmd('gait', m)
            return cmd
        btn = create_button(left_frame, text=mode, command=make_gait_command())
        btn.pack(pady=2)

    # Center - Movement control
    center_frame = tk.Frame(root)
    center_frame.grid(row=0, column=1, padx=10)
    motions = [("Forward", 0, 1), ("Roll L", 1, 0), ("STOP", 1, 1), ("Roll R", 1, 2), ("Back", 2, 1)]
    for label, r, c in motions:
        btn = create_button(center_frame, label, command=make_command('motion', label))
        btn.grid(row=r, column=c)

    # Right - Adjust radius
    right_frame = tk.Frame(root)
    right_frame.grid(row=0, column=2, padx=10)
    tk.Label(right_frame, text="Change Radius").pack()
    for label in ["Increase", "STOP Radius", "Decrease"]:
        btn = create_button(right_frame, label, command=make_command('radius', label.replace('STOP Radius', 'STOP')))
        btn.pack(pady=2)

    try:
        root.mainloop()
    except KeyboardInterrupt:
        pass

def spin_node(node):
    try:
        rclpy.spin(node)
    except ExternalShutdownException:
        pass
    except Exception as e:
        node.get_logger().error(f"spin: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = SnakeControllerGUI()

    spin_thread = threading.Thread(target=spin_node, args=(node,), daemon=True)
    spin_thread.start()

    start_gui(node)

    if rclpy.ok():
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
