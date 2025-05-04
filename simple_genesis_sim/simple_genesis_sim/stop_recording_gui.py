import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from std_msgs.msg import Empty
import tkinter as tk
import threading

class ShutdownPublisher(Node):
    def __init__(self):
        super().__init__('gui_test')
        self.publisher_ = self.create_publisher(Empty, '/stop_recording', 10)

    def publish_stop_recording(self):
        msg = Empty()
        self.publisher_.publish(msg)
        self.get_logger().info('Publish /stop_recording')

def start_gui(node):
    def on_click():
        node.publish_stop_recording()
        status_label.config(text="After the video is saved,\nExit with 'ctrl+c'")

    window = tk.Tk()
    window.title("Stop Recording Button")

    stop_button = tk.Button(window, text="STOP REC", font=("Arial", 24), command=on_click, bg="red", fg="white")
    stop_button.pack(padx=20, pady=20)

    global status_label
    status_label = tk.Label(window, text="", font=("Arial", 14))
    status_label.pack(pady=(0, 20))

    try:
        window.mainloop()
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
    node = ShutdownPublisher()

    # rclpyスピンは別スレッドで実行
    spin_thread = threading.Thread(target=spin_node, args=(node,), daemon=True)
    spin_thread.start()

    start_gui(node)

    if rclpy.ok():
        node.destroy_node()
        rclpy.shutdown()
