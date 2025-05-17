import rclpy
import rclpy.clock
from rclpy.node import Node
from rclpy.time import Time

from std_msgs.msg import String, Float64, Empty
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Pose, Point, Quaternion
from rosgraph_msgs.msg import Clock

import numpy as np
import math
import time
from datetime import datetime
import threading
import os
os.environ["PYOPENGL_PLATFORM"] = "glx"

import genesis as gs


class GenesisSim(Node):

    def __init__(self):
        super().__init__('simple_genesis_sim')

        self.declare_parameter('model_path', '')
        self.model_path: str = self.get_parameter('model_path').get_parameter_value().string_value

        self.declare_parameter('jnt_names', [''])
        self.jnt_names: list[str] = self.get_parameter('jnt_names').get_parameter_value().string_array_value
        if self.jnt_names[0]=='':
            self.get_logger().warn("No joints are set to operate in 'parameter.yaml'.")

        self.declare_parameter('dofs_kp', [float('nan')]) 
        self.declare_parameter('dofs_kv', [float('nan')])
        self.declare_parameter('dofs_force_upper', [float('nan')]) 
        self.declare_parameter('dofs_force_lower', [float('nan')])
        self.dofs_kp: list[float] = self.get_parameter('dofs_kp').get_parameter_value().double_array_value
        self.dofs_kv: list[float]  = self.get_parameter('dofs_kv').get_parameter_value().double_array_value
        self.dofs_force_lower: list[float]  = self.get_parameter('dofs_force_lower').get_parameter_value().double_array_value
        self.dofs_force_upper: list[float]  = self.get_parameter('dofs_force_upper').get_parameter_value().double_array_value

        self.declare_parameter('put_objects', False)
        self.declare_parameter('env_objects_dir', '')


        self.clock_msg = Clock()
        self.publisher_clock = self.create_publisher(Clock, '/clock', 1)
        self.sim_time: float  = 0.0  # シミュレーション時刻の初期化
        self.sim_dt: float = 0.01
        
        self.target_joint_positions: list[float] = [0.0]*len(self.jnt_names)

        # 目標角subscribe用
        self.lock = threading.Lock()
        self.callback_group_1 = rclpy.callback_groups.MutuallyExclusiveCallbackGroup()
        self.target_pos_subscription = self.create_subscription(JointState, '/target_pos', self.joint_callback, 1, callback_group=self.callback_group_1)

        # ロボット情報publish用
        self.joint_state_msg = JointState()
        self.link_pose_msg = Pose()
        self.publisher_joint_state = self.create_publisher(JointState, '/joint_state', 1)
        self.publisher_link_pose   = self.create_publisher(Pose, '/link_pose', 1)  

        
        # cpu or gpuの使用
        self.declare_parameter('use_gpu', False)
        self.use_gpu: bool = self.get_parameter('use_gpu').get_parameter_value().bool_value
        self.get_logger().info(f"simulated on: {'GPU' if self.use_gpu else 'CPU'}")
        
        # 録画用
        self.declare_parameter('recording', False)
        self.recording: bool = self.get_parameter('recording').get_parameter_value().bool_value
        self.get_logger().info(f"recording: {self.recording}")
        if self.recording:
            self.video_name: str = 'video_' + datetime.now().strftime('%Y%m%d_%H%M%S') + '.mp4'
            self.callback_group_2 = rclpy.callback_groups.MutuallyExclusiveCallbackGroup()
            self.record_stop_subscription = self.create_subscription(Empty, '/stop_recording', self.record_stop_callback, 1, callback_group=self.callback_group_2)

        
        self.genesis_build()  

        self.running: bool = True

        self.pub_clock(self.sim_time)
        if self.recording:
            self.cam.start_recording()
        self.callback_group_3 = rclpy.callback_groups.MutuallyExclusiveCallbackGroup()
        self.step_timer = self.create_timer(0.0001, self.sim_step, clock=rclpy.clock.Clock(clock_type=rclpy.clock.ClockType.SYSTEM_TIME), callback_group=self.callback_group_3)

    

    def record_stop_callback(self, msg):
        self.get_logger().info("STOP Simulation")
        self.stop()
        self.get_logger().info("Exit by 'ctrl+c'")


    def genesis_build(self):
        if self.use_gpu:
            gs.init(backend=gs.gpu)
        else:
            gs.init(backend=gs.cpu)


        self.scene = gs.Scene(
            viewer_options=gs.options.ViewerOptions(
                camera_pos=(0, -3.5, 2.5),
                camera_lookat=(0.0, 0.0, 0.5),
                camera_fov=30,
                max_FPS=60,
            ),
            sim_options=gs.options.SimOptions(
                dt=self.sim_dt,
            ),
            show_viewer=True,
            show_FPS=True,
        )

        ########################## entities ##########################
        self.plane = self.scene.add_entity(
            gs.morphs.Plane(),
        )

        if self.model_path.endswith(".xml"):
            self.robot = self.scene.add_entity(
                #gs.morphs.MJCF(file="xml/robot_emika_panda/panda.xml"),
                gs.morphs.MJCF(file=self.model_path),
            )
        elif self.model_path.endswith(".urdf"):
            self.robot = self.scene.add_entity(
                gs.morphs.URDF(file=self.model_path,
                               fixed=False,
                               ),
            )

        self.cam = self.scene.add_camera(
            res=(640, 480),
            pos=(0.0, -10.0, 2.5),
            lookat=(0, 0, 0.5),
            fov=30,
            GUI=False,
        )

        #### environment objects ####
        if self.get_parameter('put_objects').get_parameter_value().bool_value:
            self.env_objects_dir: str = self.get_parameter('env_objects_dir').get_parameter_value().string_value

            self.cylinder = self.scene.add_entity(
                gs.morphs.URDF(
                     file=self.env_objects_dir+'/urdf/cylinder.urdf',
                     pos=(1.0, -1.0, 0.0),
                     fixed=True,
                ),
            )

        ########################## build ##########################
        self.scene.build()

        #self.jnt_names = [
        #    "joint1",
        #    "joint2",
        #    "joint3",
        #    "joint4",
        #    "joint5",
        #    "joint6",
        #    "joint7",
        #    "finger_joint1",
        #    "finger_joint2",
        #]
        self.dofs_idx = [self.robot.get_joint(name).dof_idx_local for name in self.jnt_names]

        #self.link_names = [
        #    "link0",
        #    "link1",
        #    "link2",
        #    "link3",
        #    "link4",
        #    "link5",
        #    "link6",
        #    "link7",
        #    "hand",
        #    "left_finger",
        #    "right_finger",
        #]
        #self.links_idx = [self.robot.get_link(name).idx_local for name in self.link_names]

        ############ Optional: set control gains ############
        # set positional gains
        if  not math.isnan(self.dofs_kp[0]) and len(self.jnt_names)==len(self.dofs_kp):
            self.robot.set_dofs_kp(
                kp=np.array(self.dofs_kp),
                dofs_idx_local=self.dofs_idx,
            )
        else:
            self.get_logger().info("'dofs_kp' are unchanged")

        # set velocity gains
        if  not math.isnan(self.dofs_kv[0]) and len(self.jnt_names)==len(self.dofs_kv):
            self.robot.set_dofs_kv(
                kv=np.array(self.dofs_kv),
                dofs_idx_local=self.dofs_idx,
            )
        else:
            self.get_logger().info("'dofs_kv' are unchanged")

        # set force range
        if  not math.isnan(self.dofs_force_lower[0]) and not math.isnan(self.dofs_force_upper[0]) and len(self.jnt_names)==len(self.dofs_force_lower)==len(self.dofs_force_upper):
            self.robot.set_dofs_force_range(
                lower=np.array(self.dofs_force_lower),
                upper=np.array(self.dofs_force_upper),
                dofs_idx_local=self.dofs_idx,
            )
        else:
            self.get_logger().info("force ranges are unchanged")
        



    def sim_step(self):

        with self.lock:
            self.robot.control_dofs_position(
                np.array(self.target_joint_positions),
                self.dofs_idx,
            )


        self.scene.step()
        if self.recording:
            self.cam.render()

        # シミュレーション時間publish
        self.sim_time = self.scene.cur_t
        self.pub_clock(self.sim_time)

        # ロボット情報publish
        pos = self.robot.get_dofs_position(dofs_idx_local=self.dofs_idx)
        vel = self.robot.get_dofs_velocity(dofs_idx_local=self.dofs_idx)
        eff = self.robot.get_dofs_force(dofs_idx_local=self.dofs_idx)
        self.pub_joint_state(self.jnt_names, pos.tolist(), vel.tolist(), eff.tolist())

        #base_pos = self.robot.get_pos()             # base_linkの位置
        #base_quat = self.robot.get_quat()           # base_linkの姿勢
        links_pos = self.robot.get_links_pos()      # 各linkの位置
        links_quat = self.robot.get_links_quat()    # 各linkの姿勢
        #links_vel = self.robot.get_links_vel()      # 各linkの速度
        #links_ang_vel = self.robot.get_links_ang()  # 各linkの角速度
        self.pub_link_pose(links_pos.tolist()[0], links_quat.tolist()[0])

        
        if not self.running:
            self.step_timer.cancel()
            if self.recording:
                self.cam.stop_recording(save_to_filename=f"{self.video_name}", fps=int(1/self.scene.dt))

            
    def pub_clock(self, sim_time: float):
        if rclpy.ok():
            # Clockメッセージを作成
            self.clock_msg.clock = Time(seconds=int(sim_time), nanoseconds=int((sim_time % 1) * 1e9)).to_msg()
            # /clockに送信
            self.publisher_clock.publish(self.clock_msg)
            #self.get_logger().info(f'Publishing simulation time: {int(sim_time)}.{int((sim_time % 1) * 1e9)}')


    def pub_joint_state(self, names: list[String], positions: list[float], velocities: list[float], efforts: list[float]):
        if rclpy.ok():
            self.joint_state_msg.header.stamp = self.get_clock().now().to_msg()
            self.joint_state_msg.name     = names
            self.joint_state_msg.position = positions
            self.joint_state_msg.velocity = velocities
            self.joint_state_msg.effort   = efforts

            self.publisher_joint_state.publish(self.joint_state_msg)


    def pub_link_pose(self, position: list[float], quaternion: list[float]):
        if rclpy.ok():
            self.link_pose_msg.position = Point(x=position[0], y=position[1], z=position[2])
            self.link_pose_msg.orientation = Quaternion(x=quaternion[0], y=quaternion[1], z=quaternion[2], w=quaternion[3])

            self.publisher_link_pose.publish(self.link_pose_msg)
    

    def joint_callback(self, msg):
        with self.lock:
            self.target_joint_positions = msg.position

    def stop(self):
        self.running = False


def main(args=None):
    rclpy.init(args=args)

    node = GenesisSim()
    executor = rclpy.executors.MultiThreadedExecutor()

    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard interrupt, shutting down.\n')
    finally:
        node.stop()
        if rclpy.ok():  # 既に shutdown されていないか確認
            node.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    main()
