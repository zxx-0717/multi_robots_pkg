import rclpy
from rclpy.node import Node
import os
import re
import sys

from capella_ros_msg.msg import RobotInfo,RobotPoseWithNamespace,PlanWithNamespace,MultiRobotsInfo
from geometry_msgs.msg import PoseStamped


class Multi_Info_Sub(Node):
    def __init__(self, ):
        super().__init__('multi_robot_info_sub_node')

        # 获取环境变量
        self.robot_namespase = os.environ.get('CAPELLA_ROS_NAMESPACE')
        if self.robot_namespase is not None:
                self.robot_namespase = str(self.robot_namespase)
                self.get_logger().info(f'当前机器人 CAPELLA_ROS_NAMESPACE ：{self.robot_namespase}')
        else:
                self.get_logger().info(r'没有获取到当前机器人的 CAPELLA_ROS_NAMESPACE 环境变量，请配置后再启动程序，退出程序！')
                self.robot_namespase = str('mk5')
                # rclpy.shutdown()
                # sys.exit()

        # 订阅消息节点
        self.multi_robots_info_sub  = self.create_subscription(MultiRobotsInfo,r'/multi_robots_info_rx',self.multi_robots_info_sub_callback,10)

        # 消息发布节点
        self.robot_info_pub = self.create_publisher(RobotInfo,r'/robot_info',10)
        self.robot_pose_with_namespace_pub = self.create_publisher(RobotPoseWithNamespace,r'/robot_pose',10)
        self.plan_with_namespace_pub = self.create_publisher(PlanWithNamespace,r'/plan_stamped',10)

        self.get_logger().info(r'节点 multi_robot_info_sub_node 初始化完成！')


    def multi_robots_info_sub_callback(self,msg):
        decode_data = ''.join([x.decode(encoding='utf-8') for x in msg.data])
        if msg.type == 0:
            self.get_logger().info(r'receive robot_info！',throttle_duration_sec=5)
            # robot_info
            name_space = re.findall(r"namespace_name='(.*?)',",decode_data)[0]
            if name_space != self.robot_namespase:
                send_msg = RobotInfo()
                send_msg.namespace_name = name_space
                send_msg.priority = int(re.findall(r"priority=(.*?)\)",decode_data)[0])
                self.robot_info_pub.publish(send_msg)
        elif msg.type == 1:
            self.get_logger().info(r'receive robot_pose_with_namespace',throttle_duration_sec=5)
            # robot_pose_with_namespace
            name_space = re.findall(r"namespace_name='(.*?)',",decode_data)[0]
            if name_space != self.robot_namespase:
                send_msg = RobotPoseWithNamespace()
                send_msg.namespace_name = name_space
                stamps = re.findall(r"header=std_msgs.msg.Header\(stamp=builtin_interfaces.msg.Time\(sec=(.*?), nanosec=(.*?)\)",decode_data)
                send_msg.pose.header.stamp.sec = int(stamps[0][0])
                send_msg.pose.header.stamp.nanosec = int(stamps[0][1])
                send_msg.pose.header.frame_id = re.findall(r"frame_id='(.*?)'",decode_data)[0]
                position_xyz = re.findall(r"position=geometry_msgs.msg.Point\(x=(.*?), y=(.*?), z=(.*?)\)",decode_data)[0]
                send_msg.pose.pose.position.x = float(position_xyz[0])
                send_msg.pose.pose.position.y = float(position_xyz[1])
                send_msg.pose.pose.position.z = float(position_xyz[2])
                orientation_xyzw = re.findall(r"orientation=geometry_msgs.msg.Quaternion\(x=(.*?), y=(.*?), z=(.*?), w=(.*?)\)",decode_data)[0]
                send_msg.pose.pose.orientation.x = float(orientation_xyzw[0])
                send_msg.pose.pose.orientation.y = float(orientation_xyzw[1])
                send_msg.pose.pose.orientation.z = float(orientation_xyzw[2])
                send_msg.pose.pose.orientation.w = float(orientation_xyzw[3])
                self.robot_pose_with_namespace_pub.publish(send_msg)
        elif msg.type == 2:
            self.get_logger().info(r'receive plan_with_namespace',throttle_duration_sec=5)
            # plan_with_namespace
            name_space = re.findall(r"namespace_name='(.*?)',",decode_data)[0]
            if name_space != self.robot_namespase:
                send_msg = PlanWithNamespace()
                send_msg.namespace_name = name_space
                stamps = re.findall(r"header=std_msgs.msg.Header\(stamp=builtin_interfaces.msg.Time\(sec=(.*?), nanosec=(.*?)\)",decode_data)
                send_msg.path.header.stamp.sec = int(stamps[0][0])
                send_msg.path.header.stamp.nanosec = int(stamps[0][1])
                frame_ids = re.findall(r"frame_id='(.*?)'",decode_data)
                send_msg.path.header.frame_id = frame_ids[0]
                position_xyz_list = re.findall(r"position=geometry_msgs.msg.Point\(x=(.*?), y=(.*?), z=(.*?)\)",decode_data)
                orientation_xyzw_list = re.findall(r"orientation=geometry_msgs.msg.Quaternion\(x=(.*?), y=(.*?), z=(.*?), w=(.*?)\)",decode_data)
                for i in range(len(position_xyz_list)):
                     posestamp_msg = PoseStamped()
                     posestamp_msg.header.stamp.sec = int(stamps[1:][i][0])
                     posestamp_msg.header.stamp.nanosec = int(stamps[1:][i][1])
                     posestamp_msg.header.frame_id = frame_ids[1:][i]
                     posestamp_msg.pose.position.x = float(position_xyz_list[i][0])
                     posestamp_msg.pose.position.y = float(position_xyz_list[i][1])
                     posestamp_msg.pose.position.z = float(position_xyz_list[i][2])
                     posestamp_msg.pose.orientation.x = float(orientation_xyzw_list[i][0])
                     posestamp_msg.pose.orientation.y = float(orientation_xyzw_list[i][1])
                     posestamp_msg.pose.orientation.z = float(orientation_xyzw_list[i][2])
                     posestamp_msg.pose.orientation.w = float(orientation_xyzw_list[i][3])

                     send_msg.path.poses.append(posestamp_msg)
                self.plan_with_namespace_pub.publish(send_msg)

def main(args=None):
        rclpy.init(args=args)

        minimal_subscriber = Multi_Info_Sub()

        rclpy.spin(minimal_subscriber)

        # Destroy the node explicitly
        # (optional - otherwise it will be done automatically
        # when the garbage collector destroys the node object)
        minimal_subscriber.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
        main()
