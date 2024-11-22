import rclpy
from rclpy.node import Node
import os
import sys

from capella_ros_msg.msg import RobotInfo,RobotPoseWithNamespace,PlanWithNamespace,MultiRobotsInfo


class Multi_Info_Pub(Node):
    def __init__(self,):
        super().__init__('multi_robot_info_pub_node')

        # 获取环境变量
        self.robot_namespase = os.environ.get('CAPELLA_ROS_NAMESPACE')
        if self.robot_namespase is not None:
                self.robot_namespase = str(self.robot_namespase)
        else:
                self.get_logger().info(r'没有获取到当前机器人的 CAPELLA_ROS_NAMESPACE 环境变量，请配置后再启动程序，退出程序！')
                self.robot_namespase = str('mk5')
                # rclpy.shutdown()
            #     sys.exit()

        # 订阅当前机器人的话题
        self.robot_info_sub = self.create_subscription(RobotInfo,r'/robot_info',self.robot_info_sub_callback,10)
        self.robot_pose_with_namespace_sub = self.create_subscription(RobotPoseWithNamespace,r'/robot_pose',self.robot_pose_with_namespace_sub_callback,10)
        self.plan_with_namespace_sub = self.create_subscription(PlanWithNamespace,r'/plan_stamped',self.plan_with_namespace_sub_callback,10)

        # 发布
        self.multi_robots_info_pub = self.create_publisher(MultiRobotsInfo,r'/multi_robots_info_tx',10)

        self.get_logger().info(r'节点 multi_robot_info_pub_node 初始化完成！')


    def robot_info_sub_callback(self,msg):
        # print(msg.__str__())
        if msg.namespace_name == self.robot_namespase:
                multi_robots_info_pub_msg = MultiRobotsInfo()
                multi_robots_info_pub_msg.type = 0
                multi_robots_info_pub_msg.data = [x.encode(encoding='utf-8') for x in msg.__str__()]
                self.multi_robots_info_pub.publish(multi_robots_info_pub_msg)

    
    def robot_pose_with_namespace_sub_callback(self,msg):
        # print(msg.__str__())
        if msg.namespace_name == self.robot_namespase:
                multi_robots_info_pub_msg = MultiRobotsInfo()
                multi_robots_info_pub_msg.type = 1
                multi_robots_info_pub_msg.data = [x.encode(encoding='utf-8') for x in msg.__str__()]
                self.multi_robots_info_pub.publish(multi_robots_info_pub_msg)
    
    def plan_with_namespace_sub_callback(self,msg):
        # print(msg.__str__())
        if msg.namespace_name == self.robot_namespase:
                multi_robots_info_pub_msg = MultiRobotsInfo()
                multi_robots_info_pub_msg.type = 2
                multi_robots_info_pub_msg.data = [x.encode(encoding='utf-8') for x in msg.__str__()]
                self.multi_robots_info_pub.publish(multi_robots_info_pub_msg)
    



def main(args=None):
        rclpy.init(args=args)

        minimal_subscriber = Multi_Info_Pub()

        rclpy.spin(minimal_subscriber)

        # Destroy the node explicitly
        # (optional - otherwise it will be done automatically
        # when the garbage collector destroys the node object)
        minimal_subscriber.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
        main()
