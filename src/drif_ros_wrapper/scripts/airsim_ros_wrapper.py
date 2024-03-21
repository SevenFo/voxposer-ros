import rospy
import msgpackrpc

from nav_msgs.msg import Odometry
from sensor_msgs.msg import CameraInfo, Image
from sensor_msgs.msg import Imu, MagneticField, Range, PointCloud2
from sensor_msgs.msg import NavSatFix
from tf2_msgs.msg import TFMessage

from airsim_ros_pkgs.msg import GPSYaw
from airsim_ros_pkgs.msg import VelCmd, GimbalAngleEulerCmd, GimbalAngleQuatCmd
from airsim_ros_pkgs.srv import Takeoff, Reset
from airsim_ros_pkgs.msg import Altimeter

from airsimClient import MultirotorClient
class AirSimROSWrapper:
    def __init__(self):
        # 初始化ROS节点
        rospy.init_node('airsim_ros_wrapper')
        
        # 初始化参数
        airsim_client_host = rospy.get_param('/airsim_node/airsim_client_host', 'localhost')
        airsim_client_port = rospy.get_param('/airsim_node/airsim_client_port', 41451)
        update_airsim_img_response_every_n_sec = rospy.get_param('/airsim_node/update_airsim_img_response_every_n_sec', 0.1)
        
        # airsim client initialization
        self.multirotor_client = MultirotorClient(ip=airsim_client_host, port=airsim_client_port)
        self.multirotor_client.confirmConnection()
        try:
            self.multirotor_client.enableApiControl(True)
            self.client.armDisarm(True) # 启动无人机的电机
        except Exception as e:
            rospy.logerr("Error enabling control of the drone: {0}".format(e))
            return None
        
        # create timer
        self.update_airsim_img_response_timer = rospy.Timer(rospy.Duration(update_airsim_img_response_every_n_sec), self.update_airsim_img_response_timer_callback)
        
        # 创建发布者
        # self.origin_geo_point_pub = rospy.Publisher('/airsim_node/origin_geo_point', GPSYaw, queue_size=10)
        # self.global_gps_pub = rospy.Publisher('/airsim_node/VEHICLE_NAME/global_gps', NavSatFix, queue_size=10)
        self.odom_local_ned_pub = rospy.Publisher('/airsim_node/VEHICLE_NAME/odom_local_ned', Odometry, queue_size=10)  # 发布本地NED坐标系的里程计数据
        self.camera_info_pub = rospy.Publisher('/airsim_node/VEHICLE_NAME/CAMERA_NAME/IMAGE_TYPE/camera_info', CameraInfo, queue_size=10)  # 发布相机信息
        self.image_pub = rospy.Publisher('/airsim_node/VEHICLE_NAME/CAMERA_NAME/IMAGE_TYPE', Image, queue_size=10)  # 发布图像数据
        self.tf_pub = rospy.Publisher('/tf', TFMessage, queue_size=10)  # 发布TF变换
        self.altimeter_pub = rospy.Publisher('/airsim_node/VEHICLE_NAME/altimeter/SENSOR_NAME', Altimeter, queue_size=10)  # 发布气压计数据
        self.imu_pub = rospy.Publisher('/airsim_node/VEHICLE_NAME/imu/SENSOR_NAME', Imu, queue_size=10)  # 发布IMU数据
        self.magnetometer_pub = rospy.Publisher('/airsim_node/VEHICLE_NAME/magnetometer/SENSOR_NAME', MagneticField, queue_size=10)  # 发布磁力计数据
        self.distance_pub = rospy.Publisher('/airsim_node/VEHICLE_NAME/distance/SENSOR_NAME', Range, queue_size=10)  # 发布距离传感器数据
        self.lidar_pub = rospy.Publisher('/airsim_node/VEHICLE_NAME/lidar/SENSOR_NAME', PointCloud2, queue_size=10)  # 发布激光雷达数据

        # 创建订阅者
        self.vel_cmd_body_frame_sub = rospy.Subscriber('/airsim_node/vel_cmd_body_frame', VelCmd, self.vel_cmd_body_frame_callback)  # 订阅来自airsim_node的vel_cmd_body_frame消息
        self.vel_cmd_world_frame_sub = rospy.Subscriber('/airsim_node/vel_cmd_world_frame', VelCmd, self.vel_cmd_world_frame_callback)  # 订阅来自airsim_node的vel_cmd_world_frame消息
        self.gimbal_angle_euler_cmd_sub = rospy.Subscriber('/gimbal_angle_euler_cmd', GimbalAngleEulerCmd, self.gimbal_angle_euler_cmd_callback)  # 订阅来自gimbal_angle_euler_cmd的消息
        self.gimbal_angle_quat_cmd_sub = rospy.Subscriber('/gimbal_angle_quat_cmd', GimbalAngleQuatCmd, self.gimbal_angle_quat_cmd_callback)  # 订阅来自gimbal_angle_quat_cmd的消息

        # 创建服务
        self.land_service = rospy.Service('/airsim_node/VEHICLE_NAME/land', Takeoff, self.land_service_callback)  # 创建land服务
        self.takeoff_service = rospy.Service('/airsim_node/takeoff', Takeoff, self.takeoff_service_callback)  # 创建takeoff服务
        self.reset_service = rospy.Service('/airsim_node/reset', Reset, self.reset_service_callback)  # 创建reset服务

        # 设置参数
        rospy.set_param('/airsim_node/world_frame_id', 'world_ned')
        rospy.set_param('/airsim_node/odom_frame_id', 'odom_local_ned')
        rospy.set_param('/airsim_node/coordinate_system_enu', False)
        rospy.set_param('/airsim_node/update_airsim_control_every_n_sec', 0.01)
        rospy.set_param('/airsim_node/update_airsim_img_response_every_n_sec', 0.01)
        rospy.set_param('/airsim_node/publish_clock', False)
        
    def update_airsim_img_response_timer_callback(self, event):
        # 处理update_airsim_img_response_timer回调
        try
            image_response_idx = 0
            
        pass

    def vel_cmd_body_frame_callback(self, msg):
        # 处理vel_cmd_body_frame回调
        pass

    def vel_cmd_world_frame_callback(self, msg):
        # 处理vel_cmd_world_frame回调
        pass

    def gimbal_angle_euler_cmd_callback(self, msg):
        # 处理gimbal_angle_euler_cmd回调
        pass

    def gimbal_angle_quat_cmd_callback(self, msg):
        # 处理gimbal_angle_quat_cmd回调
        pass

    def car_cmd_callback(self, msg):
        # 处理car_cmd回调
        pass

    def land_service_callback(self, req):
        # 处理land服务回调
        pass

    def takeoff_service_callback(self, req):
        # 处理takeoff服务回调
        pass

    def reset_service_callback(self, req):
        # 处理reset服务回调
        pass

    def run(self):
        # 运行ROS节点
        rospy.spin()

if __name__ == '__main__':
    wrapper = AirSimROSWrapper()
    wrapper.run()
