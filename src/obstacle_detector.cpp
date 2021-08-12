#include <rclcpp/rclcpp.hpp>
#include <opencv2/opencv.hpp>
#include <ryusei/common/defs.hpp>
#include <ryusei/navi/obstacle_detector.hpp>
#include <ryusei/navi/potential.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <sensor_msgs/msg/point_cloud.hpp>
#include <ros_mercury_interfaces/msg/mercury_state_msg.hpp>
#include <ros2_rs_interfaces/msg/obstacles.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>

#include <string>
#include <thread>
#include <mutex>
#include <chrono>

namespace rs = project_ryusei;
using namespace std;
using namespace std::chrono_literals;

namespace project_ryusei
{

class ROS2ObstacleDetector : public rclcpp::Node
{
public:
  ROS2ObstacleDetector(rclcpp::NodeOptions);
  ~ROS2ObstacleDetector();
private:
  void execute();
  void onLidarSubscribed(sensor_msgs::msg::PointCloud::SharedPtr msg);

  rs::ObstacleDetector detector_;
  std::vector<rs::Obstacle> obstacles_;
  
  sensor_msgs::msg::PointCloud point_cloud_;
  std::string RESOURCE_DIRECTORY_NAME_;
  std::string INIT_FILE_NAME_;

  rclcpp::Subscription<sensor_msgs::msg::PointCloud>::SharedPtr sub_lidar_;
  rclcpp::Publisher<ros2_rs_interfaces::msg::Obstacles>::SharedPtr pub_obstacles_;

  std::mutex mutex_;
};

/*** コンストラクタ ***/
ROS2ObstacleDetector::ROS2ObstacleDetector(rclcpp::NodeOptions options) : Node("rs_obstacle_detector", options)
{

  using std::placeholders::_1;

  /*** パラメータ ***/
  RESOURCE_DIRECTORY_NAME_ = this->declare_parameter<std::string>("resource_directory_name", "");
  INIT_FILE_NAME_          = this->declare_parameter<std::string>("init_file_name", "");

  /*** トピック ***/
  sub_lidar_     = this->create_subscription<sensor_msgs::msg::PointCloud>("/pandar_40/points", 10,
                          std::bind(&ROS2ObstacleDetector::onLidarSubscribed, this, _1));
  pub_obstacles_ = this->create_publisher<ros2_rs_interfaces::msg::Obstacles>("~/obstacles", 10);

  /*** parameterからファイル名を取得 ***/
  std::string init_path;
  init_path = ament_index_cpp::get_package_share_directory("ros2_rs_obstacle_detector");
  init_path += "/" + RESOURCE_DIRECTORY_NAME_ + "/";
  init_path += INIT_FILE_NAME_;

  /*** ObstacleDetectorの設定ファイル読み込み ***/
  if(!detector_.init(init_path))
  {
    RCLCPP_ERROR(this->get_logger(), "Failed to load config file");
  }

  /*** 別スレッドで実行 ***/
  std::thread t1{&ROS2ObstacleDetector::execute, this};
  t1.detach();
}

/*** デストラクタ ***/
ROS2ObstacleDetector::~ROS2ObstacleDetector()
{

}

/****************************************************************
* LiDARに対してのコールバック関数
*****************************************************************/
void ROS2ObstacleDetector::onLidarSubscribed(const sensor_msgs::msg::PointCloud::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(mutex_);
  point_cloud_ = *msg;
}

/***************************************
* 実行関数 
***************************************/
void ROS2ObstacleDetector::execute()
{
  float a = 0.0;
  while(1)
  {
    cv::Point3f map_point;
    rs::LocalMap local_map;

    geometry_msgs::msg::Point32 test;
    // test.x = 0.0;
    // test.y = 0.0;
    // point_cloud_.points.push_back(test);
    // if(a < 0.5){
    //   a += 0.1;
    // }else{
    //   a = 0.0;
    // }
    // test.x = a;
    // test.y = a;
    // point_cloud_.points.push_back(test);

    mutex_.lock();

    /*** vectorの大きさを確保 ***/
    local_map.points.resize(point_cloud_.points.size());

    /*** PointCloud型をLocalMap型に移す ***/
    for(int i = 0, size = point_cloud_.points.size(); i < size; i++){
      local_map.points[i].x = point_cloud_.points[i].x;
      local_map.points[i].y = point_cloud_.points[i].y;
      local_map.points[i].z = 1.0;
    }

    point_cloud_.points.clear();

    mutex_.unlock();

    RCLCPP_INFO_STREAM(this->get_logger(), "localmappoint_SIZE: " << local_map.points.size());

    /*** 障害物の検出 ***/
    local_map.pose.x = 0.0;
    local_map.pose.y = 0.0;
    local_map.pose.yaw = 0.0;
    detector_.visualizeLocalMap(local_map);
    detector_.detect(local_map.pose, obstacles_);

    RCLCPP_INFO_STREAM(this->get_logger(), "obstacles_SIZE: " << obstacles_.size());

    geometry_msgs::msg::Point32 obstacle_point;
    geometry_msgs::msg::Polygon obstacle_points;
    ros2_rs_interfaces::msg::Obstacles obs_msg;

    /*** 独自メッセージ型ros2_rs_interfaces::msg::Obstaclesに移す ***/
    for(int i = 0; i < obstacles_.size(); i++)
    {
      if(obstacles_[i].abs.size() == 2)  // 直線障害物
      {
        for(int j = 0; j < 2; j++){
          obstacle_point.x = obstacles_[i].abs[j].x;
          obstacle_point.y = obstacles_[i].abs[j].y;
          obstacle_points.points.push_back(obstacle_point);
        }
        obs_msg.abs.push_back(obstacle_points);
        obstacle_points.points.clear();

        for(int j = 0; j < 2; j++){
          obstacle_point.x = obstacles_[i].rel[j].x;
          obstacle_point.y = obstacles_[i].rel[j].y;
          obstacle_points.points.push_back(obstacle_point);
        }
        obs_msg.rel.push_back(obstacle_points);
        obstacle_points.points.clear();
      }

      if(obstacles_[i].abs.size() == 4)  // 矩形障害物
      {
        for(int j = 0; j < 4; j++){
          obstacle_point.x = obstacles_[i].abs[j].x;
          obstacle_point.y = obstacles_[i].abs[j].y;
          obstacle_points.points.push_back(obstacle_point);
        }
        obs_msg.abs.push_back(obstacle_points);
        obstacle_points.points.clear();

        for(int j = 0; j < 4; j++){
          obstacle_point.x = obstacles_[i].rel[j].x;
          obstacle_point.y = obstacles_[i].rel[j].y;
          obstacle_points.points.push_back(obstacle_point);
        }
        obs_msg.rel.push_back(obstacle_points);
        obstacle_points.points.clear();
      }
    }

    /*** 障害物をパブリッシュ ***/
    pub_obstacles_ -> publish(obs_msg);

    local_map.points.clear();
    obstacles_.clear();

    /*** ループ待機 ***/
    rclcpp::WallRate loop_rate(50ms);
    loop_rate.sleep();
  }
}

}  // project_ryusei

/*** PotentialAngleクラスをコンポーネントとして登録 ***/
#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(project_ryusei::ROS2ObstacleDetector)