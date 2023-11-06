#include <iostream>

#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <rclcpp/rclcpp.hpp>
#include <cv_bridge/cv_bridge.h>

#include "ml/libsoslab_ml.h"

using namespace std::chrono_literals;

int nCols = 0;
int nRows = 0;

unsigned char soslab_r[] = {3, 4, 5, 5, 6, 6, 7, 8, 8, 9, 9, 10, 10, 11, 12, 12, 13, 13, 14, 15, 15, 16, 16, 17, 18, 18, 19, 19, 20, 21, 21, 22, 22, 23, 24, 24, 25, 25, 26, 27, 27, 28, 28, 29, 29, 30, 31, 31, 32, 32, 33, 34, 34, 35, 35, 36, 37, 37, 38, 38, 39, 40, 40, 41, 41, 42, 42, 43, 43, 43, 44, 44, 45, 45, 46, 46, 46, 47, 47, 48, 48, 49, 49, 49, 50, 50, 51, 51, 51, 52, 52, 53, 53, 54, 54, 54, 55, 55, 56, 56, 57, 57, 57, 58, 58, 59, 59, 60, 60, 60, 61, 61, 62, 62, 62, 63, 63, 64, 64, 65, 65, 66, 66, 66, 67, 67, 68, 68, 70, 71, 73, 75, 77, 79, 80, 82, 84, 86, 88, 90, 92, 93, 95, 97, 99, 101, 102, 104, 106, 108, 110, 111, 113, 115, 117, 119, 120, 122, 124, 126, 128, 129, 131, 133, 135, 137, 138, 140, 142, 144, 145, 147, 149, 151, 153, 154, 156, 158, 160, 162, 163, 165, 167, 169, 171, 172, 174, 176, 178, 180, 181, 183, 184, 185, 187, 188, 189, 190, 191, 192, 194, 195, 196, 197, 198, 199, 200, 201, 203, 204, 205, 206, 207, 208, 209, 210, 212, 213, 214, 215, 216, 217, 218, 219, 221, 222, 223, 224, 225, 226, 227, 228, 230, 231, 232, 233, 234, 235, 236, 237, 239, 240, 241, 242, 243, 244, 245, 246, 248, 249, 250, 251, 252, 253, 254, 254};
unsigned char soslab_g[] = {18, 19, 21, 23, 24, 26, 27, 29, 30, 32, 33, 35, 37, 38, 40, 41, 43, 44, 46, 47, 49, 50, 52, 54, 55, 57, 58, 60, 61, 63, 64, 66, 67, 69, 71, 72, 74, 75, 77, 78, 80, 81, 83, 84, 86, 88, 89, 91, 92, 94, 95, 97, 98, 100, 101, 103, 105, 106, 108, 109, 111, 112, 114, 116, 117, 118, 119, 120, 121, 122, 124, 125, 126, 127, 128, 129, 130, 131, 132, 133, 134, 136, 137, 138, 139, 140, 141, 142, 143, 144, 145, 146, 148, 149, 150, 151, 152, 153, 154, 155, 156, 157, 159, 160, 161, 162, 163, 164, 165, 166, 167, 168, 169, 171, 172, 173, 174, 175, 176, 177, 178, 180, 181, 182, 183, 184, 185, 186, 187, 188, 189, 190, 191, 192, 193, 194, 196, 197, 198, 199, 200, 201, 202, 203, 204, 206, 207, 208, 209, 210, 211, 212, 213, 214, 215, 216, 217, 218, 220, 221, 222, 223, 224, 225, 226, 227, 228, 229, 230, 231, 232, 234, 235, 236, 237, 238, 239, 240, 241, 242, 243, 244, 245, 247, 248, 249, 250, 251, 252, 253, 254, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255};
unsigned char soslab_b[] = {107, 108, 109, 110, 111, 112, 113, 114, 114, 115, 116, 117, 118, 119, 120, 120, 121, 122, 123, 124, 125, 126, 126, 127, 128, 129, 130, 131, 132, 132, 133, 134, 135, 136, 137, 138, 138, 139, 140, 141, 142, 143, 144, 144, 145, 146, 147, 148, 149, 150, 151, 151, 152, 153, 154, 155, 156, 157, 157, 158, 159, 160, 161, 162, 162, 163, 163, 163, 164, 164, 164, 164, 165, 165, 165, 166, 166, 166, 167, 167, 167, 168, 168, 168, 169, 169, 169, 169, 170, 170, 170, 171, 171, 171, 172, 172, 172, 173, 173, 173, 174, 174, 174, 174, 175, 175, 175, 176, 176, 176, 177, 177, 177, 178, 178, 178, 179, 179, 179, 179, 180, 180, 180, 181, 181, 181, 182, 182, 181, 180, 179, 178, 177, 175, 175, 174, 172, 171, 170, 169, 168, 167, 166, 165, 164, 162, 161, 160, 159, 158, 157, 156, 155, 154, 153, 152, 151, 150, 148, 147, 146, 145, 144, 143, 142, 141, 140, 139, 138, 137, 136, 134, 133, 132, 131, 130, 129, 128, 127, 126, 125, 124, 123, 121, 120, 119, 118, 117, 116, 115, 114, 113, 111, 109, 107, 105, 104, 102, 100, 98, 96, 95, 93, 91, 89, 88, 86, 84, 82, 81, 79, 77, 75, 73, 72, 70, 68, 66, 65, 63, 61, 59, 58, 56, 54, 52, 51, 49, 47, 45, 43, 42, 40, 38, 36, 35, 33, 31, 29, 28, 26, 24, 22, 21, 19, 17, 15, 13, 12, 10, 8, 6, 5, 2, 1, 1};

static const char* DEFAULT_IP_ADDR_DEVICE          = "192.168.1.10";
static const unsigned short DEFAULT_IP_PORT_DEVICE = 2000;

static const char* DEFAULT_IP_ADDR_PC              = "0.0.0.0";
static const unsigned short DEFAULT_IP_PORT_PC     = 0;

static const char* DEFAULT_PACKAGE_NAME            = "ml";

// Default publisher template
static const char* DEFAULT_FRAME_ID = "map";

cv::Mat colormap(cv::Mat image)
{
    nCols = image.cols;
    nRows = image.rows;
    
    int temp_intensity = 0;

    cv::Mat rgb_image(nRows, nCols, CV_8UC3);

    for (int col = 0; col < nCols; col++){
        for (int row = 0; row < nRows; row++){
            temp_intensity = image.at<uint8_t>(row, col);
            float colormap_val0, colormap_val1, colormap_val2;
            
            colormap_val0 = soslab_r[temp_intensity];
            colormap_val1 = soslab_g[temp_intensity];
            colormap_val2 = soslab_b[temp_intensity];

            rgb_image.at<cv::Vec3b>(row, col)[0] = int(colormap_val0);
            rgb_image.at<cv::Vec3b>(row, col)[1] = int(colormap_val1);
            rgb_image.at<cv::Vec3b>(row, col)[2] = int(colormap_val2);
        }
    }
    return rgb_image;
}

class ML : public rclcpp::Node
{
public:
  ML() : Node("mlx_ros2_node")
  {
    point_cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("pointcloud", 1);
    intensity_pub_ = this->create_publisher<sensor_msgs::msg::Image>("intensity_color", 1);
    depth_pub_ = this->create_publisher<sensor_msgs::msg::Image>("depth_color", 1);
    ambient_pub_ = this->create_publisher<sensor_msgs::msg::Image>("ambient_color", 1);

    point_cloud_msg_ = std::make_unique<sensor_msgs::msg::PointCloud2>();
    ambient_msg_ = std::make_unique<sensor_msgs::msg::Image>();
    depth_msg_ = std::make_unique<sensor_msgs::msg::Image>();
    intensity_msg_ = std::make_unique<sensor_msgs::msg::Image>();

    timer = create_wall_timer(50ms, std::bind(&ML::publish, this));
  }

  ~ML()
  {
    lidar_ml->stop();
    std::cout << "Streaming stopped!" << std::endl;

    lidar_ml->disconnect();

    std::cout << "Done." << std::endl;
  }

  bool connect()
  {
    lidar_ml.reset(new SOSLAB::LidarML);

    SOSLAB::ip_settings_t ip_settings_device;
    SOSLAB::ip_settings_t ip_settings_pc;

    declare_parameter("ip_address_device", DEFAULT_IP_ADDR_DEVICE);
    declare_parameter("ip_port_device", DEFAULT_IP_PORT_DEVICE);
    declare_parameter("ip_address_pc", DEFAULT_IP_ADDR_PC);
    declare_parameter("ip_port_pc", DEFAULT_IP_PORT_PC);

    ip_settings_device.ip_address = get_parameter("ip_address_device").as_string();
    ip_settings_device.port_number = get_parameter("ip_port_device").as_int();
    ip_settings_pc.ip_address = get_parameter("ip_address_pc").as_string();
    ip_settings_pc.port_number =  get_parameter("ip_port_pc").as_int();

    std::cout << "> ip_address_device: " << ip_settings_device.ip_address << std::endl;
    std::cout << "> ip_port_device: " << ip_settings_device.port_number << std::endl;
    std::cout << "> ip_address_pc: " << ip_settings_pc.ip_address << std::endl;
    std::cout << "> ip_port_pc: " << ip_settings_pc.port_number << std::endl;

    bool success = lidar_ml->connect(ip_settings_device, ip_settings_pc);
    if (!success) {
      std::cerr << "LiDAR ML :: connection failed." << std::endl;
      return false;
    }

    /* FPS 10 */
    bool fps10_enable                     = false;

    /* Depth Completion */
    bool depth_completion_enable          = false;

    /* Data Selection */
    bool ambient_enable                   = true;
    bool depth_enable                     = true;
    bool intensity_enable                 = true;

    lidar_ml->fps10(fps10_enable);
    lidar_ml->depth_completion(depth_completion_enable);

    lidar_ml->ambient_enable(ambient_enable);
    lidar_ml->depth_enable(depth_enable);
    lidar_ml->intensity_enable(intensity_enable);


    success = lidar_ml->run();

    if (!success) {
      std::cerr << "LiDAR ML :: run failed." << std::endl;
      return false;
    }
    else {
      std::cout << "LiDAR ML :: run." << std::endl;
    }
    std::cout << "LiDAR ML :: Streaming started!" << std::endl;

    return true;
  }

  void publish()
  {
    SOSLAB::LidarML::scene_t scene;
    if (lidar_ml->get_scene(scene)) {

    point_cloud_msg_.reset(new sensor_msgs::msg::PointCloud2);
    ambient_msg_.reset(new sensor_msgs::msg::Image);
    depth_msg_.reset(new sensor_msgs::msg::Image);
    intensity_msg_.reset(new sensor_msgs::msg::Image);


    std::vector<uint32_t> ambient;
    std::vector<uint16_t> intensity;
    std::vector<uint32_t> depth;
    std::vector<SOSLAB::point_t> pointcloud = scene.pointcloud[0];

    std::size_t height = scene.rows;
    std::size_t width = scene.cols;
    std::size_t width2 = (scene.cols == 192) ? scene.cols*3 : scene.cols;

    /* Ambient Image */
    if(!scene.ambient_image.empty()){
        ambient = scene.ambient_image;

        cv::Mat ambient_image(height, width2, CV_32SC1, ambient.data());

        ambient_image.convertTo(ambient_image, CV_8UC1, (255.0 / (max_ambient_img_val - 0)), 0);
        ambient_image = colormap(ambient_image);
        cv::normalize(ambient_image, ambient_image, 0, 255, cv::NORM_MINMAX);
        
        ambient_msg_->header.frame_id = DEFAULT_FRAME_ID;
        ambient_msg_->width = width2;
        ambient_msg_->height = height;
        ambient_msg_->encoding = "rgb8";
        ambient_msg_->step = ambient_image.cols * ambient_image.elemSize();
        ambient_msg_->data.resize(0);
        ambient_msg_->data.insert(ambient_msg_->data.begin(), ambient_image.data, ambient_image.data + ambient_msg_->step * ambient_image.rows);
    }

    /* Depth Image */
    if(!scene.depth_image.empty()){
        depth = scene.depth_image[0];

        cv::Mat depth_image(height, width, CV_32SC1, depth.data());

        depth_image.convertTo(depth_image, CV_16U);
        depth_image.convertTo(depth_image, CV_8UC1, (255.0 / (max_depth_img_val - 0)), 0);
        
        depth_image = colormap(depth_image);
        cv::normalize(depth_image, depth_image, 0, 255, cv::NORM_MINMAX);
        
        depth_msg_->header.frame_id = DEFAULT_FRAME_ID;
        depth_msg_->width = width;
        depth_msg_->height = height;
        depth_msg_->encoding = "rgb8";
        depth_msg_->step = depth_image.cols * depth_image.elemSize();
        depth_msg_->data.resize(0);
        depth_msg_->data.insert(depth_msg_->data.begin(), depth_image.data, depth_image.data + depth_msg_->step * depth_image.rows);
    }

    /* Intensity Image */
    cv::Mat intensity_image;
    if(!scene.intensity_image.empty()){
        intensity = scene.intensity_image[0];
        intensity_image = cv::Mat(height, width, CV_16UC1, intensity.data());
        intensity_image.convertTo(intensity_image, CV_8UC1, (255.0 / (max_intensity_img_val - 0)), 0);
        intensity_image = colormap(intensity_image);

        intensity_msg_->header.frame_id = DEFAULT_FRAME_ID;
        intensity_msg_->width = width;
        intensity_msg_->height = height;
        intensity_msg_->encoding = "rgb8";
        intensity_msg_->step = intensity_image.cols * intensity_image.elemSize();
        intensity_msg_->data.resize(0);
        intensity_msg_->data.insert(intensity_msg_->data.begin(), intensity_image.data, intensity_image.data + intensity_msg_->step * intensity_image.rows);
    }


    /* Point Cloud */
    point_cloud_msg_->header.frame_id = DEFAULT_FRAME_ID;
    point_cloud_msg_->width = width;
    point_cloud_msg_->height = height;
    point_cloud_msg_->fields.resize(4);

    point_cloud_msg_->is_bigendian = false;
    point_cloud_msg_->is_dense = true;

    point_cloud_msg_->fields[0].name = "x";
    point_cloud_msg_->fields[0].offset = 0;
    point_cloud_msg_->fields[0].datatype = sensor_msgs::msg::PointField::FLOAT32;
    point_cloud_msg_->fields[0].count = 1;

    point_cloud_msg_->fields[1].name = "y";
    point_cloud_msg_->fields[1].offset = 4;
    point_cloud_msg_->fields[1].datatype = sensor_msgs::msg::PointField::FLOAT32;
    point_cloud_msg_->fields[1].count = 1;

    point_cloud_msg_->fields[2].name = "z";
    point_cloud_msg_->fields[2].offset = 8;
    point_cloud_msg_->fields[2].datatype = sensor_msgs::msg::PointField::FLOAT32;
    point_cloud_msg_->fields[2].count = 1;

    point_cloud_msg_->fields[3].name = "rgb";
    point_cloud_msg_->fields[3].offset = 16;
    point_cloud_msg_->fields[3].datatype = sensor_msgs::msg::PointField::FLOAT32;
    point_cloud_msg_->fields[3].count = 3;
    
    point_cloud_msg_->point_step = 20;
    point_cloud_msg_->row_step = point_cloud_msg_->point_step * width;
    point_cloud_msg_->data.resize(point_cloud_msg_->row_step * height);

    for (int col=0; col < width; col++) {
      for (int row = 0; row < height; row++) {
          int idx = col + (width * row);
          
          //unit : (m)
          float x = pointcloud[idx].x / 1000.0 ;
          float y = pointcloud[idx].y / 1000.0 ;
          float z = pointcloud[idx].z / 1000.0 ;

          uint32_t rgb = 0;

          if(!scene.intensity_image.empty()){
            cv::Vec3b color =  intensity_image.at<cv::Vec3b>(row, col);
            rgb = (static_cast<uint32_t>(color[0]) << 16 |
            static_cast<uint32_t>(color[1]) << 8 |
            static_cast<uint32_t>(color[2]));
          }

          memcpy(&point_cloud_msg_->data[idx * point_cloud_msg_->point_step + 0], &x, sizeof(float));
          memcpy(&point_cloud_msg_->data[idx * point_cloud_msg_->point_step + 4], &y, sizeof(float));
          memcpy(&point_cloud_msg_->data[idx * point_cloud_msg_->point_step + 8], &z, sizeof(float));
          memcpy(&point_cloud_msg_->data[idx * point_cloud_msg_->point_step + 16], &rgb, sizeof(uint32_t));

      }
    }

    auto c_time = this->get_clock()->now();

    point_cloud_msg_->header.stamp = c_time;
    ambient_msg_->header.stamp = c_time;
    depth_msg_->header.stamp = c_time;
    intensity_msg_->header.stamp = c_time;

    // set the values of the PointCloud2 message
    point_cloud_pub_->publish(std::move(point_cloud_msg_));

    // set the values of the Image message
    ambient_pub_->publish(std::move(ambient_msg_));
    depth_pub_->publish(std::move(depth_msg_));
    intensity_pub_->publish(std::move(intensity_msg_));
    }
  }

private:
  rclcpp::TimerBase::SharedPtr timer;

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr point_cloud_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr ambient_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr depth_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr intensity_pub_;

  std::unique_ptr<sensor_msgs::msg::PointCloud2> point_cloud_msg_;
  std::unique_ptr<sensor_msgs::msg::Image> ambient_msg_;
  std::unique_ptr<sensor_msgs::msg::Image> depth_msg_;
  std::unique_ptr<sensor_msgs::msg::Image> intensity_msg_;
  
  std::shared_ptr<SOSLAB::LidarML> lidar_ml;

  //Visualize param
  int max_ambient_img_val = 3000;
  int max_depth_img_val = 10000;
  int max_intensity_img_val = 3000;
  
};

int main(int argc, char * argv[])
{
  bool success;

  /* ROS node init */
  rclcpp::init(argc, argv);
  
  auto node = std::make_shared<ML>();
  if(!node->connect()){
    return 0;
  }

  rclcpp::spin(node);
  rclcpp::shutdown();
  
  return 0;
}
