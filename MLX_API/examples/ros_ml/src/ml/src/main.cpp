#include <ros/ros.h>
#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <pcl_ros/point_cloud.h>
#include "ml/libsoslab_ml.h"

typedef pcl::PointXYZRGB PointRGB_T;
typedef pcl::PointCloud<PointRGB_T> PointCloud_T;

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

image_transport::Publisher pub_depth;
image_transport::Publisher pub_intensity;
image_transport::Publisher pub_ambient;
ros::Publisher pub_lidar;

sensor_msgs::ImagePtr msg_ambient;
sensor_msgs::ImagePtr msg_depth;
sensor_msgs::ImagePtr msg_intensity;

PointCloud_T::Ptr msg_pointcloud(new PointCloud_T);

int max_ambient_img_val = 30000;
int max_depth_img_val = 10000;
int max_intensity_img_val = 3500;

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

void ml_scene_data_callback(void* arg, SOSLAB::LidarML::scene_t& scene)
{
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
        if (pub_ambient.getNumSubscribers() > 0) {
            msg_ambient = cv_bridge::CvImage(std_msgs::Header(), "rgb8", ambient_image).toImageMsg();
            pub_ambient.publish(msg_ambient);
        }
    }

    /* Depth Image */
    if(!scene.depth_image.empty()){
        depth = scene.depth_image[0];

        cv::Mat depth_image(height, width, CV_32SC1, depth.data());

        depth_image.convertTo(depth_image, CV_16U);
        depth_image.convertTo(depth_image, CV_8UC1, (255.0 / (max_depth_img_val - 0)), 0);
        
        depth_image = colormap(depth_image);
        cv::normalize(depth_image, depth_image, 0, 255, cv::NORM_MINMAX);
        if(pub_depth.getNumSubscribers() > 0) {
            msg_depth = cv_bridge::CvImage(std_msgs::Header(), "rgb8", depth_image).toImageMsg();
            pub_depth.publish(msg_depth);
        }
    }

    /* Intensity Image */
    cv::Mat intensity_image;
    if(!scene.intensity_image.empty()){
        intensity = scene.intensity_image[0];
        cv::Mat intensity_image_raw(height, width, CV_16UC1, intensity.data());
        intensity_image_raw.convertTo(intensity_image, CV_8UC1, (255.0 / (max_intensity_img_val - 0)), 0);
        intensity_image = colormap(intensity_image);

        if (pub_intensity.getNumSubscribers() > 0) {
            msg_intensity = cv_bridge::CvImage(std_msgs::Header(), "rgb8", intensity_image).toImageMsg();
            pub_intensity.publish(msg_intensity);
        }
    }

    /* Point Cloud */
    msg_pointcloud->header.frame_id = DEFAULT_FRAME_ID;
    msg_pointcloud->width = width;
    msg_pointcloud->height = height;
    msg_pointcloud->points.resize(pointcloud.size());

    for (int col=0; col < width; col++) {
        for (int row = 0; row < height; row++) {
            int idx = col + (width * row);

            //unit : (m)
            msg_pointcloud->points[idx].x = pointcloud[idx].x / 1000.0 ;
            msg_pointcloud->points[idx].y = pointcloud[idx].y / 1000.0 ;
            msg_pointcloud->points[idx].z = pointcloud[idx].z / 1000.0 ;

            if(!scene.intensity_image.empty()){
                msg_pointcloud->points[idx].r = (uint8_t)(intensity_image.at<cv::Vec3b>(row, col)[0]);
                msg_pointcloud->points[idx].g = (uint8_t)(intensity_image.at<cv::Vec3b>(row, col)[1]);
                msg_pointcloud->points[idx].b = (uint8_t)(intensity_image.at<cv::Vec3b>(row, col)[2]);
            }
            else{
                msg_pointcloud->points[idx].r = 255;
                msg_pointcloud->points[idx].g = 255;
                msg_pointcloud->points[idx].b = 255;
            }
        }
    }
    // publish the pointcloud
    pcl_conversions::toPCL(ros::Time::now(), msg_pointcloud->header.stamp);
    pub_lidar.publish(msg_pointcloud);
}

int main (int argc, char **argv)
{
    bool success;
    /* ROS node init */
    ros::init(argc, argv, DEFAULT_PACKAGE_NAME);

    /* FPS 10 */
    bool fps10_enable                     = false;

    /* Depth Completion */
    bool depth_completion_enable          = false;

    /* Data Selection */
    bool ambient_enable                   = true;
    bool depth_enable                     = true;
    bool intensity_enable                 = true;

    /* get parameters */
    ros::NodeHandle nh("~");

    nh.param<bool>("fps10", fps10_enable, false);
    nh.param<bool>("depth_completion_enable", depth_completion_enable, false);

    nh.param<bool>("ambient_enable", ambient_enable, true);
    nh.param<bool>("depth_enable", depth_enable, true);
    nh.param<bool>("intensity_enable", intensity_enable, true);

    /* publisher setting */
    image_transport::ImageTransport it(nh);

    pub_depth = it.advertise("depth_color", 1);
    pub_intensity = it.advertise("intensity_color", 1);
    pub_ambient = it.advertise("ambient_color", 1);
    pub_lidar = nh.advertise<PointCloud_T>("pointcloud", 10);

    SOSLAB::ip_settings_t ip_settings_device;
    SOSLAB::ip_settings_t ip_settings_pc;

    nh.param<std::string>("ip_address_device", ip_settings_device.ip_address, DEFAULT_IP_ADDR_DEVICE);
    nh.param<int>("ip_port_device", ip_settings_device.port_number, DEFAULT_IP_PORT_DEVICE);
    nh.param<std::string>("ip_address_pc", ip_settings_pc.ip_address, DEFAULT_IP_ADDR_PC);
    nh.param<int>("ip_port_pc", ip_settings_pc.port_number, DEFAULT_IP_PORT_PC);

    std::shared_ptr<SOSLAB::LidarML> lidar_ml(new SOSLAB::LidarML);

    std::cout << lidar_ml->api_info() << std::endl;
    std::cout << "> ip_address_device: " << ip_settings_device.ip_address << std::endl;
    std::cout << "> ip_port_device: " << ip_settings_device.port_number << std::endl;
    std::cout << "> ip_address_pc: " << ip_settings_pc.ip_address << std::endl;
    std::cout << "> ip_port_pc: " << ip_settings_pc.port_number << std::endl;

	success = lidar_ml->connect(ip_settings_device, ip_settings_pc);
	if (!success) {
		std::cerr << "LiDAR ML :: connection failed." << std::endl;
		return 0;
	}

    lidar_ml->ambient_enable(ambient_enable);       //Ambient enable (True / False)
    lidar_ml->depth_enable(depth_enable);           //Depth enable (True / False)
    lidar_ml->intensity_enable(intensity_enable);   //Intensity enable (True / False)

    /* FPS 10 */
    lidar_ml->fps10(fps10_enable);
    /* Depth Completion */
    lidar_ml->depth_completion(depth_completion_enable);

    lidar_ml->register_scene_callback(ml_scene_data_callback, nullptr);
	success = lidar_ml->run();

	if (!success) {
		std::cerr << "LiDAR ML :: run failed." << std::endl;
	}
	else {
		std::cout << "LiDAR ML :: run." << std::endl;
	}
    std::cout << "LiDAR ML :: Streaming started!" << std::endl;

    /* publishing start */
    ros::Rate r(50);
    while (ros::ok()) {
        ros::spinOnce();
        r.sleep();
    }

    lidar_ml->stop();
    std::cout << "Streaming stopped!" << std::endl;

    lidar_ml->disconnect();

    std::cout << "Done." << std::endl;

    return 0;
}
