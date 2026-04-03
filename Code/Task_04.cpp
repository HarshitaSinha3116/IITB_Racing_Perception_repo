#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include<geometry_msgs/msg/point32.hpp>
#include<sensor_msgs/msg/channel_float32.hpp>

#include <vector>
#include <cmath>
#include <cstdlib>
#include <ctime>

struct Point
{
    float x, y, z, intensity;
};

struct Plane
{
    float a, b, c, d;
};

class ConeDetector : public rclcpp::Node
{
public:
    ConeDetector() : Node("cone_detector")
    {

        sub_ = this->create_subscription<sensor_msgs::msg::PointCloud>(
            "/carmaker/pointcloud",
            rclcpp::SensorDataQoS(),
            std::bind(&ConeDetector::callback, this, std::placeholders::_1));

        marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
            "/cone_markers", 10);

        std::srand(std::time(nullptr));

        RCLCPP_INFO(this->get_logger(), "Cone Detector Node Started");
    }

private:
    rclcpp::Subscription<sensor_msgs::msg::PointCloud>::SharedPtr sub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
    std::vector<int> best_inliers;

   std::vector<Point> convertCloud(const sensor_msgs::msg::PointCloud::SharedPtr msg)
{
    std::vector<Point> cloud;

    int intensity_channel = -1;

    for (size_t c = 0; c < msg->channels.size(); c++)
    {
        if (msg->channels[c].name == "intensity")
        {
            intensity_channel = c;
            break;
        }
    }

    for (size_t i = 0; i < msg->points.size(); i++)
    {
        Point p;

        p.x = msg->points[i].x;
        p.y = msg->points[i].y;
        p.z = msg->points[i].z;

        if (intensity_channel != -1 &&
            msg->channels[intensity_channel].values.size() > i)
        {
            p.intensity = msg->channels[intensity_channel].values[i];
        }
        else
        {
            p.intensity = 0.0;
        }

        cloud.push_back(p);
    }

    return cloud;
}



    Plane computePlane(Point p1, Point p2, Point p3)
    {
        float v1x = p2.x - p1.x;
        float v1y = p2.y - p1.y;
        float v1z = p2.z - p1.z;

        float v2x = p3.x - p1.x;
        float v2y = p3.y - p1.y;
        float v2z = p3.z - p1.z;

        float a = v1y * v2z - v1z * v2y;
        float b = v1z * v2x - v1x * v2z;
        float c = v1x * v2y - v1y * v2x;
        float d = -(a * p1.x + b * p1.y + c * p1.z);

        return {a, b, c, d};
    }

    float distanceToPlane(Point p, Plane plane)
    {
        return std::fabs(
                   plane.a * p.x + plane.b * p.y + plane.c * p.z + plane.d) /
               std::sqrt(
                   plane.a * plane.a + plane.b * plane.b + plane.c * plane.c);
    }

    Plane ransac(std::vector<Point> &cloud)
    {

        RCLCPP_INFO(this->get_logger(), "Running RANSAC on %ld points", cloud.size());

        best_inliers.clear();
        int max_iterations = 200;
        float threshold = 0.02;

        Plane best_plane;

        for (int i = 0; i < max_iterations; i++)
        {
            int i1 = rand() % cloud.size();
            int i2 = rand() % cloud.size();
            int i3 = rand() % cloud.size();

            if (i1 == i2 || i2 == i3 || i1 == i3)
                continue;

            Plane plane = computePlane(cloud[i1], cloud[i2], cloud[i3]);

            std::vector<int> current_inliers;

            for (int i = 0; i < cloud.size(); i++)
            {
                float dist = distanceToPlane(cloud[i], plane);

                if (dist < threshold)
                    current_inliers.push_back(i);
            }

            if (current_inliers.size() > best_inliers.size())
            {
                best_inliers = current_inliers;
                best_plane = plane;
            }
        }
        RCLCPP_INFO(this->get_logger(), "RANSAC finished. Inliers: %ld", best_inliers.size());

        return best_plane;
    }

    std::vector<std::vector<int>> dbscan(std::vector<Point> &cloud)
    {

        float eps = 0.2;
        int min_pts = 8;

        std::vector<bool> visited(cloud.size(), false);
        std::vector<std::vector<int>> clusters;

        for (size_t i = 0; i < cloud.size(); i++)
        {
            if (visited[i])
                continue;
            visited[i] = true;

            std::vector<int> neighbors = regionQuery(cloud, i, eps);

            if (neighbors.size() < min_pts)
                continue;

            std::vector<int> cluster;
            cluster.push_back(i);

            for (int j = 0; j < neighbors.size(); j++)
            {
                int n = neighbors[j];

                if (!visited[n])
                {
                    visited[n] = true;

                    auto new_neighbors = regionQuery(cloud, n, eps);

                    if (new_neighbors.size() >= min_pts)
                        neighbors.insert(neighbors.end(), new_neighbors.begin(), new_neighbors.end());
                }
                cluster.push_back(n);
            }
            clusters.push_back(cluster);
        }

        RCLCPP_INFO(this->get_logger(), "DBSCAN found %ld clusters", clusters.size());

        return clusters;
    }

    std::vector<int> regionQuery(std::vector<Point> &cloud, int index, float eps)
    {
        std::vector<int> neighbors;

        for (size_t i = 0; i < cloud.size(); i++)
        {
            float dx = cloud[i].x - cloud[index].x;
            float dy = cloud[i].y - cloud[index].y;
            float dz = cloud[i].z - cloud[index].z;

            float dist = std::sqrt(dx * dx + dy * dy + dz * dz);

            if (dist < eps)
                neighbors.push_back(i);
        }

        return neighbors;
    }
     
    float computeQuadraticA(const std::vector<Point> &cluster_points)
{
    float sum_z = 0;
    float sum_z2 = 0;
    float sum_z3 = 0;
    float sum_z4 = 0;

    float sum_i = 0;
    float sum_zi = 0;
    float sum_z2i = 0;

    int n = cluster_points.size();

    for(const auto &p : cluster_points)
    {
        float z = p.z;
        float I = p.intensity;

        sum_z  += z;
        sum_z2 += z*z;
        sum_z3 += z*z*z;
        sum_z4 += z*z*z*z;

        sum_i  += I;
        sum_zi += z*I;
        sum_z2i += z*z*I;
    }

    float A[3][3] = {
        {sum_z4, sum_z3, sum_z2},
        {sum_z3, sum_z2, sum_z},
        {sum_z2, sum_z,  (float)n}
    };

    float B[3] = {sum_z2i, sum_zi, sum_i};

    float detA =
        A[0][0]*(A[1][1]*A[2][2] - A[1][2]*A[2][1]) -
        A[0][1]*(A[1][0]*A[2][2] - A[1][2]*A[2][0]) +
        A[0][2]*(A[1][0]*A[2][1] - A[1][1]*A[2][0]);

    if(std::fabs(detA) < 1e-6)
        return 0;

    float detA1 =
        B[0]*(A[1][1]*A[2][2] - A[1][2]*A[2][1]) -
        A[0][1]*(B[1]*A[2][2] - A[1][2]*B[2]) +
        A[0][2]*(B[1]*A[2][1] - A[1][1]*B[2]);

    float a = detA1 / detA;

    return a;
}
   

    
    void callback(const sensor_msgs::msg::PointCloud::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), "PointCloud message received");
        auto cloud = convertCloud(msg);

        RCLCPP_INFO(this->get_logger(), "Converted cloud size: %ld", cloud.size());

        if (cloud.size() < 50)
        {
            RCLCPP_WARN(this->get_logger(), "Point cloud too small");
            return;
        }

        RCLCPP_INFO(this->get_logger(), "Starting RANSAC ground removal");

        Plane ground_plane = ransac(cloud);

        std::vector<bool> is_ground(cloud.size(), false);

        for (auto idx : best_inliers)
            is_ground[idx] = true;

        std::vector<Point> cloud_no_ground;

       
        for (int i = 0; i < cloud.size(); i++){
            if (!is_ground[i])
                cloud_no_ground.push_back(cloud[i]);
        }

        RCLCPP_INFO(this->get_logger(), "Non-ground points: %ld", cloud_no_ground.size());


        RCLCPP_INFO(this->get_logger(), "Running DBSCAN clustering");

        auto clusters = dbscan(cloud_no_ground);

        visualization_msgs::msg::MarkerArray marker_array;

        visualization_msgs::msg::Marker delete_marker;
        delete_marker.action = visualization_msgs::msg::Marker::DELETEALL;
        marker_array.markers.push_back(delete_marker);

        int id = 0;

        for (auto &cluster : clusters)
        {
            RCLCPP_INFO(this->get_logger(), "Processing cluster with %ld points", cluster.size());
            float cx = 0, cy = 0, cz = 0;
            for (int idx : cluster)
            {
                cx += cloud_no_ground[idx].x;
                cy += cloud_no_ground[idx].y;
                cz += cloud_no_ground[idx].z;
            }

            cx /= cluster.size();
            cy /= cluster.size();
            cz /= cluster.size();

            std::vector<Point> cluster_points;

            for (int idx : cluster){
        
            cluster_points.push_back(cloud_no_ground[idx]);
    }

            float a = computeQuadraticA(cluster_points);
            RCLCPP_INFO(this->get_logger(),"Polynomial coefficient a = %f", a);

            visualization_msgs::msg::Marker marker;

            marker.header.frame_id = "Lidar_F";
            marker.header.stamp = this->get_clock()->now();
            marker.ns = "cones";
            marker.id = id++;
            marker.type = visualization_msgs::msg::Marker::SPHERE;
            marker.action = visualization_msgs::msg::Marker::ADD;

            marker.pose.position.x = cx;
            marker.pose.position.y = cy;
            marker.pose.position.z = cz;

            marker.scale.x = 0.3;
            marker.scale.y = 0.3;
            marker.scale.z = 0.3;
            
            float threshold = 1e-5;
            if(a > threshold){
            marker.color.r = 0.0;
            marker.color.g = 0.0;
            marker.color.b = 1.0;
            }
            else if(a<-threshold){


            marker.color.r = 1.0;
            marker.color.g = 1.0;
            marker.color.b = 0.0;
            }

            marker.color.a = 1.0;

            marker.lifetime = rclcpp::Duration::from_seconds(0);

            marker_array.markers.push_back(marker);
        }

        RCLCPP_INFO(this->get_logger(), "Publishing %ld cone markers", marker_array.markers.size());

        marker_pub_->publish(marker_array);
        RCLCPP_INFO(this->get_logger(), "Markers published successfully");
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    RCLCPP_INFO(rclcpp::get_logger("cone_detector"), "Starting node...");
    rclcpp::spin(std::make_shared<ConeDetector>());
    rclcpp::shutdown();
    return 0;
}



