/// Gaze Correlation

////////////////////
/// DEPENDENCIES ///
////////////////////

// ROS 2
#include <rclcpp/rclcpp.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <tf2_eigen/tf2_eigen.h>
#include <tf2_ros/transform_listener.h>

// ROS 2 interfaces
#include <gaze_msgs/msg/gaze_stamped.hpp>
#include <geometric_primitive_msgs/msg/geometric_primitive_list_stamped.hpp>
#include <geometric_primitive_msgs/msg/geometric_primitive_stamped.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>

// Eigen
#include <Eigen/Geometry>

//////////////////
/// NAMESPACES ///
//////////////////

using geometric_primitive_msgs::msg::Cylinder;
using geometric_primitive_msgs::msg::GeometricPrimitive;
using geometric_primitive_msgs::msg::Plane;
using geometric_primitive_msgs::msg::Sphere;
using namespace std::literals;

/////////////////
/// CONSTANTS ///
/////////////////

/// The name of this node
const std::string NODE_NAME = "gaze_correlation";
/// Size of the queue size used by the synchronizer in its policy
const uint8_t SYNCHRONIZER_QUEUE_SIZE = 10;

const bool VISUALISE_GAZE = true;
const float VISUAL_GAZE_LENGTH = 5.0;
const float VISUAL_GAZE_WIDTH = 0.005;
const float VISUAL_GAZE_COLOR[] = {0, 0, 1.0, 1};

const float VISUAL_POINT_OF_GAZE_SCALE = 0.0125;
const float VISUAL_POINT_OF_GAZE_COLOR[] = {1.0, 0.0, 0, 1};

/////////////
/// TYPES ///
/////////////

/// Policy of the synchronizer
typedef message_filters::sync_policies::ApproximateTime<gaze_msgs::msg::GazeStamped,
                                                        geometric_primitive_msgs::msg::GeometricPrimitiveListStamped>
    synchronizer_policy;

////////////////////////
/// HELPER FUNCTIONS ///
////////////////////////

namespace Eigen
{
void fromMsg(const geometry_msgs::msg::Vector3 &input, Eigen::Vector3d &output)
{
  output.x() = input.x;
  output.y() = input.y;
  output.z() = input.z;
}

Eigen::ParametrizedLine<double, 3> transform(Eigen::ParametrizedLine<double, 3> &parametrized_line, const Eigen::Isometry3d &transformation)
{
  return Eigen::ParametrizedLine<double, 3>(transformation * parametrized_line.origin(), transformation.rotation() * parametrized_line.direction());
}
} // namespace Eigen

//////////////////
/// NODE CLASS ///
//////////////////

/// Class representation of this node
class GazeCorrelation : public rclcpp::Node
{
public:
  /// Constructor
  GazeCorrelation();

private:
  /// Subscriber to gaze
  message_filters::Subscriber<gaze_msgs::msg::GazeStamped> sub_gaze_;
  /// Subscriber to geometry primitives
  message_filters::Subscriber<geometric_primitive_msgs::msg::GeometricPrimitiveListStamped> sub_primitives_;
  /// Synchronizer of the subscribers
  message_filters::Synchronizer<synchronizer_policy> synchronizer_;

  /// Publisher of the correlated primitive
  rclcpp::Publisher<geometric_primitive_msgs::msg::GeometricPrimitiveStamped>::SharedPtr pub_object_of_interest_;
  /// Publisher of visualisation markers
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_markers_;

  /// Buffer for tf2 transforms
  tf2_ros::Buffer tf2_buffer;
  /// Listener of tf2 transforms
  tf2_ros::TransformListener tf_listener;

  /// Callback called each time a message is received on all topics
  void synchronized_callback(const gaze_msgs::msg::GazeStamped::SharedPtr msg_gaze,
                             const geometric_primitive_msgs::msg::GeometricPrimitiveListStamped::SharedPtr msg_primitives);
};

GazeCorrelation::GazeCorrelation() : Node(NODE_NAME),
                                     sub_gaze_(this, "gaze"),
                                     sub_primitives_(this, "geometric_primitives"),
                                     synchronizer_(synchronizer_policy(SYNCHRONIZER_QUEUE_SIZE), sub_gaze_, sub_primitives_),
                                     tf2_buffer(this->get_clock()),
                                     tf_listener(tf2_buffer)
{
  // Synchronize the subscriptions under a single callback
  synchronizer_.registerCallback(&GazeCorrelation::synchronized_callback, this);

  // Parameters
  bool publish_markers = this->declare_parameter<bool>("publish_markers", true);
  this->declare_parameter<bool>("enable.plane", true);
  this->declare_parameter<bool>("enable.sphere", true);
  this->declare_parameter<bool>("enable.cylinder", true);

  // Register publisher of the primitive correlated with gaze
  rclcpp::QoS qos_object_of_interest = rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_default));
  pub_object_of_interest_ = this->create_publisher<geometric_primitive_msgs::msg::GeometricPrimitiveStamped>("object_of_interest", qos_object_of_interest);

  // Register publisher of visualisation markers
  if (publish_markers)
  {
    rclcpp::QoS qos_markers = rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_default));
    pub_markers_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("visualisation_markers", qos_markers);
  }

  RCLCPP_INFO(this->get_logger(), "Node initialised");
}

void GazeCorrelation::synchronized_callback(const gaze_msgs::msg::GazeStamped::SharedPtr msg_gaze,
                                            const geometric_primitive_msgs::msg::GeometricPrimitiveListStamped::SharedPtr msg_primitives)
{
  RCLCPP_DEBUG(this->get_logger(), "Received synchronized messages for processing");

  // Cleanup visualisation markers, if publishing is enabled
  if (this->get_parameter("publish_markers").get_value<bool>())
  {
    visualization_msgs::msg::MarkerArray cleanup;
    visualization_msgs::msg::Marker cleanup_marker;
    cleanup_marker.header = msg_primitives->header;
    cleanup_marker.action = visualization_msgs::msg::Marker::DELETEALL;
    cleanup.markers.push_back(cleanup_marker);
    pub_markers_->publish(cleanup);
  }

  // Create a parametrized line from the gaze
  Eigen::Vector3d origin, direction;
  Eigen::fromMsg(msg_gaze->gaze.eyeball_centre, origin);
  Eigen::fromMsg(msg_gaze->gaze.visual_axis, direction);
  Eigen::ParametrizedLine<double, 3> gaze(origin, direction);

  // Make sure the gaze and primitives are in the same coordinate system
  if (msg_gaze->header.frame_id != msg_primitives->header.frame_id)
  {
    // Attemp to find transformation between the coordinate frame of the gaze and primitives
    geometry_msgs::msg::TransformStamped transform_stamped;
    try
    {
      transform_stamped = tf2_buffer.lookupTransform(msg_primitives->header.frame_id, msg_gaze->header.frame_id, rclcpp::Time(0));
    }
    catch (tf2::TransformException &ex)
    {
      RCLCPP_ERROR(this->get_logger(), "%s", ex.what());
      return;
    }

    // Transform gaze into coordinate system of geometric primitives
    gaze = Eigen::transform(gaze, tf2::transformToEigen(transform_stamped.transform));
  }

  // List of found intersections in form of <double, <type, id>>
  std::vector<std::pair<double, std::pair<uint8_t, uint32_t>>> intersections;

  // Find intersections with planes, if desired
  if (this->get_parameter("enable.plane").get_value<bool>())
  {
    for (auto &plane : msg_primitives->primitives.planes)
    {
      // Convert to Eigen
      Eigen::Vector3d normal_vector(plane.coefficients[plane.COEFFICIENT_A],
                                    plane.coefficients[plane.COEFFICIENT_B],
                                    plane.coefficients[plane.COEFFICIENT_C]);

      // Determine intersection parameter
      double numerator = normal_vector.dot(gaze.origin()) + plane.coefficients[plane.COEFFICIENT_D];
      double denominator = normal_vector.dot(gaze.direction());
      if (denominator == 0.0)
      {
        if (numerator == 0.0)
        {
          RCLCPP_INFO(this->get_logger(), "No intersection with plane #%d - gaze is coincident with the plane", plane.id);
        }
        else
        {
          RCLCPP_INFO(this->get_logger(), "No intersection with plane #%d - gaze is parallel to the plane", plane.id);
        }
        continue;
      }
      double intersection_distance = -numerator / denominator;

      // Ignore intersections that occur in the negative gaze direction, i.e. behind the user
      if (intersection_distance <= 0.0)
      {
        RCLCPP_INFO(this->get_logger(), "Ignoring intersection with plane #%d - intersection occurs in negative gaze direction", plane.id);
        continue;
      }

      // Add the intersection to the list
      intersections.push_back(std::pair(intersection_distance, std::pair(GeometricPrimitive::PLANE, plane.id)));
    }
  }

  // Find intersections with spheres, if desired
  if (this->get_parameter("enable.sphere").get_value<bool>())
  {
    for (auto &sphere : msg_primitives->primitives.spheres)
    {
      // Convert to Eigen
      Eigen::Vector3d centre;
      Eigen::fromMsg(sphere.centre, centre);

      // Determine intersection parameter
      Eigen::Vector3d term_a = gaze.origin() - centre;
      double term_b = gaze.direction().dot(term_a);
      double value_under_sqrt = std::pow(term_b, 2) - (std::pow(term_a.norm(), 2) - std::pow(sphere.radius, 2));
      if (value_under_sqrt < 0.0)
      {
        RCLCPP_INFO(this->get_logger(), "No intersection with sphere #%d", sphere.id);
        continue;
      }
      double intersection_distance = -term_b - std::sqrt(value_under_sqrt);

      // Ignore intersections that occur in the negative gaze direction, i.e. behind the user
      if (intersection_distance <= 0.0)
      {
        RCLCPP_INFO(this->get_logger(), "Ignoring intersection with sphere #%d - intersection occurs in negative gaze direction", sphere.id);
        continue;
      }

      // Add the intersection to the list
      intersections.push_back(std::pair(intersection_distance, std::pair(GeometricPrimitive::SPHERE, sphere.id)));
    }
  }

  // Find intersections with cylinders, if desired
  if (this->get_parameter("enable.cylinder").get_value<bool>())
  {
    for (auto &cylinder : msg_primitives->primitives.cylinders)
    {
      // Convert to Eigen
      Eigen::Isometry3d pose;
      tf2::fromMsg(cylinder.pose, pose);

      // Transform the gaze to the coordinate system of the cyllinder
      Eigen::ParametrizedLine<double, 3> gaze_wrt_cyllinder = Eigen::transform(gaze, pose.inverse());

      // Convert to 2D (line direction should not be normalized!)
      Eigen::ParametrizedLine<double, 2> gaze_wrt_cyllinder_2d(gaze_wrt_cyllinder.origin().head<2>(), gaze_wrt_cyllinder.direction().head<2>());

      // Determine intersection parameter of the transformed and converted 2D gaze with a circle centred at origin with cylinder's radius
      double term_a = (2 * gaze_wrt_cyllinder_2d.origin()).dot(gaze_wrt_cyllinder_2d.direction());
      double term_b = std::pow(gaze_wrt_cyllinder_2d.direction().norm(), 2);
      double value_under_sqrt = std::pow(term_a, 2) - 4 * term_b * (std::pow(gaze_wrt_cyllinder_2d.origin().norm(), 2) - std::pow(cylinder.radius, 2));
      if (value_under_sqrt < 0.0)
      {
        RCLCPP_INFO(this->get_logger(), "No intersection with cylinder #%d - radius check", cylinder.id);
        continue;
      }
      double intersection_distance = (-term_a - std::sqrt(value_under_sqrt)) / (2 * term_b);

      // Make sure the intersection is within the limits of the cylinder's height
      double intersection_wrt_cylinder_z = gaze_wrt_cyllinder.origin().z() + intersection_distance * gaze_wrt_cyllinder.direction().z();
      if (std::abs(intersection_wrt_cylinder_z) > cylinder.height / 2)
      {
        RCLCPP_INFO(this->get_logger(), "No intersection with cylinder #%d - height check", cylinder.id);
        continue;
      }

      // Ignore intersections that occur in the negative gaze direction, i.e. behind the user
      if (intersection_distance <= 0.0)
      {
        RCLCPP_INFO(this->get_logger(), "Ignoring intersection with cylinder #%d - intersection occurs in negative gaze direction", cylinder.id);
        continue;
      }

      // Add the intersection to the list
      intersections.push_back(std::pair(intersection_distance, std::pair(GeometricPrimitive::CYLINDER, cylinder.id)));
    }
  }

  // Sort the intersections
  std::sort(intersections.begin(), intersections.end());

  // Make sure at least one intersction was found
  if (intersections.empty())
  {
    RCLCPP_WARN(this->get_logger(), "No intersection of gaze with surroundings was founds");
    return;
  }

  // Determine the appropriate intersection
  // TODO: Refine if needed
  auto intersection = intersections[0];

  // Find the 3D point of gaze
  Eigen::Vector3d point_of_gaze = gaze.pointAt(intersection.first);

  // Find the object of interest
  geometric_primitive_msgs::msg::GeometricPrimitiveStamped msg_object_of_interest;
  msg_object_of_interest.header = msg_primitives->header;
  msg_object_of_interest.primitive.type = intersection.second.first;
  switch (msg_object_of_interest.primitive.type)
  {
  case GeometricPrimitive::PLANE:
    for (auto &plane : msg_primitives->primitives.planes)
    {
      if (intersection.second.second == plane.id)
      {
        msg_object_of_interest.primitive.plane.push_back(plane);
        break;
      }
    }
    break;

  case GeometricPrimitive::SPHERE:
    for (auto &sphere : msg_primitives->primitives.spheres)
    {
      if (intersection.second.second == sphere.id)
      {
        msg_object_of_interest.primitive.sphere.push_back(sphere);
        break;
      }
    }
    break;

  case GeometricPrimitive::CYLINDER:
    for (auto &cylinder : msg_primitives->primitives.cylinders)
    {
      if (intersection.second.second == cylinder.id)
      {
        msg_object_of_interest.primitive.cylinder.push_back(cylinder);
        break;
      }
    }
    break;
  }

  // Publish the correlated object of interest
  pub_object_of_interest_->publish(msg_object_of_interest);

  if (this->get_parameter("publish_markers").get_value<bool>())
  {
    visualization_msgs::msg::MarkerArray markers;
    visualization_msgs::msg::Marker default_marker;
    default_marker.header = msg_primitives->header;
    default_marker.action = visualization_msgs::msg::Marker::ADD;

    // Gaze
    {
      visualization_msgs::msg::Marker gaze_marker = default_marker;
      gaze_marker.ns = std::string(this->get_namespace()) + "gaze";
      gaze_marker.id = 0;
      gaze_marker.type = visualization_msgs::msg::Marker::ARROW;
      geometry_msgs::msg::Point start, end;
      start = Eigen::toMsg(gaze.origin());
      end = Eigen::toMsg(gaze.pointAt(VISUAL_GAZE_LENGTH));
      gaze_marker.points.push_back(start);
      gaze_marker.points.push_back(end);
      gaze_marker.scale.x = VISUAL_GAZE_WIDTH;
      gaze_marker.scale.y =
          gaze_marker.scale.z = 0;
      gaze_marker.color.r = VISUAL_GAZE_COLOR[0];
      gaze_marker.color.g = VISUAL_GAZE_COLOR[1];
      gaze_marker.color.b = VISUAL_GAZE_COLOR[2];
      gaze_marker.color.a = VISUAL_GAZE_COLOR[3];
      markers.markers.push_back(gaze_marker);
    }

    // Point of gaze
    if (VISUALISE_GAZE)
    {
      visualization_msgs::msg::Marker point_of_gaze_marker = default_marker;
      point_of_gaze_marker.ns = std::string(this->get_namespace()) + "point_of_gaze";
      point_of_gaze_marker.id = 0;

      point_of_gaze_marker.type = visualization_msgs::msg::Marker::SPHERE;
      point_of_gaze_marker.pose.position = Eigen::toMsg(point_of_gaze);
      point_of_gaze_marker.scale.x =
          point_of_gaze_marker.scale.y =
              point_of_gaze_marker.scale.z = VISUAL_POINT_OF_GAZE_SCALE;
      point_of_gaze_marker.color.r = VISUAL_POINT_OF_GAZE_COLOR[0];
      point_of_gaze_marker.color.g = VISUAL_POINT_OF_GAZE_COLOR[1];
      point_of_gaze_marker.color.b = VISUAL_POINT_OF_GAZE_COLOR[2];
      point_of_gaze_marker.color.a = VISUAL_POINT_OF_GAZE_COLOR[3];
      markers.markers.push_back(point_of_gaze_marker);
    }

    // Object of interest
    {
      visualization_msgs::msg::Marker object_of_interest_marker = default_marker;
      object_of_interest_marker.ns = std::string(this->get_namespace()) + "object_of_interest";
      object_of_interest_marker.id = intersection.second.second;

      switch (msg_object_of_interest.primitive.type)
      {
      case GeometricPrimitive::PLANE:
      {
        object_of_interest_marker.type = visualization_msgs::msg::Marker::CUBE;

        Eigen::Vector3d normal_vector(msg_object_of_interest.primitive.plane[0].coefficients[0],
                                      msg_object_of_interest.primitive.plane[0].coefficients[1],
                                      msg_object_of_interest.primitive.plane[0].coefficients[2]);
        Eigen::Hyperplane<double, 3> hyperplane(normal_vector, msg_object_of_interest.primitive.plane[0].coefficients[3]);

        Eigen::Vector3d point_on_plane = hyperplane.projection(Eigen::Vector3d(0.0, 0.0, 0.0));
        object_of_interest_marker.pose.position.x = point_on_plane.x();
        object_of_interest_marker.pose.position.y = point_on_plane.y();
        object_of_interest_marker.pose.position.z = point_on_plane.z();

        Eigen::Quaternion<double> quat;
        quat.setFromTwoVectors(Eigen::Vector3d::UnitZ(), hyperplane.normal());
        object_of_interest_marker.pose.orientation.x = quat.x();
        object_of_interest_marker.pose.orientation.y = quat.y();
        object_of_interest_marker.pose.orientation.z = quat.z();
        object_of_interest_marker.pose.orientation.w = quat.w();

        object_of_interest_marker.scale.x = 2.0;
        object_of_interest_marker.scale.y = 2.0;
        object_of_interest_marker.scale.z = 0.001;
      }
      break;
      case GeometricPrimitive::SPHERE:
      {
        object_of_interest_marker.type = visualization_msgs::msg::Marker::SPHERE;

        object_of_interest_marker.pose.position = msg_object_of_interest.primitive.sphere[0].centre;
        object_of_interest_marker.scale.x =
            object_of_interest_marker.scale.y =
                object_of_interest_marker.scale.z = 2 * msg_object_of_interest.primitive.sphere[0].radius;
      }
      break;
      case GeometricPrimitive::CYLINDER:
      {
        object_of_interest_marker.type = visualization_msgs::msg::Marker::CYLINDER;

        object_of_interest_marker.pose = msg_object_of_interest.primitive.cylinder[0].pose;
        object_of_interest_marker.scale.x =
            object_of_interest_marker.scale.y = 2 * msg_object_of_interest.primitive.cylinder[0].radius;
        object_of_interest_marker.scale.z = msg_object_of_interest.primitive.cylinder[0].height;
      }
      break;
      }

      object_of_interest_marker.color.r = static_cast<float>(rand()) / static_cast<float>(RAND_MAX);
      object_of_interest_marker.color.g = static_cast<float>(rand()) / static_cast<float>(RAND_MAX);
      object_of_interest_marker.color.b = static_cast<float>(rand()) / static_cast<float>(RAND_MAX);
      object_of_interest_marker.color.a = 0.85;
      markers.markers.push_back(object_of_interest_marker);
    }

    pub_markers_->publish(markers);
  }
}

////////////
/// MAIN ///
////////////

/// Main function that initiates an object of `GazeCorrelation` class as the core of this node.
int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GazeCorrelation>());
  rclcpp::shutdown();
  return EXIT_SUCCESS;
}
