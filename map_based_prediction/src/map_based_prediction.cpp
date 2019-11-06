#include "map_based_prediction.h"

MapBasedPrediction::MapBasedPrediction()
{
  // pnh_.param<bool>("map_based_prediction/has_subscribed_map", has_subscribed_map_, false);
}


bool MapBasedPrediction::getNearestLane(const std::vector<LanePoint>& lane_points, 
                                             const geometry_msgs::Pose object_pose,
                                             LanePoint& nearest_lane_point)
{
  double dist = 1e+10;
  bool flag = false;
  for(const auto& lane_point : lane_points)
  {
    if(lane_point.is_in_intersection)
    {
      //sanity check for a point inside intersection
      double roll, pitch, object_yaw;
      toEulerAngle(object_pose.orientation, roll, pitch, object_yaw);
      const double lane_yaw = lane_point.rz;
      const double delta_yaw = std::acos(
        Eigen::Vector2d(std::cos(object_yaw), std::sin(object_yaw)).dot(
          Eigen::Vector2d(std::cos(lane_yaw), std::sin(lane_yaw))));
      if(delta_yaw > M_PI/2)
      {
        continue;
      }
      // isAllighWithLanePoint(object_yaw, lane_yaw)
    }
    double dx = object_pose.position.x - lane_point.tx;
    double dy = object_pose.position.y - lane_point.ty;
    double dr = (dx * dx) + (dy * dy);
    if((dr < dist) && (dr < lane_point.sr))
    {
        dist = dr;
        nearest_lane_point = lane_point;
        flag = true;
    }
  }
  return flag;
}

bool MapBasedPrediction::doPrediction(const autoware_msgs::DynamicObjectArray& in_objects,
                                      const std::vector<LanePoint>& lane_poitns,
                                      autoware_msgs::DynamicObjectArray& out_objects)
{
  // std::cerr << "frame start  ***************" << std::endl;
  // std::cerr << "num input object " << in_objects->objects.size()<< std::endl;
  out_objects = in_objects;
  for (auto& object: out_objects.objects)
  {
    if(object.semantic.type != autoware_msgs::Semantic::CAR &&
       object.semantic.type != autoware_msgs::Semantic::BUS)
    {
      continue;
    }
    
    const double abs_velo = std::sqrt(std::pow(object.state.twist.twist.linear.x, 2)+std::pow(object.state.twist.twist.linear.y,2));
    double minimum_velocity_threshold = 0.5;
    if(abs_velo < minimum_velocity_threshold)
    {
      continue;
    }
    // std::cerr << "id " << unique_id::toHexString(object.id) << std::endl;
    // std::cerr << "clsdd " << object.semantic.type << std::endl;
    // std::cerr << "velo x " << object.state.twist.twist.linear.x << std::endl;
    // std::cerr << "velo y " << object.state.twist.twist.linear.y << std::endl;
    
    geometry_msgs::Pose object_pose = object.state.pose.pose;
    LanePoint nearest_lane;
    if(getNearestLane(lane_poitns, object_pose, nearest_lane))
    {
      // calculate initial position in Frenet coordinate
      // Optimal Trajectory Generation for Dynamic Street Scenarios in a Frenet Frame
      // Path Planning for Highly Automated Driving on Embedded GPUs
      double px = object.state.pose.pose.position.x;
      double py = object.state.pose.pose.position.y;

      // std::cerr << "object 0000000" << std::endl;
      for(const auto& goal_path: nearest_lane.goal_paths)
      {
        const auto& origin_lane_point = goal_path.front();
        double lx = origin_lane_point.tx;
        double ly = origin_lane_point.ty;
        double px_lane_origin = px - lx;
        double py_lane_origin = py - ly;

        double distance = std::sqrt(std::pow(px_lane_origin,2) + std::pow(py_lane_origin,2));
        
        double atan_theta = std::atan2(py_lane_origin, px_lane_origin);
        const double phi = origin_lane_point.rz - atan_theta;
        double current_s_position = std::cos(phi) * distance;
        double current_d_position = std::sin(phi) * distance;
        
        double yaw;
        double object_velocity;
        if(object.state.pose_reliable)
        {
          double roll, pitch;
          toEulerAngle(object.state.pose.pose.orientation, roll, pitch, yaw);
          object_velocity = object.state.twist.twist.linear.x;
        }
        else
        {
          double theta = std::atan2(object.state.pose.pose.position.y,
                           object.state.pose.pose.position.x);
          // East is 0(rad) according to REP
          // since x and y is inverse in map coordinate, offset -M_PI/2
          yaw = theta + M_PI/2;
          // normalize angle (0, 2*M_PI)
          yaw = std::atan2(std::sin(yaw), std::cos(yaw)) + M_PI;
          // if(yaw < - M_PI)
          // {
          //   yaw += 2*M_PI;
          // }
          object_velocity = abs_velo;
        }
        double theta_r = origin_lane_point.rz;
        double delta_theta = yaw - theta_r;
        double current_d_velocity = object_velocity* std::sin(delta_theta);
        double current_s_velocity = object_velocity* std::cos(delta_theta);
        double target_s_position = goal_path.back().cumulated_s;

        autoware_msgs::PredictedPath path;

        // for ego point
        const double height = object_pose.position.z;
        geometry_msgs::PoseWithCovarianceStamped point;
        point.pose.pose.position.x = px;
        point.pose.pose.position.y = py;
        point.pose.pose.position.z = height;
        path.path.push_back(point);
        
        // std::vector<geometry_msgs::Point> tmp_path;
        getPredictedPath(goal_path,
                        yaw,
                        height,
                        current_d_position,
                        current_d_velocity,
                        current_s_position,
                        current_s_velocity,
                        target_s_position,
                        path);
    
        object.state.predicted_paths.push_back(path);
      }
      normalizeLikelyhood(object.state.predicted_paths);
    }
    else
    {
      // std::cerr << "Could not find nearest lane" << std::endl;
      continue;
    }
  }
  
  return true;
}

bool MapBasedPrediction::normalizeLikelyhood(std::vector<autoware_msgs::PredictedPath>& paths)
{
  // TODO: is could not be the smartest way
  double sum_confidence = 0;
  for (const auto& path: paths)
  {
    sum_confidence += 1/path.confidence;
  }
  
  for (auto& path: paths)
  {
    path.confidence =  (1/path.confidence)/sum_confidence;
  }
}

bool MapBasedPrediction::getPredictedPath(
  const std::vector<LanePoint>& goal_path,
  const double current_yaw,
  const double current_height,
  const double current_d_position,
  const double current_d_velocity,
  const double current_s_position,
  const double current_s_velocity,
  const double target_s_position,
  autoware_msgs::PredictedPath& path)
{
  //Quintic polynominal for d
  // A = np.array([[T**3, T**4, T**5],
  //               [3 * T ** 2, 4 * T ** 3, 5 * T ** 4],
  //               [6 * T, 12 * T ** 2, 20 * T ** 3]])
  // A_inv = np.matrix([[10/(T**3), -4/(T**2), 1/(2*T)],
  //                    [-15/(T**4), 7/(T**3), -1/(T**2)],
  //                    [6/(T**5), -3/(T**4),  1/(2*T**3)]])
  // b = np.matrix([[xe - self.a0 - self.a1 * T - self.a2 * T**2],
  //                [vxe - self.a1 - 2 * self.a2 * T],
  //                [axe - 2 * self.a2]])
  double target_d_position = 0;
  
  double t = 5.0;
  Eigen::Matrix3d a_3_inv;
  a_3_inv << 10/std::pow(t,3), -4/std::pow(t,2), 1/(2*t),
          -15/std::pow(t,4), 7/std::pow(t,3), -1/std::pow(t,2),
           6/std::pow(t,5), -3/std::pow(t,4), 1/(2*std::pow(t,3));
           
  double target_d_velocity = current_d_velocity;
  double target_d_accerelation = 0;
  Eigen::Vector3d b_3;
  b_3 << target_d_position - current_d_position - current_d_velocity * t,
         target_d_velocity - current_d_velocity,
         target_d_accerelation;
         
  Eigen::Vector3d x_3;
  x_3 = a_3_inv * b_3;
  
         
  // Quatric polynominal
  // A_inv = np.matrix([[1/(T**2), -1/(3*T)],
  //                         [-1/(2*T**3), 1/(4*T**2)]])
  // b = np.matrix([[vxe - self.a1 - 2 * self.a2 * T],
  //               [axe - 2 * self.a2]])
  Eigen::Matrix2d a_2_inv;
  a_2_inv << 1/std::pow(t,2), -1/(3*t),
             -1/(2*std::pow(t,3)), 1/(4*std::pow(t,2));
  double target_s_velocity = current_s_velocity;
  Eigen::Vector2d b_2;
  b_2 << target_s_velocity - current_s_velocity,
         0;
  Eigen::Vector2d x_2;
  x_2 = a_2_inv*b_2;
  
  // sampling points from calculated path
  double dt = 0.5;
  std::vector<double> d_vec;
  double calculated_d, calculated_s;
  // std::cerr << "-------"  << std::endl;
  // std::cerr << "current d " << current_d_position << " current s " << current_s_position << std::endl;
  for(double i = dt; i < t; i+=dt)
  {
    calculated_d = current_d_position + 
                          current_d_velocity * i +
                          0*2*i*i + 
                          x_3(0) * i*i*i+
                          x_3(1)*i*i*i*i+
                          x_3(2)*i*i*i*i*i;
    calculated_s = current_s_position + 
                          current_s_velocity*i + 
                          2*0*i*i +
                          x_2(0) * i*i*i +
                          x_2(1) *i*i*i*i;
                          
    // geometry_msgs::Pose pose;
    // std::cerr << "d " << calculated_d << " s " << calculated_s << std::endl;
    
    geometry_msgs::PoseWithCovarianceStamped tmp_point;
    calculateCartesianPositionUsingLanePoints(goal_path, calculated_s, calculated_d,tmp_point);
    tmp_point.pose.pose.position.z = current_height;
    path.path.push_back(tmp_point);
  }
  const double desired_yaw = goal_path.front().rz;
  path.confidence = calculateLikelyhood(desired_yaw, current_d_position, current_yaw);
  // std::cerr << "desired yaw " << desired_yaw << std::endl;
  // std::cerr << "current d " << current_d_position << std::endl;
  // std::cerr << "current yaw " << current_yaw << std::endl;
  // std::cerr << "path cpnfidence " << path.confidence << std::endl;
  // std::cerr << "-------" << path.confidence << std::endl;
  return false;
}

bool MapBasedPrediction::calculateCartesianPositionUsingLanePoints(const std::vector<LanePoint>& goal_path, 
                           const double target_s,
                           const double target_d,
                           geometry_msgs::PoseWithCovarianceStamped& converted_position)
{
  // std::cerr << "-------" << std::endl;
  double min_abs_delta = 100000;
  double cache_delta, cache_yaw;
  geometry_msgs::Point cache_cumulated_s_position;
  for (const auto& point: goal_path)
  {
    // double delta_s = point.cumulated_s - target_s;
    double delta_s = target_s - point.cumulated_s;
    if(std::abs(delta_s) < min_abs_delta)
    {
      min_abs_delta = std::abs(delta_s);
      cache_delta = delta_s;
      cache_yaw = point.rz;
      cache_cumulated_s_position.x = point.tx;
      cache_cumulated_s_position.y = point.ty;
    }
  }
  convertFrenetPosition2CartesianPosition(target_s,
                                          target_d,
                                          cache_cumulated_s_position,
                                          cache_delta,
                                          cache_yaw,
                                          converted_position);

  return true;
}

bool MapBasedPrediction::convertFrenetPosition2CartesianPosition(
                           const double target_s,
                           const double target_d,
                           const geometry_msgs::Point& cumulated_s_position,
                           const double delta_s,
                           const double delta_yaw,
                           geometry_msgs::PoseWithCovarianceStamped& converted_position)
{
  converted_position.pose.pose.position.x = cumulated_s_position.x + delta_s*std::cos(delta_yaw);
  converted_position.pose.pose.position.y = cumulated_s_position.y + delta_s*std::sin(delta_yaw);
  // TODO: this might be naive implemetation: need to be revisited
  converted_position.pose.pose.position.x += target_d*std::cos(delta_yaw-M_PI/2);
  converted_position.pose.pose.position.y += target_d*std::sin(delta_yaw-M_PI/2);

  return true;
  // std::cerr <<"cumulated s "<< point.cumulated_s << std::endl;
}

double MapBasedPrediction::calculateLikelyhood(const double desired_yaw, 
                                             const double current_d, 
                                             const double current_yaw)
{
  Eigen::Vector2d current_state;
  current_state << current_d, current_yaw;
  Eigen::Vector2d desired_state;
  const double desired_d = 0;
  desired_state << desired_d, desired_yaw;
  Eigen::Matrix2d covariance;
  covariance << 0.5 , 0,
                  0 , 0.5;
  // calculate Mahalanobis' Distance
  double likelyhood = (current_state - desired_state).transpose() *
                       covariance.inverse() * 
                      (current_state - desired_state);
  return likelyhood;
}

//from https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles
void MapBasedPrediction::toEulerAngle(const geometry_msgs::Quaternion& q, 
                                      double& roll, double& pitch, double& yaw)
{
  // roll (x-axis rotation)
  double sinr_cosp = +2.0 * (q.w * q.x + q.y * q.z);
  double cosr_cosp = +1.0 - 2.0 * (q.x * q.x + q.y * q.y);
  roll = std::atan2(sinr_cosp, cosr_cosp);

  // pitch (y-axis rotation)
  double sinp = +2.0 * (q.w * q.y - q.z * q.x);
  if (std::fabs(sinp) >= 1)
    pitch = std::copysign(M_PI / 2, sinp); // use 90 degrees if out of range
  else
    pitch = std::asin(sinp);

  // yaw (z-axis rotation)
  double siny_cosp = +2.0 * (q.w * q.z + q.x * q.y);
  double cosy_cosp = +1.0 - 2.0 * (q.y * q.y + q.z * q.z);  
  yaw = std::atan2(siny_cosp, cosy_cosp);
}