#ifndef TF2_POLYGON_H
#define TF2_POLYGON_H

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

namespace tf2
{

/******************/
/** PolygonStamped **/
/******************/

// method to extract timestamp from object
template<>
inline
const ros::Time &getTimestamp(const geometry_msgs::PolygonStamped &t) { return t.header.stamp; }

// method to extract frame id from object
template<>
inline
const std::string &getFrameId(const geometry_msgs::PolygonStamped &t) { return t.header.frame_id; }

// this method needs to be implemented by client library developers
template<>
inline
void doTransform(const geometry_msgs::PolygonStamped &t_in, geometry_msgs::PolygonStamped &t_out,
                 const geometry_msgs::TransformStamped &transform)
{
  KDL::Frame frame = gmTransformToKDL(transform);
  t_out.polygon.points.clear();
  geometry_msgs::Point32 p_out;

  for (unsigned int i = 0; i < t_in.polygon.points.size(); i++)
  {
    KDL::Vector v_out = gmTransformToKDL(transform) *
                        KDL::Vector(t_in.polygon.points[i].x, t_in.polygon.points[i].y, t_in.polygon.points[i].z);
    p_out.x = v_out[0];
    p_out.y = v_out[1];
    p_out.z = v_out[2];
    t_out.polygon.points.push_back(p_out);
  }

  t_out.header = t_in.header;
  t_out.header.frame_id = transform.header.frame_id;
}

inline
geometry_msgs::PolygonStamped toMsg(const geometry_msgs::PolygonStamped &in)
{
  return in;
}

inline
void fromMsg(const geometry_msgs::PolygonStamped &msg, geometry_msgs::PolygonStamped &out)
{
  out = msg;
}


} // namespace

#endif // TF2_POLYGON_H
