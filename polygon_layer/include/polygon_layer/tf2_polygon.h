#ifndef TF2_POLYGON_H
#define TF2_POLYGON_H

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

namespace tf2
{

/********************/
/** PolygonStamped **/
/********************/

template<>
inline
const ros::Time &getTimestamp(const geometry_msgs::PolygonStamped &t) { return t.header.stamp; }

template<>
inline
const std::string &getFrameId(const geometry_msgs::PolygonStamped &t) { return t.header.frame_id; }

// inline
// geometry_msgs::PolygonStamped toMsg(const geometry_msgs::PolygonStamped &in)
// {
//   return in;
// }
//
// inline
// void fromMsg(const geometry_msgs::PolygonStamped &msg, geometry_msgs::PolygonStamped &out)
// {
//   out = msg;
// }

template<>
inline
void convert(const tf2::Vector3 &from, geometry_msgs::Point32 &to)
{
  to.x = from.m_floats[0];
  to.y = from.m_floats[1];
  to.z = from.m_floats[2];
}

template<>
inline
void convert(const geometry_msgs::Point32 &from, tf2::Vector3 &to)
{
  to.m_floats[0] = from.x;
  to.m_floats[1] = from.y;
  to.m_floats[2] = from.z;
}

template<>
inline
void doTransform(const geometry_msgs::PolygonStamped &in, geometry_msgs::PolygonStamped &out,
                 const geometry_msgs::TransformStamped &transform)
{
  out.polygon.points.resize(in.polygon.points.size());

  tf2::Transform t;
  convert(transform.transform, t);

  for (std::size_t i = 0; i < in.polygon.points.size(); i++)
  {
    tf2::Vector3 v_in;
    convert(in.polygon.points[i], v_in);
    tf2::Vector3 v_out = t * v_in;
    convert(v_out, out.polygon.points[i]);
  }

  out.header = in.header;
  out.header.frame_id = transform.header.frame_id;
}

} // namespace

#endif // TF2_POLYGON_H
