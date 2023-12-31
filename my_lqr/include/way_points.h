// Generated by gencpp from file my_lqr/way_points.msg
// DO NOT EDIT!


#ifndef MY_LQR_MESSAGE_WAY_POINTS_H
#define MY_LQR_MESSAGE_WAY_POINTS_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <way_point.h>

namespace my_lqr
{
template <class ContainerAllocator>
struct way_points_
{
  typedef way_points_<ContainerAllocator> Type;

  way_points_()
    : way_points()  {
    }
  way_points_(const ContainerAllocator& _alloc)
    : way_points(_alloc)  {
  (void)_alloc;
    }



   typedef std::vector< ::my_lqr::way_point_<ContainerAllocator> , typename ContainerAllocator::template rebind< ::my_lqr::way_point_<ContainerAllocator> >::other >  _way_points_type;
  _way_points_type way_points;





  typedef boost::shared_ptr< ::my_lqr::way_points_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::my_lqr::way_points_<ContainerAllocator> const> ConstPtr;

}; // struct way_points_

typedef ::my_lqr::way_points_<std::allocator<void> > way_points;

typedef boost::shared_ptr< ::my_lqr::way_points > way_pointsPtr;
typedef boost::shared_ptr< ::my_lqr::way_points const> way_pointsConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::my_lqr::way_points_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::my_lqr::way_points_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::my_lqr::way_points_<ContainerAllocator1> & lhs, const ::my_lqr::way_points_<ContainerAllocator2> & rhs)
{
  return lhs.way_points == rhs.way_points;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::my_lqr::way_points_<ContainerAllocator1> & lhs, const ::my_lqr::way_points_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace my_lqr

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsFixedSize< ::my_lqr::way_points_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::my_lqr::way_points_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct IsMessage< ::my_lqr::way_points_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::my_lqr::way_points_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::my_lqr::way_points_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::my_lqr::way_points_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::my_lqr::way_points_<ContainerAllocator> >
{
  static const char* value()
  {
    return "0f2db0724ffea498c07e0fe78cb38333";
  }

  static const char* value(const ::my_lqr::way_points_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x0f2db0724ffea498ULL;
  static const uint64_t static_value2 = 0xc07e0fe78cb38333ULL;
};

template<class ContainerAllocator>
struct DataType< ::my_lqr::way_points_<ContainerAllocator> >
{
  static const char* value()
  {
    return "my_lqr/way_points";
  }

  static const char* value(const ::my_lqr::way_points_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::my_lqr::way_points_<ContainerAllocator> >
{
  static const char* value()
  {
    return "way_point[] way_points\n"
"================================================================================\n"
"MSG: my_lqr/way_point\n"
"float32 x\n"
"float32 y\n"
"float32 yaw\n"
"float32 delta\n"
;
  }

  static const char* value(const ::my_lqr::way_points_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::my_lqr::way_points_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.way_points);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct way_points_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::my_lqr::way_points_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::my_lqr::way_points_<ContainerAllocator>& v)
  {
    s << indent << "way_points[]" << std::endl;
    for (size_t i = 0; i < v.way_points.size(); ++i)
    {
      s << indent << "  way_points[" << i << "]: ";
      s << std::endl;
      s << indent;
      Printer< ::my_lqr::way_point_<ContainerAllocator> >::stream(s, indent + "    ", v.way_points[i]);
    }
  }
};

} // namespace message_operations
} // namespace ros

#endif // MY_LQR_MESSAGE_WAY_POINTS_H
