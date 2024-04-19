// Generated by gencpp from file assignment_2_2023/RobotState.msg
// DO NOT EDIT!


#ifndef ASSIGNMENT_2_2023_MESSAGE_ROBOTSTATE_H
#define ASSIGNMENT_2_2023_MESSAGE_ROBOTSTATE_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace assignment_2_2023
{
template <class ContainerAllocator>
struct RobotState_
{
  typedef RobotState_<ContainerAllocator> Type;

  RobotState_()
    : x(0.0)
    , y(0.0)
    , vel_x(0.0)
    , vel_z(0.0)  {
    }
  RobotState_(const ContainerAllocator& _alloc)
    : x(0.0)
    , y(0.0)
    , vel_x(0.0)
    , vel_z(0.0)  {
  (void)_alloc;
    }



   typedef double _x_type;
  _x_type x;

   typedef double _y_type;
  _y_type y;

   typedef double _vel_x_type;
  _vel_x_type vel_x;

   typedef double _vel_z_type;
  _vel_z_type vel_z;





  typedef boost::shared_ptr< ::assignment_2_2023::RobotState_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::assignment_2_2023::RobotState_<ContainerAllocator> const> ConstPtr;

}; // struct RobotState_

typedef ::assignment_2_2023::RobotState_<std::allocator<void> > RobotState;

typedef boost::shared_ptr< ::assignment_2_2023::RobotState > RobotStatePtr;
typedef boost::shared_ptr< ::assignment_2_2023::RobotState const> RobotStateConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::assignment_2_2023::RobotState_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::assignment_2_2023::RobotState_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::assignment_2_2023::RobotState_<ContainerAllocator1> & lhs, const ::assignment_2_2023::RobotState_<ContainerAllocator2> & rhs)
{
  return lhs.x == rhs.x &&
    lhs.y == rhs.y &&
    lhs.vel_x == rhs.vel_x &&
    lhs.vel_z == rhs.vel_z;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::assignment_2_2023::RobotState_<ContainerAllocator1> & lhs, const ::assignment_2_2023::RobotState_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace assignment_2_2023

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::assignment_2_2023::RobotState_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::assignment_2_2023::RobotState_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::assignment_2_2023::RobotState_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::assignment_2_2023::RobotState_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::assignment_2_2023::RobotState_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::assignment_2_2023::RobotState_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::assignment_2_2023::RobotState_<ContainerAllocator> >
{
  static const char* value()
  {
    return "9a6b8857bb44e9dfbb8aa9b340027ecc";
  }

  static const char* value(const ::assignment_2_2023::RobotState_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x9a6b8857bb44e9dfULL;
  static const uint64_t static_value2 = 0xbb8aa9b340027eccULL;
};

template<class ContainerAllocator>
struct DataType< ::assignment_2_2023::RobotState_<ContainerAllocator> >
{
  static const char* value()
  {
    return "assignment_2_2023/RobotState";
  }

  static const char* value(const ::assignment_2_2023::RobotState_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::assignment_2_2023::RobotState_<ContainerAllocator> >
{
  static const char* value()
  {
    return "float64 x\n"
"float64 y\n"
"float64 vel_x\n"
"float64 vel_z\n"
"\n"
;
  }

  static const char* value(const ::assignment_2_2023::RobotState_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::assignment_2_2023::RobotState_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.x);
      stream.next(m.y);
      stream.next(m.vel_x);
      stream.next(m.vel_z);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct RobotState_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::assignment_2_2023::RobotState_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::assignment_2_2023::RobotState_<ContainerAllocator>& v)
  {
    s << indent << "x: ";
    Printer<double>::stream(s, indent + "  ", v.x);
    s << indent << "y: ";
    Printer<double>::stream(s, indent + "  ", v.y);
    s << indent << "vel_x: ";
    Printer<double>::stream(s, indent + "  ", v.vel_x);
    s << indent << "vel_z: ";
    Printer<double>::stream(s, indent + "  ", v.vel_z);
  }
};

} // namespace message_operations
} // namespace ros

#endif // ASSIGNMENT_2_2023_MESSAGE_ROBOTSTATE_H