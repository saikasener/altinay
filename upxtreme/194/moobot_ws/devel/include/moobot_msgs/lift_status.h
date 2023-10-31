// Generated by gencpp from file moobot_msgs/lift_status.msg
// DO NOT EDIT!


#ifndef MOOBOT_MSGS_MESSAGE_LIFT_STATUS_H
#define MOOBOT_MSGS_MESSAGE_LIFT_STATUS_H


#include <string>
#include <vector>
#include <memory>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace moobot_msgs
{
template <class ContainerAllocator>
struct lift_status_
{
  typedef lift_status_<ContainerAllocator> Type;

  lift_status_()
    : up_lift(false)
    , down_lift(false)  {
    }
  lift_status_(const ContainerAllocator& _alloc)
    : up_lift(false)
    , down_lift(false)  {
  (void)_alloc;
    }



   typedef uint8_t _up_lift_type;
  _up_lift_type up_lift;

   typedef uint8_t _down_lift_type;
  _down_lift_type down_lift;





  typedef boost::shared_ptr< ::moobot_msgs::lift_status_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::moobot_msgs::lift_status_<ContainerAllocator> const> ConstPtr;

}; // struct lift_status_

typedef ::moobot_msgs::lift_status_<std::allocator<void> > lift_status;

typedef boost::shared_ptr< ::moobot_msgs::lift_status > lift_statusPtr;
typedef boost::shared_ptr< ::moobot_msgs::lift_status const> lift_statusConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::moobot_msgs::lift_status_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::moobot_msgs::lift_status_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::moobot_msgs::lift_status_<ContainerAllocator1> & lhs, const ::moobot_msgs::lift_status_<ContainerAllocator2> & rhs)
{
  return lhs.up_lift == rhs.up_lift &&
    lhs.down_lift == rhs.down_lift;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::moobot_msgs::lift_status_<ContainerAllocator1> & lhs, const ::moobot_msgs::lift_status_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace moobot_msgs

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::moobot_msgs::lift_status_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::moobot_msgs::lift_status_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::moobot_msgs::lift_status_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::moobot_msgs::lift_status_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::moobot_msgs::lift_status_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::moobot_msgs::lift_status_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::moobot_msgs::lift_status_<ContainerAllocator> >
{
  static const char* value()
  {
    return "88d1b4abfd45857169d18e9ca11502ca";
  }

  static const char* value(const ::moobot_msgs::lift_status_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x88d1b4abfd458571ULL;
  static const uint64_t static_value2 = 0x69d18e9ca11502caULL;
};

template<class ContainerAllocator>
struct DataType< ::moobot_msgs::lift_status_<ContainerAllocator> >
{
  static const char* value()
  {
    return "moobot_msgs/lift_status";
  }

  static const char* value(const ::moobot_msgs::lift_status_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::moobot_msgs::lift_status_<ContainerAllocator> >
{
  static const char* value()
  {
    return "bool up_lift\n"
"bool down_lift\n"
;
  }

  static const char* value(const ::moobot_msgs::lift_status_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::moobot_msgs::lift_status_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.up_lift);
      stream.next(m.down_lift);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct lift_status_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::moobot_msgs::lift_status_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::moobot_msgs::lift_status_<ContainerAllocator>& v)
  {
    s << indent << "up_lift: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.up_lift);
    s << indent << "down_lift: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.down_lift);
  }
};

} // namespace message_operations
} // namespace ros

#endif // MOOBOT_MSGS_MESSAGE_LIFT_STATUS_H