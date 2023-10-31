// Generated by gencpp from file moobot_msgs/conveyor_status.msg
// DO NOT EDIT!


#ifndef MOOBOT_MSGS_MESSAGE_CONVEYOR_STATUS_H
#define MOOBOT_MSGS_MESSAGE_CONVEYOR_STATUS_H


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
struct conveyor_status_
{
  typedef conveyor_status_<ContainerAllocator> Type;

  conveyor_status_()
    : load_conveyor(false)
    , unload_conveyor(false)  {
    }
  conveyor_status_(const ContainerAllocator& _alloc)
    : load_conveyor(false)
    , unload_conveyor(false)  {
  (void)_alloc;
    }



   typedef uint8_t _load_conveyor_type;
  _load_conveyor_type load_conveyor;

   typedef uint8_t _unload_conveyor_type;
  _unload_conveyor_type unload_conveyor;





  typedef boost::shared_ptr< ::moobot_msgs::conveyor_status_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::moobot_msgs::conveyor_status_<ContainerAllocator> const> ConstPtr;

}; // struct conveyor_status_

typedef ::moobot_msgs::conveyor_status_<std::allocator<void> > conveyor_status;

typedef boost::shared_ptr< ::moobot_msgs::conveyor_status > conveyor_statusPtr;
typedef boost::shared_ptr< ::moobot_msgs::conveyor_status const> conveyor_statusConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::moobot_msgs::conveyor_status_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::moobot_msgs::conveyor_status_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::moobot_msgs::conveyor_status_<ContainerAllocator1> & lhs, const ::moobot_msgs::conveyor_status_<ContainerAllocator2> & rhs)
{
  return lhs.load_conveyor == rhs.load_conveyor &&
    lhs.unload_conveyor == rhs.unload_conveyor;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::moobot_msgs::conveyor_status_<ContainerAllocator1> & lhs, const ::moobot_msgs::conveyor_status_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace moobot_msgs

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::moobot_msgs::conveyor_status_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::moobot_msgs::conveyor_status_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::moobot_msgs::conveyor_status_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::moobot_msgs::conveyor_status_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::moobot_msgs::conveyor_status_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::moobot_msgs::conveyor_status_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::moobot_msgs::conveyor_status_<ContainerAllocator> >
{
  static const char* value()
  {
    return "a8462790025e5d19ec5df8dbb71cb071";
  }

  static const char* value(const ::moobot_msgs::conveyor_status_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xa8462790025e5d19ULL;
  static const uint64_t static_value2 = 0xec5df8dbb71cb071ULL;
};

template<class ContainerAllocator>
struct DataType< ::moobot_msgs::conveyor_status_<ContainerAllocator> >
{
  static const char* value()
  {
    return "moobot_msgs/conveyor_status";
  }

  static const char* value(const ::moobot_msgs::conveyor_status_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::moobot_msgs::conveyor_status_<ContainerAllocator> >
{
  static const char* value()
  {
    return "bool load_conveyor\n"
"bool unload_conveyor\n"
;
  }

  static const char* value(const ::moobot_msgs::conveyor_status_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::moobot_msgs::conveyor_status_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.load_conveyor);
      stream.next(m.unload_conveyor);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct conveyor_status_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::moobot_msgs::conveyor_status_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::moobot_msgs::conveyor_status_<ContainerAllocator>& v)
  {
    s << indent << "load_conveyor: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.load_conveyor);
    s << indent << "unload_conveyor: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.unload_conveyor);
  }
};

} // namespace message_operations
} // namespace ros

#endif // MOOBOT_MSGS_MESSAGE_CONVEYOR_STATUS_H