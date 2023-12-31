// Generated by gencpp from file moobot_msgs/moobot_sensor_status.msg
// DO NOT EDIT!


#ifndef MOOBOT_MSGS_MESSAGE_MOOBOT_SENSOR_STATUS_H
#define MOOBOT_MSGS_MESSAGE_MOOBOT_SENSOR_STATUS_H


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
struct moobot_sensor_status_
{
  typedef moobot_sensor_status_<ContainerAllocator> Type;

  moobot_sensor_status_()
    : state()
    , state_type(0)  {
    }
  moobot_sensor_status_(const ContainerAllocator& _alloc)
    : state(_alloc)
    , state_type(0)  {
  (void)_alloc;
    }



   typedef std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> _state_type;
  _state_type state;

   typedef int16_t _state_type_type;
  _state_type_type state_type;





  typedef boost::shared_ptr< ::moobot_msgs::moobot_sensor_status_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::moobot_msgs::moobot_sensor_status_<ContainerAllocator> const> ConstPtr;

}; // struct moobot_sensor_status_

typedef ::moobot_msgs::moobot_sensor_status_<std::allocator<void> > moobot_sensor_status;

typedef boost::shared_ptr< ::moobot_msgs::moobot_sensor_status > moobot_sensor_statusPtr;
typedef boost::shared_ptr< ::moobot_msgs::moobot_sensor_status const> moobot_sensor_statusConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::moobot_msgs::moobot_sensor_status_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::moobot_msgs::moobot_sensor_status_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::moobot_msgs::moobot_sensor_status_<ContainerAllocator1> & lhs, const ::moobot_msgs::moobot_sensor_status_<ContainerAllocator2> & rhs)
{
  return lhs.state == rhs.state &&
    lhs.state_type == rhs.state_type;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::moobot_msgs::moobot_sensor_status_<ContainerAllocator1> & lhs, const ::moobot_msgs::moobot_sensor_status_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace moobot_msgs

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::moobot_msgs::moobot_sensor_status_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::moobot_msgs::moobot_sensor_status_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::moobot_msgs::moobot_sensor_status_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::moobot_msgs::moobot_sensor_status_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::moobot_msgs::moobot_sensor_status_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::moobot_msgs::moobot_sensor_status_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::moobot_msgs::moobot_sensor_status_<ContainerAllocator> >
{
  static const char* value()
  {
    return "49c3408f94118958df8cb878e67805f5";
  }

  static const char* value(const ::moobot_msgs::moobot_sensor_status_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x49c3408f94118958ULL;
  static const uint64_t static_value2 = 0xdf8cb878e67805f5ULL;
};

template<class ContainerAllocator>
struct DataType< ::moobot_msgs::moobot_sensor_status_<ContainerAllocator> >
{
  static const char* value()
  {
    return "moobot_msgs/moobot_sensor_status";
  }

  static const char* value(const ::moobot_msgs::moobot_sensor_status_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::moobot_msgs::moobot_sensor_status_<ContainerAllocator> >
{
  static const char* value()
  {
    return "string state\n"
"int16 state_type\n"
;
  }

  static const char* value(const ::moobot_msgs::moobot_sensor_status_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::moobot_msgs::moobot_sensor_status_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.state);
      stream.next(m.state_type);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct moobot_sensor_status_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::moobot_msgs::moobot_sensor_status_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::moobot_msgs::moobot_sensor_status_<ContainerAllocator>& v)
  {
    s << indent << "state: ";
    Printer<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>>::stream(s, indent + "  ", v.state);
    s << indent << "state_type: ";
    Printer<int16_t>::stream(s, indent + "  ", v.state_type);
  }
};

} // namespace message_operations
} // namespace ros

#endif // MOOBOT_MSGS_MESSAGE_MOOBOT_SENSOR_STATUS_H
