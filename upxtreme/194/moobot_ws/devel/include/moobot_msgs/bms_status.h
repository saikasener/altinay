// Generated by gencpp from file moobot_msgs/bms_status.msg
// DO NOT EDIT!


#ifndef MOOBOT_MSGS_MESSAGE_BMS_STATUS_H
#define MOOBOT_MSGS_MESSAGE_BMS_STATUS_H


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
struct bms_status_
{
  typedef bms_status_<ContainerAllocator> Type;

  bms_status_()
    : battery_volt_1(0.0)
    , battery_volt_2(0.0)
    , battery_volt_total(0.0)
    , battery_temp_1(0.0)
    , battery_temp_2(0.0)
    , current_main(0.0)
    , total_power_wh(0.0)  {
    }
  bms_status_(const ContainerAllocator& _alloc)
    : battery_volt_1(0.0)
    , battery_volt_2(0.0)
    , battery_volt_total(0.0)
    , battery_temp_1(0.0)
    , battery_temp_2(0.0)
    , current_main(0.0)
    , total_power_wh(0.0)  {
  (void)_alloc;
    }



   typedef float _battery_volt_1_type;
  _battery_volt_1_type battery_volt_1;

   typedef float _battery_volt_2_type;
  _battery_volt_2_type battery_volt_2;

   typedef float _battery_volt_total_type;
  _battery_volt_total_type battery_volt_total;

   typedef float _battery_temp_1_type;
  _battery_temp_1_type battery_temp_1;

   typedef float _battery_temp_2_type;
  _battery_temp_2_type battery_temp_2;

   typedef float _current_main_type;
  _current_main_type current_main;

   typedef float _total_power_wh_type;
  _total_power_wh_type total_power_wh;





  typedef boost::shared_ptr< ::moobot_msgs::bms_status_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::moobot_msgs::bms_status_<ContainerAllocator> const> ConstPtr;

}; // struct bms_status_

typedef ::moobot_msgs::bms_status_<std::allocator<void> > bms_status;

typedef boost::shared_ptr< ::moobot_msgs::bms_status > bms_statusPtr;
typedef boost::shared_ptr< ::moobot_msgs::bms_status const> bms_statusConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::moobot_msgs::bms_status_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::moobot_msgs::bms_status_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::moobot_msgs::bms_status_<ContainerAllocator1> & lhs, const ::moobot_msgs::bms_status_<ContainerAllocator2> & rhs)
{
  return lhs.battery_volt_1 == rhs.battery_volt_1 &&
    lhs.battery_volt_2 == rhs.battery_volt_2 &&
    lhs.battery_volt_total == rhs.battery_volt_total &&
    lhs.battery_temp_1 == rhs.battery_temp_1 &&
    lhs.battery_temp_2 == rhs.battery_temp_2 &&
    lhs.current_main == rhs.current_main &&
    lhs.total_power_wh == rhs.total_power_wh;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::moobot_msgs::bms_status_<ContainerAllocator1> & lhs, const ::moobot_msgs::bms_status_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace moobot_msgs

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::moobot_msgs::bms_status_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::moobot_msgs::bms_status_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::moobot_msgs::bms_status_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::moobot_msgs::bms_status_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::moobot_msgs::bms_status_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::moobot_msgs::bms_status_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::moobot_msgs::bms_status_<ContainerAllocator> >
{
  static const char* value()
  {
    return "3113256c57eafd1210064dbad908f61d";
  }

  static const char* value(const ::moobot_msgs::bms_status_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x3113256c57eafd12ULL;
  static const uint64_t static_value2 = 0x10064dbad908f61dULL;
};

template<class ContainerAllocator>
struct DataType< ::moobot_msgs::bms_status_<ContainerAllocator> >
{
  static const char* value()
  {
    return "moobot_msgs/bms_status";
  }

  static const char* value(const ::moobot_msgs::bms_status_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::moobot_msgs::bms_status_<ContainerAllocator> >
{
  static const char* value()
  {
    return "float32 battery_volt_1\n"
"float32 battery_volt_2\n"
"float32 battery_volt_total\n"
"float32 battery_temp_1\n"
"float32 battery_temp_2\n"
"float32 current_main\n"
"float32 total_power_wh\n"
;
  }

  static const char* value(const ::moobot_msgs::bms_status_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::moobot_msgs::bms_status_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.battery_volt_1);
      stream.next(m.battery_volt_2);
      stream.next(m.battery_volt_total);
      stream.next(m.battery_temp_1);
      stream.next(m.battery_temp_2);
      stream.next(m.current_main);
      stream.next(m.total_power_wh);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct bms_status_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::moobot_msgs::bms_status_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::moobot_msgs::bms_status_<ContainerAllocator>& v)
  {
    s << indent << "battery_volt_1: ";
    Printer<float>::stream(s, indent + "  ", v.battery_volt_1);
    s << indent << "battery_volt_2: ";
    Printer<float>::stream(s, indent + "  ", v.battery_volt_2);
    s << indent << "battery_volt_total: ";
    Printer<float>::stream(s, indent + "  ", v.battery_volt_total);
    s << indent << "battery_temp_1: ";
    Printer<float>::stream(s, indent + "  ", v.battery_temp_1);
    s << indent << "battery_temp_2: ";
    Printer<float>::stream(s, indent + "  ", v.battery_temp_2);
    s << indent << "current_main: ";
    Printer<float>::stream(s, indent + "  ", v.current_main);
    s << indent << "total_power_wh: ";
    Printer<float>::stream(s, indent + "  ", v.total_power_wh);
  }
};

} // namespace message_operations
} // namespace ros

#endif // MOOBOT_MSGS_MESSAGE_BMS_STATUS_H