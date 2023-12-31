// Generated by gencpp from file ds4_driver/Report.msg
// DO NOT EDIT!


#ifndef DS4_DRIVER_MESSAGE_REPORT_H
#define DS4_DRIVER_MESSAGE_REPORT_H


#include <string>
#include <vector>
#include <memory>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <std_msgs/Header.h>

namespace ds4_driver
{
template <class ContainerAllocator>
struct Report_
{
  typedef Report_<ContainerAllocator> Type;

  Report_()
    : header()
    , left_analog_x(0)
    , left_analog_y(0)
    , right_analog_x(0)
    , right_analog_y(0)
    , l2_analog(0)
    , r2_analog(0)
    , dpad_up(false)
    , dpad_down(false)
    , dpad_left(false)
    , dpad_right(false)
    , button_cross(false)
    , button_circle(false)
    , button_square(false)
    , button_triangle(false)
    , button_l1(false)
    , button_l2(false)
    , button_l3(false)
    , button_r1(false)
    , button_r2(false)
    , button_r3(false)
    , button_share(false)
    , button_options(false)
    , button_trackpad(false)
    , button_ps(false)
    , lin_acc_x(0)
    , lin_acc_y(0)
    , lin_acc_z(0)
    , ang_vel_x(0)
    , ang_vel_y(0)
    , ang_vel_z(0)
    , trackpad_touch0_id(0)
    , trackpad_touch0_active(0)
    , trackpad_touch0_x(0)
    , trackpad_touch0_y(0)
    , trackpad_touch1_id(0)
    , trackpad_touch1_active(0)
    , trackpad_touch1_x(0)
    , trackpad_touch1_y(0)
    , timestamp(0)
    , battery(0)
    , plug_usb(false)
    , plug_audio(false)
    , plug_mic(false)  {
    }
  Report_(const ContainerAllocator& _alloc)
    : header(_alloc)
    , left_analog_x(0)
    , left_analog_y(0)
    , right_analog_x(0)
    , right_analog_y(0)
    , l2_analog(0)
    , r2_analog(0)
    , dpad_up(false)
    , dpad_down(false)
    , dpad_left(false)
    , dpad_right(false)
    , button_cross(false)
    , button_circle(false)
    , button_square(false)
    , button_triangle(false)
    , button_l1(false)
    , button_l2(false)
    , button_l3(false)
    , button_r1(false)
    , button_r2(false)
    , button_r3(false)
    , button_share(false)
    , button_options(false)
    , button_trackpad(false)
    , button_ps(false)
    , lin_acc_x(0)
    , lin_acc_y(0)
    , lin_acc_z(0)
    , ang_vel_x(0)
    , ang_vel_y(0)
    , ang_vel_z(0)
    , trackpad_touch0_id(0)
    , trackpad_touch0_active(0)
    , trackpad_touch0_x(0)
    , trackpad_touch0_y(0)
    , trackpad_touch1_id(0)
    , trackpad_touch1_active(0)
    , trackpad_touch1_x(0)
    , trackpad_touch1_y(0)
    , timestamp(0)
    , battery(0)
    , plug_usb(false)
    , plug_audio(false)
    , plug_mic(false)  {
  (void)_alloc;
    }



   typedef  ::std_msgs::Header_<ContainerAllocator>  _header_type;
  _header_type header;

   typedef uint8_t _left_analog_x_type;
  _left_analog_x_type left_analog_x;

   typedef uint8_t _left_analog_y_type;
  _left_analog_y_type left_analog_y;

   typedef uint8_t _right_analog_x_type;
  _right_analog_x_type right_analog_x;

   typedef uint8_t _right_analog_y_type;
  _right_analog_y_type right_analog_y;

   typedef uint8_t _l2_analog_type;
  _l2_analog_type l2_analog;

   typedef uint8_t _r2_analog_type;
  _r2_analog_type r2_analog;

   typedef uint8_t _dpad_up_type;
  _dpad_up_type dpad_up;

   typedef uint8_t _dpad_down_type;
  _dpad_down_type dpad_down;

   typedef uint8_t _dpad_left_type;
  _dpad_left_type dpad_left;

   typedef uint8_t _dpad_right_type;
  _dpad_right_type dpad_right;

   typedef uint8_t _button_cross_type;
  _button_cross_type button_cross;

   typedef uint8_t _button_circle_type;
  _button_circle_type button_circle;

   typedef uint8_t _button_square_type;
  _button_square_type button_square;

   typedef uint8_t _button_triangle_type;
  _button_triangle_type button_triangle;

   typedef uint8_t _button_l1_type;
  _button_l1_type button_l1;

   typedef uint8_t _button_l2_type;
  _button_l2_type button_l2;

   typedef uint8_t _button_l3_type;
  _button_l3_type button_l3;

   typedef uint8_t _button_r1_type;
  _button_r1_type button_r1;

   typedef uint8_t _button_r2_type;
  _button_r2_type button_r2;

   typedef uint8_t _button_r3_type;
  _button_r3_type button_r3;

   typedef uint8_t _button_share_type;
  _button_share_type button_share;

   typedef uint8_t _button_options_type;
  _button_options_type button_options;

   typedef uint8_t _button_trackpad_type;
  _button_trackpad_type button_trackpad;

   typedef uint8_t _button_ps_type;
  _button_ps_type button_ps;

   typedef int16_t _lin_acc_x_type;
  _lin_acc_x_type lin_acc_x;

   typedef int16_t _lin_acc_y_type;
  _lin_acc_y_type lin_acc_y;

   typedef int16_t _lin_acc_z_type;
  _lin_acc_z_type lin_acc_z;

   typedef int16_t _ang_vel_x_type;
  _ang_vel_x_type ang_vel_x;

   typedef int16_t _ang_vel_y_type;
  _ang_vel_y_type ang_vel_y;

   typedef int16_t _ang_vel_z_type;
  _ang_vel_z_type ang_vel_z;

   typedef uint16_t _trackpad_touch0_id_type;
  _trackpad_touch0_id_type trackpad_touch0_id;

   typedef uint16_t _trackpad_touch0_active_type;
  _trackpad_touch0_active_type trackpad_touch0_active;

   typedef uint16_t _trackpad_touch0_x_type;
  _trackpad_touch0_x_type trackpad_touch0_x;

   typedef uint16_t _trackpad_touch0_y_type;
  _trackpad_touch0_y_type trackpad_touch0_y;

   typedef uint16_t _trackpad_touch1_id_type;
  _trackpad_touch1_id_type trackpad_touch1_id;

   typedef uint16_t _trackpad_touch1_active_type;
  _trackpad_touch1_active_type trackpad_touch1_active;

   typedef uint16_t _trackpad_touch1_x_type;
  _trackpad_touch1_x_type trackpad_touch1_x;

   typedef uint16_t _trackpad_touch1_y_type;
  _trackpad_touch1_y_type trackpad_touch1_y;

   typedef uint8_t _timestamp_type;
  _timestamp_type timestamp;

   typedef uint8_t _battery_type;
  _battery_type battery;

   typedef uint8_t _plug_usb_type;
  _plug_usb_type plug_usb;

   typedef uint8_t _plug_audio_type;
  _plug_audio_type plug_audio;

   typedef uint8_t _plug_mic_type;
  _plug_mic_type plug_mic;





  typedef boost::shared_ptr< ::ds4_driver::Report_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::ds4_driver::Report_<ContainerAllocator> const> ConstPtr;

}; // struct Report_

typedef ::ds4_driver::Report_<std::allocator<void> > Report;

typedef boost::shared_ptr< ::ds4_driver::Report > ReportPtr;
typedef boost::shared_ptr< ::ds4_driver::Report const> ReportConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::ds4_driver::Report_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::ds4_driver::Report_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::ds4_driver::Report_<ContainerAllocator1> & lhs, const ::ds4_driver::Report_<ContainerAllocator2> & rhs)
{
  return lhs.header == rhs.header &&
    lhs.left_analog_x == rhs.left_analog_x &&
    lhs.left_analog_y == rhs.left_analog_y &&
    lhs.right_analog_x == rhs.right_analog_x &&
    lhs.right_analog_y == rhs.right_analog_y &&
    lhs.l2_analog == rhs.l2_analog &&
    lhs.r2_analog == rhs.r2_analog &&
    lhs.dpad_up == rhs.dpad_up &&
    lhs.dpad_down == rhs.dpad_down &&
    lhs.dpad_left == rhs.dpad_left &&
    lhs.dpad_right == rhs.dpad_right &&
    lhs.button_cross == rhs.button_cross &&
    lhs.button_circle == rhs.button_circle &&
    lhs.button_square == rhs.button_square &&
    lhs.button_triangle == rhs.button_triangle &&
    lhs.button_l1 == rhs.button_l1 &&
    lhs.button_l2 == rhs.button_l2 &&
    lhs.button_l3 == rhs.button_l3 &&
    lhs.button_r1 == rhs.button_r1 &&
    lhs.button_r2 == rhs.button_r2 &&
    lhs.button_r3 == rhs.button_r3 &&
    lhs.button_share == rhs.button_share &&
    lhs.button_options == rhs.button_options &&
    lhs.button_trackpad == rhs.button_trackpad &&
    lhs.button_ps == rhs.button_ps &&
    lhs.lin_acc_x == rhs.lin_acc_x &&
    lhs.lin_acc_y == rhs.lin_acc_y &&
    lhs.lin_acc_z == rhs.lin_acc_z &&
    lhs.ang_vel_x == rhs.ang_vel_x &&
    lhs.ang_vel_y == rhs.ang_vel_y &&
    lhs.ang_vel_z == rhs.ang_vel_z &&
    lhs.trackpad_touch0_id == rhs.trackpad_touch0_id &&
    lhs.trackpad_touch0_active == rhs.trackpad_touch0_active &&
    lhs.trackpad_touch0_x == rhs.trackpad_touch0_x &&
    lhs.trackpad_touch0_y == rhs.trackpad_touch0_y &&
    lhs.trackpad_touch1_id == rhs.trackpad_touch1_id &&
    lhs.trackpad_touch1_active == rhs.trackpad_touch1_active &&
    lhs.trackpad_touch1_x == rhs.trackpad_touch1_x &&
    lhs.trackpad_touch1_y == rhs.trackpad_touch1_y &&
    lhs.timestamp == rhs.timestamp &&
    lhs.battery == rhs.battery &&
    lhs.plug_usb == rhs.plug_usb &&
    lhs.plug_audio == rhs.plug_audio &&
    lhs.plug_mic == rhs.plug_mic;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::ds4_driver::Report_<ContainerAllocator1> & lhs, const ::ds4_driver::Report_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace ds4_driver

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::ds4_driver::Report_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::ds4_driver::Report_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::ds4_driver::Report_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::ds4_driver::Report_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::ds4_driver::Report_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::ds4_driver::Report_<ContainerAllocator> const>
  : TrueType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::ds4_driver::Report_<ContainerAllocator> >
{
  static const char* value()
  {
    return "ec2c37165ced5aec5b7a50d72696b7dc";
  }

  static const char* value(const ::ds4_driver::Report_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xec2c37165ced5aecULL;
  static const uint64_t static_value2 = 0x5b7a50d72696b7dcULL;
};

template<class ContainerAllocator>
struct DataType< ::ds4_driver::Report_<ContainerAllocator> >
{
  static const char* value()
  {
    return "ds4_driver/Report";
  }

  static const char* value(const ::ds4_driver::Report_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::ds4_driver::Report_<ContainerAllocator> >
{
  static const char* value()
  {
    return "# Raw report from DualShock 4\n"
"Header header\n"
"\n"
"# Left: 0, Right: 255\n"
"uint8 left_analog_x\n"
"# Up: 0, Down: 255\n"
"uint8 left_analog_y\n"
"uint8 right_analog_x\n"
"uint8 right_analog_y\n"
"\n"
"# Released: 0, Pressed: 255\n"
"uint8 l2_analog\n"
"# Released: 0, Pressed: 255\n"
"uint8 r2_analog\n"
"\n"
"# Released: 0, Pressed: 1\n"
"bool dpad_up\n"
"bool dpad_down\n"
"bool dpad_left\n"
"bool dpad_right\n"
"bool button_cross\n"
"bool button_circle\n"
"bool button_square\n"
"bool button_triangle\n"
"bool button_l1\n"
"bool button_l2\n"
"bool button_l3\n"
"bool button_r1\n"
"bool button_r2\n"
"bool button_r3\n"
"bool button_share\n"
"bool button_options\n"
"bool button_trackpad\n"
"bool button_ps\n"
"\n"
"# IMU\n"
"int16 lin_acc_x\n"
"int16 lin_acc_y\n"
"int16 lin_acc_z\n"
"int16 ang_vel_x\n"
"int16 ang_vel_y\n"
"int16 ang_vel_z\n"
"\n"
"# Top-left: (0, 0), Bottom-right: (1919, 942)\n"
"uint16 trackpad_touch0_id\n"
"uint16 trackpad_touch0_active\n"
"uint16 trackpad_touch0_x\n"
"uint16 trackpad_touch0_y\n"
"uint16 trackpad_touch1_id\n"
"uint16 trackpad_touch1_active\n"
"uint16 trackpad_touch1_x\n"
"uint16 trackpad_touch1_y\n"
"\n"
"uint8 timestamp\n"
"# Full: 8, Full (and charging): 11\n"
"uint8 battery\n"
"\n"
"# Unused: 0, Plugged: 1\n"
"bool plug_usb\n"
"bool plug_audio\n"
"bool plug_mic\n"
"\n"
"================================================================================\n"
"MSG: std_msgs/Header\n"
"# Standard metadata for higher-level stamped data types.\n"
"# This is generally used to communicate timestamped data \n"
"# in a particular coordinate frame.\n"
"# \n"
"# sequence ID: consecutively increasing ID \n"
"uint32 seq\n"
"#Two-integer timestamp that is expressed as:\n"
"# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')\n"
"# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')\n"
"# time-handling sugar is provided by the client library\n"
"time stamp\n"
"#Frame this data is associated with\n"
"string frame_id\n"
;
  }

  static const char* value(const ::ds4_driver::Report_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::ds4_driver::Report_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.header);
      stream.next(m.left_analog_x);
      stream.next(m.left_analog_y);
      stream.next(m.right_analog_x);
      stream.next(m.right_analog_y);
      stream.next(m.l2_analog);
      stream.next(m.r2_analog);
      stream.next(m.dpad_up);
      stream.next(m.dpad_down);
      stream.next(m.dpad_left);
      stream.next(m.dpad_right);
      stream.next(m.button_cross);
      stream.next(m.button_circle);
      stream.next(m.button_square);
      stream.next(m.button_triangle);
      stream.next(m.button_l1);
      stream.next(m.button_l2);
      stream.next(m.button_l3);
      stream.next(m.button_r1);
      stream.next(m.button_r2);
      stream.next(m.button_r3);
      stream.next(m.button_share);
      stream.next(m.button_options);
      stream.next(m.button_trackpad);
      stream.next(m.button_ps);
      stream.next(m.lin_acc_x);
      stream.next(m.lin_acc_y);
      stream.next(m.lin_acc_z);
      stream.next(m.ang_vel_x);
      stream.next(m.ang_vel_y);
      stream.next(m.ang_vel_z);
      stream.next(m.trackpad_touch0_id);
      stream.next(m.trackpad_touch0_active);
      stream.next(m.trackpad_touch0_x);
      stream.next(m.trackpad_touch0_y);
      stream.next(m.trackpad_touch1_id);
      stream.next(m.trackpad_touch1_active);
      stream.next(m.trackpad_touch1_x);
      stream.next(m.trackpad_touch1_y);
      stream.next(m.timestamp);
      stream.next(m.battery);
      stream.next(m.plug_usb);
      stream.next(m.plug_audio);
      stream.next(m.plug_mic);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct Report_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::ds4_driver::Report_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::ds4_driver::Report_<ContainerAllocator>& v)
  {
    s << indent << "header: ";
    s << std::endl;
    Printer< ::std_msgs::Header_<ContainerAllocator> >::stream(s, indent + "  ", v.header);
    s << indent << "left_analog_x: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.left_analog_x);
    s << indent << "left_analog_y: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.left_analog_y);
    s << indent << "right_analog_x: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.right_analog_x);
    s << indent << "right_analog_y: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.right_analog_y);
    s << indent << "l2_analog: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.l2_analog);
    s << indent << "r2_analog: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.r2_analog);
    s << indent << "dpad_up: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.dpad_up);
    s << indent << "dpad_down: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.dpad_down);
    s << indent << "dpad_left: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.dpad_left);
    s << indent << "dpad_right: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.dpad_right);
    s << indent << "button_cross: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.button_cross);
    s << indent << "button_circle: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.button_circle);
    s << indent << "button_square: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.button_square);
    s << indent << "button_triangle: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.button_triangle);
    s << indent << "button_l1: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.button_l1);
    s << indent << "button_l2: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.button_l2);
    s << indent << "button_l3: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.button_l3);
    s << indent << "button_r1: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.button_r1);
    s << indent << "button_r2: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.button_r2);
    s << indent << "button_r3: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.button_r3);
    s << indent << "button_share: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.button_share);
    s << indent << "button_options: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.button_options);
    s << indent << "button_trackpad: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.button_trackpad);
    s << indent << "button_ps: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.button_ps);
    s << indent << "lin_acc_x: ";
    Printer<int16_t>::stream(s, indent + "  ", v.lin_acc_x);
    s << indent << "lin_acc_y: ";
    Printer<int16_t>::stream(s, indent + "  ", v.lin_acc_y);
    s << indent << "lin_acc_z: ";
    Printer<int16_t>::stream(s, indent + "  ", v.lin_acc_z);
    s << indent << "ang_vel_x: ";
    Printer<int16_t>::stream(s, indent + "  ", v.ang_vel_x);
    s << indent << "ang_vel_y: ";
    Printer<int16_t>::stream(s, indent + "  ", v.ang_vel_y);
    s << indent << "ang_vel_z: ";
    Printer<int16_t>::stream(s, indent + "  ", v.ang_vel_z);
    s << indent << "trackpad_touch0_id: ";
    Printer<uint16_t>::stream(s, indent + "  ", v.trackpad_touch0_id);
    s << indent << "trackpad_touch0_active: ";
    Printer<uint16_t>::stream(s, indent + "  ", v.trackpad_touch0_active);
    s << indent << "trackpad_touch0_x: ";
    Printer<uint16_t>::stream(s, indent + "  ", v.trackpad_touch0_x);
    s << indent << "trackpad_touch0_y: ";
    Printer<uint16_t>::stream(s, indent + "  ", v.trackpad_touch0_y);
    s << indent << "trackpad_touch1_id: ";
    Printer<uint16_t>::stream(s, indent + "  ", v.trackpad_touch1_id);
    s << indent << "trackpad_touch1_active: ";
    Printer<uint16_t>::stream(s, indent + "  ", v.trackpad_touch1_active);
    s << indent << "trackpad_touch1_x: ";
    Printer<uint16_t>::stream(s, indent + "  ", v.trackpad_touch1_x);
    s << indent << "trackpad_touch1_y: ";
    Printer<uint16_t>::stream(s, indent + "  ", v.trackpad_touch1_y);
    s << indent << "timestamp: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.timestamp);
    s << indent << "battery: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.battery);
    s << indent << "plug_usb: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.plug_usb);
    s << indent << "plug_audio: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.plug_audio);
    s << indent << "plug_mic: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.plug_mic);
  }
};

} // namespace message_operations
} // namespace ros

#endif // DS4_DRIVER_MESSAGE_REPORT_H
