// Generated by gencpp from file gps_common/GPSStatus.msg
// DO NOT EDIT!


#ifndef GPS_COMMON_MESSAGE_GPSSTATUS_H
#define GPS_COMMON_MESSAGE_GPSSTATUS_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <std_msgs/Header.h>

namespace gps_common
{
template <class ContainerAllocator>
struct GPSStatus_
{
  typedef GPSStatus_<ContainerAllocator> Type;

  GPSStatus_()
    : header()
    , satellites_used(0)
    , satellite_used_prn()
    , satellites_visible(0)
    , satellite_visible_prn()
    , satellite_visible_z()
    , satellite_visible_azimuth()
    , satellite_visible_snr()
    , status(0)
    , motion_source(0)
    , orientation_source(0)
    , position_source(0)  {
    }
  GPSStatus_(const ContainerAllocator& _alloc)
    : header(_alloc)
    , satellites_used(0)
    , satellite_used_prn(_alloc)
    , satellites_visible(0)
    , satellite_visible_prn(_alloc)
    , satellite_visible_z(_alloc)
    , satellite_visible_azimuth(_alloc)
    , satellite_visible_snr(_alloc)
    , status(0)
    , motion_source(0)
    , orientation_source(0)
    , position_source(0)  {
    }



   typedef  ::std_msgs::Header_<ContainerAllocator>  _header_type;
  _header_type header;

   typedef uint16_t _satellites_used_type;
  _satellites_used_type satellites_used;

   typedef std::vector<int32_t, typename ContainerAllocator::template rebind<int32_t>::other >  _satellite_used_prn_type;
  _satellite_used_prn_type satellite_used_prn;

   typedef uint16_t _satellites_visible_type;
  _satellites_visible_type satellites_visible;

   typedef std::vector<int32_t, typename ContainerAllocator::template rebind<int32_t>::other >  _satellite_visible_prn_type;
  _satellite_visible_prn_type satellite_visible_prn;

   typedef std::vector<int32_t, typename ContainerAllocator::template rebind<int32_t>::other >  _satellite_visible_z_type;
  _satellite_visible_z_type satellite_visible_z;

   typedef std::vector<int32_t, typename ContainerAllocator::template rebind<int32_t>::other >  _satellite_visible_azimuth_type;
  _satellite_visible_azimuth_type satellite_visible_azimuth;

   typedef std::vector<int32_t, typename ContainerAllocator::template rebind<int32_t>::other >  _satellite_visible_snr_type;
  _satellite_visible_snr_type satellite_visible_snr;

   typedef int16_t _status_type;
  _status_type status;

   typedef uint16_t _motion_source_type;
  _motion_source_type motion_source;

   typedef uint16_t _orientation_source_type;
  _orientation_source_type orientation_source;

   typedef uint16_t _position_source_type;
  _position_source_type position_source;


    enum { STATUS_NO_FIX = -1 };
     enum { STATUS_FIX = 0 };
     enum { STATUS_SBAS_FIX = 1 };
     enum { STATUS_GBAS_FIX = 2 };
     enum { STATUS_DGPS_FIX = 18 };
     enum { STATUS_WAAS_FIX = 33 };
     enum { SOURCE_NONE = 0u };
     enum { SOURCE_GPS = 1u };
     enum { SOURCE_POINTS = 2u };
     enum { SOURCE_DOPPLER = 4u };
     enum { SOURCE_ALTIMETER = 8u };
     enum { SOURCE_MAGNETIC = 16u };
     enum { SOURCE_GYRO = 32u };
     enum { SOURCE_ACCEL = 64u };
 

  typedef boost::shared_ptr< ::gps_common::GPSStatus_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::gps_common::GPSStatus_<ContainerAllocator> const> ConstPtr;

}; // struct GPSStatus_

typedef ::gps_common::GPSStatus_<std::allocator<void> > GPSStatus;

typedef boost::shared_ptr< ::gps_common::GPSStatus > GPSStatusPtr;
typedef boost::shared_ptr< ::gps_common::GPSStatus const> GPSStatusConstPtr;

// constants requiring out of line definition

   

   

   

   

   

   

   

   

   

   

   

   

   

   



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::gps_common::GPSStatus_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::gps_common::GPSStatus_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace gps_common

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': False, 'IsMessage': True, 'HasHeader': True}
// {'gps_common': ['/tmp/buildd/ros-indigo-gps-common-0.1.7-0trusty-20150326-1815/msg'], 'std_msgs': ['/opt/ros/indigo/share/std_msgs/cmake/../msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::gps_common::GPSStatus_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::gps_common::GPSStatus_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct IsMessage< ::gps_common::GPSStatus_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::gps_common::GPSStatus_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::gps_common::GPSStatus_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::gps_common::GPSStatus_<ContainerAllocator> const>
  : TrueType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::gps_common::GPSStatus_<ContainerAllocator> >
{
  static const char* value()
  {
    return "313baa8951fdd056c78bf61b1b07d249";
  }

  static const char* value(const ::gps_common::GPSStatus_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x313baa8951fdd056ULL;
  static const uint64_t static_value2 = 0xc78bf61b1b07d249ULL;
};

template<class ContainerAllocator>
struct DataType< ::gps_common::GPSStatus_<ContainerAllocator> >
{
  static const char* value()
  {
    return "gps_common/GPSStatus";
  }

  static const char* value(const ::gps_common::GPSStatus_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::gps_common::GPSStatus_<ContainerAllocator> >
{
  static const char* value()
  {
    return "Header header\n\
\n\
# Satellites used in solution\n\
uint16 satellites_used # Number of satellites\n\
int32[] satellite_used_prn # PRN identifiers\n\
\n\
# Satellites visible\n\
uint16 satellites_visible\n\
int32[] satellite_visible_prn # PRN identifiers\n\
int32[] satellite_visible_z # Elevation of satellites\n\
int32[] satellite_visible_azimuth # Azimuth of satellites\n\
int32[] satellite_visible_snr # Signal-to-noise ratios (dB)\n\
\n\
# Measurement status\n\
int16 STATUS_NO_FIX=-1   # Unable to fix position\n\
int16 STATUS_FIX=0       # Normal fix\n\
int16 STATUS_SBAS_FIX=1  # Fixed using a satellite-based augmentation system\n\
int16 STATUS_GBAS_FIX=2  #          or a ground-based augmentation system\n\
int16 STATUS_DGPS_FIX=18 # Fixed with DGPS\n\
int16 STATUS_WAAS_FIX=33 # Fixed with WAAS\n\
int16 status\n\
\n\
uint16 SOURCE_NONE=0 # No information is available\n\
uint16 SOURCE_GPS=1 # Using standard GPS location [only valid for position_source]\n\
uint16 SOURCE_POINTS=2 # Motion/orientation fix is derived from successive points\n\
uint16 SOURCE_DOPPLER=4 # Motion is derived using the Doppler effect\n\
uint16 SOURCE_ALTIMETER=8 # Using an altimeter\n\
uint16 SOURCE_MAGNETIC=16 # Using magnetic sensors\n\
uint16 SOURCE_GYRO=32 # Using gyroscopes\n\
uint16 SOURCE_ACCEL=64 # Using accelerometers\n\
\n\
uint16 motion_source # Source for speed, climb and track\n\
uint16 orientation_source # Source for device orientation\n\
uint16 position_source # Source for position\n\
\n\
\n\
================================================================================\n\
MSG: std_msgs/Header\n\
# Standard metadata for higher-level stamped data types.\n\
# This is generally used to communicate timestamped data \n\
# in a particular coordinate frame.\n\
# \n\
# sequence ID: consecutively increasing ID \n\
uint32 seq\n\
#Two-integer timestamp that is expressed as:\n\
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')\n\
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')\n\
# time-handling sugar is provided by the client library\n\
time stamp\n\
#Frame this data is associated with\n\
# 0: no frame\n\
# 1: global frame\n\
string frame_id\n\
";
  }

  static const char* value(const ::gps_common::GPSStatus_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::gps_common::GPSStatus_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.header);
      stream.next(m.satellites_used);
      stream.next(m.satellite_used_prn);
      stream.next(m.satellites_visible);
      stream.next(m.satellite_visible_prn);
      stream.next(m.satellite_visible_z);
      stream.next(m.satellite_visible_azimuth);
      stream.next(m.satellite_visible_snr);
      stream.next(m.status);
      stream.next(m.motion_source);
      stream.next(m.orientation_source);
      stream.next(m.position_source);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER;
  }; // struct GPSStatus_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::gps_common::GPSStatus_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::gps_common::GPSStatus_<ContainerAllocator>& v)
  {
    s << indent << "header: ";
    s << std::endl;
    Printer< ::std_msgs::Header_<ContainerAllocator> >::stream(s, indent + "  ", v.header);
    s << indent << "satellites_used: ";
    Printer<uint16_t>::stream(s, indent + "  ", v.satellites_used);
    s << indent << "satellite_used_prn[]" << std::endl;
    for (size_t i = 0; i < v.satellite_used_prn.size(); ++i)
    {
      s << indent << "  satellite_used_prn[" << i << "]: ";
      Printer<int32_t>::stream(s, indent + "  ", v.satellite_used_prn[i]);
    }
    s << indent << "satellites_visible: ";
    Printer<uint16_t>::stream(s, indent + "  ", v.satellites_visible);
    s << indent << "satellite_visible_prn[]" << std::endl;
    for (size_t i = 0; i < v.satellite_visible_prn.size(); ++i)
    {
      s << indent << "  satellite_visible_prn[" << i << "]: ";
      Printer<int32_t>::stream(s, indent + "  ", v.satellite_visible_prn[i]);
    }
    s << indent << "satellite_visible_z[]" << std::endl;
    for (size_t i = 0; i < v.satellite_visible_z.size(); ++i)
    {
      s << indent << "  satellite_visible_z[" << i << "]: ";
      Printer<int32_t>::stream(s, indent + "  ", v.satellite_visible_z[i]);
    }
    s << indent << "satellite_visible_azimuth[]" << std::endl;
    for (size_t i = 0; i < v.satellite_visible_azimuth.size(); ++i)
    {
      s << indent << "  satellite_visible_azimuth[" << i << "]: ";
      Printer<int32_t>::stream(s, indent + "  ", v.satellite_visible_azimuth[i]);
    }
    s << indent << "satellite_visible_snr[]" << std::endl;
    for (size_t i = 0; i < v.satellite_visible_snr.size(); ++i)
    {
      s << indent << "  satellite_visible_snr[" << i << "]: ";
      Printer<int32_t>::stream(s, indent + "  ", v.satellite_visible_snr[i]);
    }
    s << indent << "status: ";
    Printer<int16_t>::stream(s, indent + "  ", v.status);
    s << indent << "motion_source: ";
    Printer<uint16_t>::stream(s, indent + "  ", v.motion_source);
    s << indent << "orientation_source: ";
    Printer<uint16_t>::stream(s, indent + "  ", v.orientation_source);
    s << indent << "position_source: ";
    Printer<uint16_t>::stream(s, indent + "  ", v.position_source);
  }
};

} // namespace message_operations
} // namespace ros

#endif // GPS_COMMON_MESSAGE_GPSSTATUS_H
