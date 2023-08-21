// Generated by gencpp from file ublox_msgs/CfgNMEA7.msg
// DO NOT EDIT!


#ifndef UBLOX_MSGS_MESSAGE_CFGNMEA7_H
#define UBLOX_MSGS_MESSAGE_CFGNMEA7_H


#include <string>
#include <vector>
#include <memory>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace ublox_msgs
{
template <class ContainerAllocator>
struct CfgNMEA7_
{
  typedef CfgNMEA7_<ContainerAllocator> Type;

  CfgNMEA7_()
    : filter(0)
    , nmeaVersion(0)
    , numSV(0)
    , flags(0)
    , gnssToFilter(0)
    , svNumbering(0)
    , mainTalkerId(0)
    , gsvTalkerId(0)
    , reserved(0)  {
    }
  CfgNMEA7_(const ContainerAllocator& _alloc)
    : filter(0)
    , nmeaVersion(0)
    , numSV(0)
    , flags(0)
    , gnssToFilter(0)
    , svNumbering(0)
    , mainTalkerId(0)
    , gsvTalkerId(0)
    , reserved(0)  {
  (void)_alloc;
    }



   typedef uint8_t _filter_type;
  _filter_type filter;

   typedef uint8_t _nmeaVersion_type;
  _nmeaVersion_type nmeaVersion;

   typedef uint8_t _numSV_type;
  _numSV_type numSV;

   typedef uint8_t _flags_type;
  _flags_type flags;

   typedef uint32_t _gnssToFilter_type;
  _gnssToFilter_type gnssToFilter;

   typedef uint8_t _svNumbering_type;
  _svNumbering_type svNumbering;

   typedef uint8_t _mainTalkerId_type;
  _mainTalkerId_type mainTalkerId;

   typedef uint8_t _gsvTalkerId_type;
  _gsvTalkerId_type gsvTalkerId;

   typedef uint8_t _reserved_type;
  _reserved_type reserved;



// reducing the odds to have name collisions with Windows.h 
#if defined(_WIN32) && defined(CLASS_ID)
  #undef CLASS_ID
#endif
#if defined(_WIN32) && defined(MESSAGE_ID)
  #undef MESSAGE_ID
#endif
#if defined(_WIN32) && defined(FILTER_POS)
  #undef FILTER_POS
#endif
#if defined(_WIN32) && defined(FILTER_MSK_POS)
  #undef FILTER_MSK_POS
#endif
#if defined(_WIN32) && defined(FILTER_TIME)
  #undef FILTER_TIME
#endif
#if defined(_WIN32) && defined(FILTER_DATE)
  #undef FILTER_DATE
#endif
#if defined(_WIN32) && defined(FILTER_GPS_ONLY)
  #undef FILTER_GPS_ONLY
#endif
#if defined(_WIN32) && defined(FILTER_TRACK)
  #undef FILTER_TRACK
#endif
#if defined(_WIN32) && defined(NMEA_VERSION_2_3)
  #undef NMEA_VERSION_2_3
#endif
#if defined(_WIN32) && defined(NMEA_VERSION_2_1)
  #undef NMEA_VERSION_2_1
#endif
#if defined(_WIN32) && defined(NUM_SV_UNLIMITED)
  #undef NUM_SV_UNLIMITED
#endif
#if defined(_WIN32) && defined(FLAGS_COMPAT)
  #undef FLAGS_COMPAT
#endif
#if defined(_WIN32) && defined(FLAGS_CONSIDER)
  #undef FLAGS_CONSIDER
#endif
#if defined(_WIN32) && defined(GNSS_TO_FILTER_GPS)
  #undef GNSS_TO_FILTER_GPS
#endif
#if defined(_WIN32) && defined(GNSS_TO_FILTER_SBAS)
  #undef GNSS_TO_FILTER_SBAS
#endif
#if defined(_WIN32) && defined(GNSS_TO_FILTER_QZSS)
  #undef GNSS_TO_FILTER_QZSS
#endif
#if defined(_WIN32) && defined(GNSS_TO_FILTER_GLONASS)
  #undef GNSS_TO_FILTER_GLONASS
#endif
#if defined(_WIN32) && defined(SV_NUMBERING_STRICT)
  #undef SV_NUMBERING_STRICT
#endif
#if defined(_WIN32) && defined(SV_NUMBERING_EXTENDED)
  #undef SV_NUMBERING_EXTENDED
#endif
#if defined(_WIN32) && defined(MAIN_TALKER_ID_NOT_OVERRIDDEN)
  #undef MAIN_TALKER_ID_NOT_OVERRIDDEN
#endif
#if defined(_WIN32) && defined(MAIN_TALKER_ID_GP)
  #undef MAIN_TALKER_ID_GP
#endif
#if defined(_WIN32) && defined(MAIN_TALKER_ID_GL)
  #undef MAIN_TALKER_ID_GL
#endif
#if defined(_WIN32) && defined(MAIN_TALKER_ID_GN)
  #undef MAIN_TALKER_ID_GN
#endif
#if defined(_WIN32) && defined(GSV_TALKER_ID_GNSS_SPECIFIC)
  #undef GSV_TALKER_ID_GNSS_SPECIFIC
#endif
#if defined(_WIN32) && defined(GSV_TALKER_ID_MAIN)
  #undef GSV_TALKER_ID_MAIN
#endif

  enum {
    CLASS_ID = 6u,
    MESSAGE_ID = 23u,
    FILTER_POS = 1u,
    FILTER_MSK_POS = 2u,
    FILTER_TIME = 4u,
    FILTER_DATE = 8u,
    FILTER_GPS_ONLY = 16u,
    FILTER_TRACK = 32u,
    NMEA_VERSION_2_3 = 35u,
    NMEA_VERSION_2_1 = 33u,
    NUM_SV_UNLIMITED = 0u,
    FLAGS_COMPAT = 1u,
    FLAGS_CONSIDER = 2u,
    GNSS_TO_FILTER_GPS = 1u,
    GNSS_TO_FILTER_SBAS = 2u,
    GNSS_TO_FILTER_QZSS = 16u,
    GNSS_TO_FILTER_GLONASS = 32u,
    SV_NUMBERING_STRICT = 0u,
    SV_NUMBERING_EXTENDED = 1u,
    MAIN_TALKER_ID_NOT_OVERRIDDEN = 0u,
    MAIN_TALKER_ID_GP = 1u,
    MAIN_TALKER_ID_GL = 2u,
    MAIN_TALKER_ID_GN = 3u,
    GSV_TALKER_ID_GNSS_SPECIFIC = 0u,
    GSV_TALKER_ID_MAIN = 1u,
  };


  typedef boost::shared_ptr< ::ublox_msgs::CfgNMEA7_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::ublox_msgs::CfgNMEA7_<ContainerAllocator> const> ConstPtr;

}; // struct CfgNMEA7_

typedef ::ublox_msgs::CfgNMEA7_<std::allocator<void> > CfgNMEA7;

typedef boost::shared_ptr< ::ublox_msgs::CfgNMEA7 > CfgNMEA7Ptr;
typedef boost::shared_ptr< ::ublox_msgs::CfgNMEA7 const> CfgNMEA7ConstPtr;

// constants requiring out of line definition

   

   

   

   

   

   

   

   

   

   

   

   

   

   

   

   

   

   

   

   

   

   

   

   

   



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::ublox_msgs::CfgNMEA7_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::ublox_msgs::CfgNMEA7_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::ublox_msgs::CfgNMEA7_<ContainerAllocator1> & lhs, const ::ublox_msgs::CfgNMEA7_<ContainerAllocator2> & rhs)
{
  return lhs.filter == rhs.filter &&
    lhs.nmeaVersion == rhs.nmeaVersion &&
    lhs.numSV == rhs.numSV &&
    lhs.flags == rhs.flags &&
    lhs.gnssToFilter == rhs.gnssToFilter &&
    lhs.svNumbering == rhs.svNumbering &&
    lhs.mainTalkerId == rhs.mainTalkerId &&
    lhs.gsvTalkerId == rhs.gsvTalkerId &&
    lhs.reserved == rhs.reserved;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::ublox_msgs::CfgNMEA7_<ContainerAllocator1> & lhs, const ::ublox_msgs::CfgNMEA7_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace ublox_msgs

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::ublox_msgs::CfgNMEA7_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::ublox_msgs::CfgNMEA7_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::ublox_msgs::CfgNMEA7_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::ublox_msgs::CfgNMEA7_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::ublox_msgs::CfgNMEA7_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::ublox_msgs::CfgNMEA7_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::ublox_msgs::CfgNMEA7_<ContainerAllocator> >
{
  static const char* value()
  {
    return "f06212695f6e460ea30b1b436df86c2b";
  }

  static const char* value(const ::ublox_msgs::CfgNMEA7_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xf06212695f6e460eULL;
  static const uint64_t static_value2 = 0xa30b1b436df86c2bULL;
};

template<class ContainerAllocator>
struct DataType< ::ublox_msgs::CfgNMEA7_<ContainerAllocator> >
{
  static const char* value()
  {
    return "ublox_msgs/CfgNMEA7";
  }

  static const char* value(const ::ublox_msgs::CfgNMEA7_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::ublox_msgs::CfgNMEA7_<ContainerAllocator> >
{
  static const char* value()
  {
    return "# CFG-NMEA (0x06 0x17)\n"
"# NMEA protocol configuration V0\n"
"#\n"
"# Set/Get the NMEA protocol configuration. See section NMEA Protocol \n"
"# Configuration for a detailed description of the configuration effects on \n"
"# NMEA output\n"
"#\n"
"# Supported on: u-blox 7 firmware version 1.00\n"
"#\n"
"\n"
"uint8 CLASS_ID = 6\n"
"uint8 MESSAGE_ID = 23 \n"
"\n"
"uint8 filter                  # filter flags\n"
"uint8 FILTER_POS = 1          # Enable position output for failed or \n"
"                              # invalid fixes\n"
"uint8 FILTER_MSK_POS = 2      # Enable position output for invalid fixes\n"
"uint8 FILTER_TIME = 4         # Enable time output for invalid times\n"
"uint8 FILTER_DATE = 8         # Enable date output for invalid dates\n"
"uint8 FILTER_GPS_ONLY = 16    # Restrict output to GPS satellites only\n"
"uint8 FILTER_TRACK = 32       # Enable COG output even if COG is frozen\n"
"\n"
"uint8 nmeaVersion             # NMEA version\n"
"uint8 NMEA_VERSION_2_3 = 35     # Version 2.3\n"
"uint8 NMEA_VERSION_2_1 = 33     # Version 2.1\n"
"\n"
"uint8 numSV                   # Maximum Number of SVs to report per TalkerId: \n"
"                              # unlimited (0) or 8, 12, 16\n"
"uint8 NUM_SV_UNLIMITED = 0 \n"
"\n"
"uint8 flags                   # flags\n"
"uint8 FLAGS_COMPAT = 1          # enable compatibility mode.\n"
"                                # This might be needed for certain applications  \n"
"                                # when customer's NMEA parser expects a fixed \n"
"                                # number of  digits in position coordinates\n"
"uint8 FLAGS_CONSIDER = 2        # enable considering mode\n"
"\n"
"uint32 gnssToFilter           # Filters out satellites based on their GNSS.  \n"
"                              # If a bitfield is enabled, the corresponding \n"
"                              # satellites will be not output.\n"
"uint32 GNSS_TO_FILTER_GPS = 1       # Disable reporting of GPS satellites\n"
"uint32 GNSS_TO_FILTER_SBAS = 2      # Disable reporting of SBAS satellites\n"
"uint32 GNSS_TO_FILTER_QZSS = 16     # Disable reporting of QZSS satellites\n"
"uint32 GNSS_TO_FILTER_GLONASS = 32  # Disable reporting of GLONASS satellites\n"
"\n"
"uint8 svNumbering             # Configures the display of satellites that do not  \n"
"                              # have an NMEA-defined value. Note: this does not\n"
"                              # apply to satellites with an unknown ID.\n"
"uint8 SV_NUMBERING_STRICT = 0     # Strict - Satellites are not output\n"
"uint8 SV_NUMBERING_EXTENDED = 1   # Extended - Use proprietary numbering\n"
"\n"
"uint8 mainTalkerId            # By default the main Talker ID (i.e. the Talker \n"
"                              # ID used  for all messages other than GSV) is \n"
"                              # determined by the  GNSS assignment of the \n"
"                              # receiver's channels (see CfgGNSS). \n"
"                              # This field enables the main Talker ID to be \n"
"                              # overridden\n"
"uint8 MAIN_TALKER_ID_NOT_OVERRIDDEN = 0   # Main Talker ID is not overridden\n"
"uint8 MAIN_TALKER_ID_GP = 1               # Set main Talker ID to 'GP'\n"
"uint8 MAIN_TALKER_ID_GL = 2               # Set main Talker ID to 'GL'\n"
"uint8 MAIN_TALKER_ID_GN = 3               # Set main Talker ID to 'GN'\n"
"\n"
"uint8 gsvTalkerId             # By default the Talker ID for GSV messages is  \n"
"                              # GNSS specific (as defined by NMEA). This field \n"
"                              # enables the GSV Talker ID to be overridden.\n"
"uint8 GSV_TALKER_ID_GNSS_SPECIFIC = 0   # Use GNSS specific Talker ID \n"
"                                        # (as defined by NMEA)\n"
"uint8 GSV_TALKER_ID_MAIN = 1            # Use the main Talker ID\n"
"\n"
"uint8 reserved              # Reserved\n"
;
  }

  static const char* value(const ::ublox_msgs::CfgNMEA7_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::ublox_msgs::CfgNMEA7_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.filter);
      stream.next(m.nmeaVersion);
      stream.next(m.numSV);
      stream.next(m.flags);
      stream.next(m.gnssToFilter);
      stream.next(m.svNumbering);
      stream.next(m.mainTalkerId);
      stream.next(m.gsvTalkerId);
      stream.next(m.reserved);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct CfgNMEA7_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::ublox_msgs::CfgNMEA7_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::ublox_msgs::CfgNMEA7_<ContainerAllocator>& v)
  {
    s << indent << "filter: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.filter);
    s << indent << "nmeaVersion: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.nmeaVersion);
    s << indent << "numSV: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.numSV);
    s << indent << "flags: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.flags);
    s << indent << "gnssToFilter: ";
    Printer<uint32_t>::stream(s, indent + "  ", v.gnssToFilter);
    s << indent << "svNumbering: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.svNumbering);
    s << indent << "mainTalkerId: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.mainTalkerId);
    s << indent << "gsvTalkerId: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.gsvTalkerId);
    s << indent << "reserved: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.reserved);
  }
};

} // namespace message_operations
} // namespace ros

#endif // UBLOX_MSGS_MESSAGE_CFGNMEA7_H
