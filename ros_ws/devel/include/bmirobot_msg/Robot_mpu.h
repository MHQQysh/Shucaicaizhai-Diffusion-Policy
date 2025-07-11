// Generated by gencpp from file bmirobot_msg/Robot_mpu.msg
// DO NOT EDIT!


#ifndef BMIROBOT_MSG_MESSAGE_ROBOT_MPU_H
#define BMIROBOT_MSG_MESSAGE_ROBOT_MPU_H


#include <string>
#include <vector>
#include <memory>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace bmirobot_msg
{
template <class ContainerAllocator>
struct Robot_mpu_
{
  typedef Robot_mpu_<ContainerAllocator> Type;

  Robot_mpu_()
    : mpu_Ax()
    , mpu_Ay()
    , mpu_Az()
    , mpu_Rx()
    , mpu_Ry()
    , mpu_Rz()  {
      mpu_Ax.assign(0);

      mpu_Ay.assign(0);

      mpu_Az.assign(0);

      mpu_Rx.assign(0);

      mpu_Ry.assign(0);

      mpu_Rz.assign(0);
  }
  Robot_mpu_(const ContainerAllocator& _alloc)
    : mpu_Ax()
    , mpu_Ay()
    , mpu_Az()
    , mpu_Rx()
    , mpu_Ry()
    , mpu_Rz()  {
  (void)_alloc;
      mpu_Ax.assign(0);

      mpu_Ay.assign(0);

      mpu_Az.assign(0);

      mpu_Rx.assign(0);

      mpu_Ry.assign(0);

      mpu_Rz.assign(0);
  }



   typedef boost::array<int16_t, 8>  _mpu_Ax_type;
  _mpu_Ax_type mpu_Ax;

   typedef boost::array<int16_t, 8>  _mpu_Ay_type;
  _mpu_Ay_type mpu_Ay;

   typedef boost::array<int16_t, 8>  _mpu_Az_type;
  _mpu_Az_type mpu_Az;

   typedef boost::array<int16_t, 8>  _mpu_Rx_type;
  _mpu_Rx_type mpu_Rx;

   typedef boost::array<int16_t, 8>  _mpu_Ry_type;
  _mpu_Ry_type mpu_Ry;

   typedef boost::array<int16_t, 8>  _mpu_Rz_type;
  _mpu_Rz_type mpu_Rz;





  typedef boost::shared_ptr< ::bmirobot_msg::Robot_mpu_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::bmirobot_msg::Robot_mpu_<ContainerAllocator> const> ConstPtr;

}; // struct Robot_mpu_

typedef ::bmirobot_msg::Robot_mpu_<std::allocator<void> > Robot_mpu;

typedef boost::shared_ptr< ::bmirobot_msg::Robot_mpu > Robot_mpuPtr;
typedef boost::shared_ptr< ::bmirobot_msg::Robot_mpu const> Robot_mpuConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::bmirobot_msg::Robot_mpu_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::bmirobot_msg::Robot_mpu_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::bmirobot_msg::Robot_mpu_<ContainerAllocator1> & lhs, const ::bmirobot_msg::Robot_mpu_<ContainerAllocator2> & rhs)
{
  return lhs.mpu_Ax == rhs.mpu_Ax &&
    lhs.mpu_Ay == rhs.mpu_Ay &&
    lhs.mpu_Az == rhs.mpu_Az &&
    lhs.mpu_Rx == rhs.mpu_Rx &&
    lhs.mpu_Ry == rhs.mpu_Ry &&
    lhs.mpu_Rz == rhs.mpu_Rz;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::bmirobot_msg::Robot_mpu_<ContainerAllocator1> & lhs, const ::bmirobot_msg::Robot_mpu_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace bmirobot_msg

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::bmirobot_msg::Robot_mpu_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::bmirobot_msg::Robot_mpu_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::bmirobot_msg::Robot_mpu_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::bmirobot_msg::Robot_mpu_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::bmirobot_msg::Robot_mpu_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::bmirobot_msg::Robot_mpu_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::bmirobot_msg::Robot_mpu_<ContainerAllocator> >
{
  static const char* value()
  {
    return "742fca3c0a78013d1f45f4495e1ad202";
  }

  static const char* value(const ::bmirobot_msg::Robot_mpu_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x742fca3c0a78013dULL;
  static const uint64_t static_value2 = 0x1f45f4495e1ad202ULL;
};

template<class ContainerAllocator>
struct DataType< ::bmirobot_msg::Robot_mpu_<ContainerAllocator> >
{
  static const char* value()
  {
    return "bmirobot_msg/Robot_mpu";
  }

  static const char* value(const ::bmirobot_msg::Robot_mpu_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::bmirobot_msg::Robot_mpu_<ContainerAllocator> >
{
  static const char* value()
  {
    return "int16[8] mpu_Ax\n"
"int16[8] mpu_Ay\n"
"int16[8] mpu_Az\n"
"int16[8] mpu_Rx\n"
"int16[8] mpu_Ry\n"
"int16[8] mpu_Rz\n"
;
  }

  static const char* value(const ::bmirobot_msg::Robot_mpu_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::bmirobot_msg::Robot_mpu_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.mpu_Ax);
      stream.next(m.mpu_Ay);
      stream.next(m.mpu_Az);
      stream.next(m.mpu_Rx);
      stream.next(m.mpu_Ry);
      stream.next(m.mpu_Rz);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct Robot_mpu_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::bmirobot_msg::Robot_mpu_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::bmirobot_msg::Robot_mpu_<ContainerAllocator>& v)
  {
    s << indent << "mpu_Ax[]" << std::endl;
    for (size_t i = 0; i < v.mpu_Ax.size(); ++i)
    {
      s << indent << "  mpu_Ax[" << i << "]: ";
      Printer<int16_t>::stream(s, indent + "  ", v.mpu_Ax[i]);
    }
    s << indent << "mpu_Ay[]" << std::endl;
    for (size_t i = 0; i < v.mpu_Ay.size(); ++i)
    {
      s << indent << "  mpu_Ay[" << i << "]: ";
      Printer<int16_t>::stream(s, indent + "  ", v.mpu_Ay[i]);
    }
    s << indent << "mpu_Az[]" << std::endl;
    for (size_t i = 0; i < v.mpu_Az.size(); ++i)
    {
      s << indent << "  mpu_Az[" << i << "]: ";
      Printer<int16_t>::stream(s, indent + "  ", v.mpu_Az[i]);
    }
    s << indent << "mpu_Rx[]" << std::endl;
    for (size_t i = 0; i < v.mpu_Rx.size(); ++i)
    {
      s << indent << "  mpu_Rx[" << i << "]: ";
      Printer<int16_t>::stream(s, indent + "  ", v.mpu_Rx[i]);
    }
    s << indent << "mpu_Ry[]" << std::endl;
    for (size_t i = 0; i < v.mpu_Ry.size(); ++i)
    {
      s << indent << "  mpu_Ry[" << i << "]: ";
      Printer<int16_t>::stream(s, indent + "  ", v.mpu_Ry[i]);
    }
    s << indent << "mpu_Rz[]" << std::endl;
    for (size_t i = 0; i < v.mpu_Rz.size(); ++i)
    {
      s << indent << "  mpu_Rz[" << i << "]: ";
      Printer<int16_t>::stream(s, indent + "  ", v.mpu_Rz[i]);
    }
  }
};

} // namespace message_operations
} // namespace ros

#endif // BMIROBOT_MSG_MESSAGE_ROBOT_MPU_H
