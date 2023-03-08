// Generated by gencpp from file Car_Spraying/QueryServoAngleResponse.msg
// DO NOT EDIT!


#ifndef CAR_SPRAYING_MESSAGE_QUERYSERVOANGLERESPONSE_H
#define CAR_SPRAYING_MESSAGE_QUERYSERVOANGLERESPONSE_H


#include <string>
#include <vector>
#include <memory>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace Car_Spraying
{
template <class ContainerAllocator>
struct QueryServoAngleResponse_
{
  typedef QueryServoAngleResponse_<ContainerAllocator> Type;

  QueryServoAngleResponse_()
    : angle(0.0)  {
    }
  QueryServoAngleResponse_(const ContainerAllocator& _alloc)
    : angle(0.0)  {
  (void)_alloc;
    }



   typedef float _angle_type;
  _angle_type angle;





  typedef boost::shared_ptr< ::Car_Spraying::QueryServoAngleResponse_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::Car_Spraying::QueryServoAngleResponse_<ContainerAllocator> const> ConstPtr;

}; // struct QueryServoAngleResponse_

typedef ::Car_Spraying::QueryServoAngleResponse_<std::allocator<void> > QueryServoAngleResponse;

typedef boost::shared_ptr< ::Car_Spraying::QueryServoAngleResponse > QueryServoAngleResponsePtr;
typedef boost::shared_ptr< ::Car_Spraying::QueryServoAngleResponse const> QueryServoAngleResponseConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::Car_Spraying::QueryServoAngleResponse_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::Car_Spraying::QueryServoAngleResponse_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::Car_Spraying::QueryServoAngleResponse_<ContainerAllocator1> & lhs, const ::Car_Spraying::QueryServoAngleResponse_<ContainerAllocator2> & rhs)
{
  return lhs.angle == rhs.angle;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::Car_Spraying::QueryServoAngleResponse_<ContainerAllocator1> & lhs, const ::Car_Spraying::QueryServoAngleResponse_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace Car_Spraying

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::Car_Spraying::QueryServoAngleResponse_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::Car_Spraying::QueryServoAngleResponse_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::Car_Spraying::QueryServoAngleResponse_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::Car_Spraying::QueryServoAngleResponse_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::Car_Spraying::QueryServoAngleResponse_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::Car_Spraying::QueryServoAngleResponse_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::Car_Spraying::QueryServoAngleResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "2d11dcdbe5a6f73dd324353dc52315ab";
  }

  static const char* value(const ::Car_Spraying::QueryServoAngleResponse_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x2d11dcdbe5a6f73dULL;
  static const uint64_t static_value2 = 0xd324353dc52315abULL;
};

template<class ContainerAllocator>
struct DataType< ::Car_Spraying::QueryServoAngleResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "Car_Spraying/QueryServoAngleResponse";
  }

  static const char* value(const ::Car_Spraying::QueryServoAngleResponse_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::Car_Spraying::QueryServoAngleResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "float32 angle\n"
;
  }

  static const char* value(const ::Car_Spraying::QueryServoAngleResponse_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::Car_Spraying::QueryServoAngleResponse_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.angle);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct QueryServoAngleResponse_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::Car_Spraying::QueryServoAngleResponse_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::Car_Spraying::QueryServoAngleResponse_<ContainerAllocator>& v)
  {
    s << indent << "angle: ";
    Printer<float>::stream(s, indent + "  ", v.angle);
  }
};

} // namespace message_operations
} // namespace ros

#endif // CAR_SPRAYING_MESSAGE_QUERYSERVOANGLERESPONSE_H
