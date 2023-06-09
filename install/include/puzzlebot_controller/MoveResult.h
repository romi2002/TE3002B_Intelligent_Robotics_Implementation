// Generated by gencpp from file puzzlebot_controller/MoveResult.msg
// DO NOT EDIT!


#ifndef PUZZLEBOT_CONTROLLER_MESSAGE_MOVERESULT_H
#define PUZZLEBOT_CONTROLLER_MESSAGE_MOVERESULT_H


#include <string>
#include <vector>
#include <memory>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace puzzlebot_controller
{
template <class ContainerAllocator>
struct MoveResult_
{
  typedef MoveResult_<ContainerAllocator> Type;

  MoveResult_()
    {
    }
  MoveResult_(const ContainerAllocator& _alloc)
    {
  (void)_alloc;
    }







  typedef boost::shared_ptr< ::puzzlebot_controller::MoveResult_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::puzzlebot_controller::MoveResult_<ContainerAllocator> const> ConstPtr;

}; // struct MoveResult_

typedef ::puzzlebot_controller::MoveResult_<std::allocator<void> > MoveResult;

typedef boost::shared_ptr< ::puzzlebot_controller::MoveResult > MoveResultPtr;
typedef boost::shared_ptr< ::puzzlebot_controller::MoveResult const> MoveResultConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::puzzlebot_controller::MoveResult_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::puzzlebot_controller::MoveResult_<ContainerAllocator> >::stream(s, "", v);
return s;
}


} // namespace puzzlebot_controller

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::puzzlebot_controller::MoveResult_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::puzzlebot_controller::MoveResult_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::puzzlebot_controller::MoveResult_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::puzzlebot_controller::MoveResult_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::puzzlebot_controller::MoveResult_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::puzzlebot_controller::MoveResult_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::puzzlebot_controller::MoveResult_<ContainerAllocator> >
{
  static const char* value()
  {
    return "d41d8cd98f00b204e9800998ecf8427e";
  }

  static const char* value(const ::puzzlebot_controller::MoveResult_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xd41d8cd98f00b204ULL;
  static const uint64_t static_value2 = 0xe9800998ecf8427eULL;
};

template<class ContainerAllocator>
struct DataType< ::puzzlebot_controller::MoveResult_<ContainerAllocator> >
{
  static const char* value()
  {
    return "puzzlebot_controller/MoveResult";
  }

  static const char* value(const ::puzzlebot_controller::MoveResult_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::puzzlebot_controller::MoveResult_<ContainerAllocator> >
{
  static const char* value()
  {
    return "# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======\n"
"\n"
;
  }

  static const char* value(const ::puzzlebot_controller::MoveResult_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::puzzlebot_controller::MoveResult_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream&, T)
    {}

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct MoveResult_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::puzzlebot_controller::MoveResult_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream&, const std::string&, const ::puzzlebot_controller::MoveResult_<ContainerAllocator>&)
  {}
};

} // namespace message_operations
} // namespace ros

#endif // PUZZLEBOT_CONTROLLER_MESSAGE_MOVERESULT_H
