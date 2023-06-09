// Generated by gencpp from file puzzlebot_controller/MoveGoal.msg
// DO NOT EDIT!


#ifndef PUZZLEBOT_CONTROLLER_MESSAGE_MOVEGOAL_H
#define PUZZLEBOT_CONTROLLER_MESSAGE_MOVEGOAL_H


#include <string>
#include <vector>
#include <memory>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <geometry_msgs/Point.h>

namespace puzzlebot_controller
{
template <class ContainerAllocator>
struct MoveGoal_
{
  typedef MoveGoal_<ContainerAllocator> Type;

  MoveGoal_()
    : goal()  {
    }
  MoveGoal_(const ContainerAllocator& _alloc)
    : goal(_alloc)  {
  (void)_alloc;
    }



   typedef  ::geometry_msgs::Point_<ContainerAllocator>  _goal_type;
  _goal_type goal;





  typedef boost::shared_ptr< ::puzzlebot_controller::MoveGoal_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::puzzlebot_controller::MoveGoal_<ContainerAllocator> const> ConstPtr;

}; // struct MoveGoal_

typedef ::puzzlebot_controller::MoveGoal_<std::allocator<void> > MoveGoal;

typedef boost::shared_ptr< ::puzzlebot_controller::MoveGoal > MoveGoalPtr;
typedef boost::shared_ptr< ::puzzlebot_controller::MoveGoal const> MoveGoalConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::puzzlebot_controller::MoveGoal_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::puzzlebot_controller::MoveGoal_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::puzzlebot_controller::MoveGoal_<ContainerAllocator1> & lhs, const ::puzzlebot_controller::MoveGoal_<ContainerAllocator2> & rhs)
{
  return lhs.goal == rhs.goal;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::puzzlebot_controller::MoveGoal_<ContainerAllocator1> & lhs, const ::puzzlebot_controller::MoveGoal_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace puzzlebot_controller

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::puzzlebot_controller::MoveGoal_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::puzzlebot_controller::MoveGoal_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::puzzlebot_controller::MoveGoal_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::puzzlebot_controller::MoveGoal_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::puzzlebot_controller::MoveGoal_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::puzzlebot_controller::MoveGoal_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::puzzlebot_controller::MoveGoal_<ContainerAllocator> >
{
  static const char* value()
  {
    return "8ad3bd0e46ff6777ce7cd2fdd945cb9e";
  }

  static const char* value(const ::puzzlebot_controller::MoveGoal_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x8ad3bd0e46ff6777ULL;
  static const uint64_t static_value2 = 0xce7cd2fdd945cb9eULL;
};

template<class ContainerAllocator>
struct DataType< ::puzzlebot_controller::MoveGoal_<ContainerAllocator> >
{
  static const char* value()
  {
    return "puzzlebot_controller/MoveGoal";
  }

  static const char* value(const ::puzzlebot_controller::MoveGoal_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::puzzlebot_controller::MoveGoal_<ContainerAllocator> >
{
  static const char* value()
  {
    return "# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======\n"
"# Goal to move to, only x, z are used\n"
"# Relative to starting position\n"
"# If rot / translations are sent in the same goal,\n"
"# translation will be applied first\n"
"geometry_msgs/Point goal\n"
"\n"
"================================================================================\n"
"MSG: geometry_msgs/Point\n"
"# This contains the position of a point in free space\n"
"float64 x\n"
"float64 y\n"
"float64 z\n"
;
  }

  static const char* value(const ::puzzlebot_controller::MoveGoal_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::puzzlebot_controller::MoveGoal_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.goal);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct MoveGoal_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::puzzlebot_controller::MoveGoal_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::puzzlebot_controller::MoveGoal_<ContainerAllocator>& v)
  {
    s << indent << "goal: ";
    s << std::endl;
    Printer< ::geometry_msgs::Point_<ContainerAllocator> >::stream(s, indent + "  ", v.goal);
  }
};

} // namespace message_operations
} // namespace ros

#endif // PUZZLEBOT_CONTROLLER_MESSAGE_MOVEGOAL_H
