// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from base_msgs:msg/ReferlinPoint.idl
// generated code does not contain a copyright notice

#ifndef BASE_MSGS__MSG__DETAIL__REFERLIN_POINT__BUILDER_HPP_
#define BASE_MSGS__MSG__DETAIL__REFERLIN_POINT__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "base_msgs/msg/detail/referlin_point__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace base_msgs
{

namespace msg
{

namespace builder
{

class Init_ReferlinPoint_rdkappa
{
public:
  explicit Init_ReferlinPoint_rdkappa(::base_msgs::msg::ReferlinPoint & msg)
  : msg_(msg)
  {}
  ::base_msgs::msg::ReferlinPoint rdkappa(::base_msgs::msg::ReferlinPoint::_rdkappa_type arg)
  {
    msg_.rdkappa = std::move(arg);
    return std::move(msg_);
  }

private:
  ::base_msgs::msg::ReferlinPoint msg_;
};

class Init_ReferlinPoint_rkappa
{
public:
  explicit Init_ReferlinPoint_rkappa(::base_msgs::msg::ReferlinPoint & msg)
  : msg_(msg)
  {}
  Init_ReferlinPoint_rdkappa rkappa(::base_msgs::msg::ReferlinPoint::_rkappa_type arg)
  {
    msg_.rkappa = std::move(arg);
    return Init_ReferlinPoint_rdkappa(msg_);
  }

private:
  ::base_msgs::msg::ReferlinPoint msg_;
};

class Init_ReferlinPoint_rtheta
{
public:
  explicit Init_ReferlinPoint_rtheta(::base_msgs::msg::ReferlinPoint & msg)
  : msg_(msg)
  {}
  Init_ReferlinPoint_rkappa rtheta(::base_msgs::msg::ReferlinPoint::_rtheta_type arg)
  {
    msg_.rtheta = std::move(arg);
    return Init_ReferlinPoint_rkappa(msg_);
  }

private:
  ::base_msgs::msg::ReferlinPoint msg_;
};

class Init_ReferlinPoint_rs
{
public:
  explicit Init_ReferlinPoint_rs(::base_msgs::msg::ReferlinPoint & msg)
  : msg_(msg)
  {}
  Init_ReferlinPoint_rtheta rs(::base_msgs::msg::ReferlinPoint::_rs_type arg)
  {
    msg_.rs = std::move(arg);
    return Init_ReferlinPoint_rtheta(msg_);
  }

private:
  ::base_msgs::msg::ReferlinPoint msg_;
};

class Init_ReferlinPoint_pose
{
public:
  Init_ReferlinPoint_pose()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_ReferlinPoint_rs pose(::base_msgs::msg::ReferlinPoint::_pose_type arg)
  {
    msg_.pose = std::move(arg);
    return Init_ReferlinPoint_rs(msg_);
  }

private:
  ::base_msgs::msg::ReferlinPoint msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::base_msgs::msg::ReferlinPoint>()
{
  return base_msgs::msg::builder::Init_ReferlinPoint_pose();
}

}  // namespace base_msgs

#endif  // BASE_MSGS__MSG__DETAIL__REFERLIN_POINT__BUILDER_HPP_
