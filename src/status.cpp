#include "commons/status.h"

#include "boost/make_shared.hpp"

// ------------
// CONSTRUCTORS
// ------------

rtask::commons::Status::Status(const State t_status, const std::string t_descr)
{
  setStatus(t_status, t_descr);
}

rtask::commons::Status::Status(const rtask_msgs::Status& t_msg)
{
  setFromStatusMsg(t_msg);
}

rtask::commons::Status::Status(const rtask_msgs::StatusConstPtr t_msg_ptr)
{
  setFromStatusMsg(t_msg_ptr);
}

// ----------------
// PUBLIC FUNCTIONS
// ----------------

rtask_msgs::StatusPtr rtask::commons::Status::toStatusMsg() const
{
  if (m_status == State::NULLVALUE) {
    return nullptr;
  }
  rtask_msgs::StatusPtr status = boost::make_shared<rtask_msgs::Status>();
  status->status = reverse_state_map.at(m_status);
  status->description = m_description;
  return status;
}

void rtask::commons::Status::setFromStatusMsg(const rtask_msgs::Status& t_msg)
{
  m_status = state_map.at(t_msg.status);
  m_description = t_msg.description;
}

void rtask::commons::Status::setFromStatusMsg(const rtask_msgs::StatusConstPtr t_msg_ptr)
{
  setFromStatusMsg(*t_msg_ptr);
}

void rtask::commons::Status::setStatus(const State t_status, const std::string& t_descr)
{
  m_status = t_status;
  m_description = t_descr;
}

void rtask::commons::Status::clear()
{
  m_description = "";
  m_status = State::NULLVALUE;
}
