#include "commons/pair.h"

// ------------
// CONSTRUCTORS
// ------------
rtask::commons::Pair::Pair(const std::string& t_capability, const std::vector<unsigned int>& t_ids)
{
  setPair(t_capability, t_ids);
}

rtask::commons::Pair::Pair(const rtask_msgs::Pair& t_msg)
{
  setFromPairMsg(t_msg);
}

rtask::commons::Pair::Pair(const rtask_msgs::PairConstPtr t_msg_ptr)
{
  setFromPairMsg(t_msg_ptr);
}

// ----------------
// PUBLIC FUNCTIONS
// ----------------

rtask_msgs::PairPtr rtask::commons::Pair::toPairMsg() const
{
  rtask_msgs::PairPtr a_msg{};
  a_msg->capability = m_capability;
  a_msg->component_ids = m_component_ids;
  return a_msg;
}

void rtask::commons::Pair::setFromPairMsg(const rtask_msgs::Pair& t_msg)
{
  m_capability = t_msg.capability;
  m_component_ids.resize(t_msg.component_ids.size());
  m_component_ids = t_msg.component_ids;
}

void rtask::commons::Pair::setFromPairMsg(const rtask_msgs::PairConstPtr t_msg_ptr)
{
  setFromPairMsg(*t_msg_ptr);
}

void rtask::commons::Pair::setPair(const std::string& t_capability, const std::vector<unsigned int>& t_ids)
{
  m_capability = t_capability;
  m_component_ids = t_ids;
}

void rtask::commons::Pair::clear()
{
  m_capability = "";
  m_component_ids.clear();
}

bool rtask::commons::Pair::hasId(const unsigned int t_id)
{
  auto it = std::find(m_component_ids.begin(), m_component_ids.end(), t_id);
  if (it == m_component_ids.end()) {
    return false;
  }
  return true;
}

bool rtask::commons::Pair::addId(const unsigned int t_id)
{
  if (hasId(t_id)) {
    return false;
  }
  m_component_ids.push_back(t_id);
  return true;
}

// -----------------
// PRIVATE FUNCTIONS
// -----------------

void rtask::commons::Pair::setIds(const std::vector<unsigned int>& t_ids)
{
  m_component_ids.clear();
  m_component_ids.reserve(t_ids.size());
  for (auto i : t_ids) {
    m_component_ids.push_back(i);
  }
}
