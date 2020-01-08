#include "commons/task.h"

#include "boost/make_shared.hpp"

// ------------
// CONSTRUCTORS
// ------------

rtask::commons::Task::Task(const unsigned int t_id,
                           const std::string t_name,
                           const std::string t_ref_frame,
                           const std::string t_description,
                           const ros::Duration t_timeout,
                           const std::string t_type,
                           const Status& t_status,
                           const std::vector<Capacity>& t_requirements,
                           const TaskDefinition& t_task_definition)
{
  setTask(t_id, t_name, t_ref_frame, t_description, t_timeout, t_type, t_status, t_requirements, t_task_definition);
}

rtask::commons::Task::Task(const rtask_msgs::Task& t_msg)
{
  setFromTaskMsg(t_msg);
}

rtask::commons::Task::Task(const rtask_msgs::TaskConstPtr t_msg_ptr)
{
  setFromTaskMsg(t_msg_ptr);
}

// ----------------
// PUBLIC FUNCTIONS
// ----------------

// ----------
// Task Level
// ----------

rtask_msgs::TaskPtr rtask::commons::Task::toTaskMsg() const
{
  rtask_msgs::TaskPtr t_msg_ptr = boost::make_shared<rtask_msgs::Task>();
  t_msg_ptr->id = m_params.id;
  t_msg_ptr->name = m_params.name;
  t_msg_ptr->header.frame_id = m_params.ref_frame;
  t_msg_ptr->description = m_params.description;
  t_msg_ptr->timeout = m_params.timeout;
  t_msg_ptr->type = m_params.type;
  t_msg_ptr->status = *(m_params.status.toStatusMsg());

  for (auto& r : m_params.requirements) {
    t_msg_ptr->requirements.push_back(*(r.second.toCapacityMsg()));
  }

  t_msg_ptr->task_definition = *(m_params.task_definition.toTaskDefinitionMsg());

  return t_msg_ptr;
}

void rtask::commons::Task::setFromTaskMsg(const rtask_msgs::Task& t_msg)
{
  m_params.id = t_msg.id;
  m_params.name = t_msg.name;
  m_params.ref_frame = t_msg.header.frame_id;
  m_params.description = t_msg.description;
  m_params.timeout = t_msg.timeout;
  m_params.type = t_msg.type;
  m_params.status.setFromStatusMsg(t_msg.status);

  for (auto& r : t_msg.requirements) {
    m_params.requirements[r.capability] = {};
    m_params.requirements[r.capability].setFromCapacityMsg(r);
  }

  m_params.task_definition.setFromTaskDefinitionMsg(t_msg.task_definition);
}

void rtask::commons::Task::setFromTaskMsg(const rtask_msgs::TaskConstPtr t_msg_ptr)
{
  setFromTaskMsg(*t_msg_ptr);
}

void rtask::commons::Task::setTask(const unsigned int t_id,
                                   const std::string t_name,
                                   const std::string t_ref_frame,
                                   const std::string t_description,
                                   const ros::Duration t_timeout,
                                   const std::string t_type,
                                   const Status t_status,
                                   const std::vector<Capacity> t_requirements,
                                   const TaskDefinition& t_task_definition)
{
  m_params.id = t_id;
  m_params.name = t_name;
  m_params.ref_frame = t_ref_frame;
  m_params.timeout = t_timeout;
  m_params.description = t_description;
  m_params.type = t_type;
  setStatus(t_status);
  for (auto& r : t_requirements) {
    m_params.requirements[r.getCapabilityName()] = {r};
  }
  setTaskDefinition(t_task_definition);
}

void rtask::commons::Task::setTask(const Task& t_task)
{
  m_params.id = t_task.getId();
  m_params.name = t_task.getName();
  m_params.ref_frame = t_task.getReferenceFrame();
  m_params.timeout = t_task.getTimeout();
  m_params.description = t_task.getDescription();
  m_params.type = t_task.getType();
  t_task.getStatus(m_params.status);
  std::vector<rtask::commons::Capacity> tmp;
  t_task.getRequirements(tmp);
  for (auto& r : tmp) {
    m_params.requirements[r.getCapabilityName()] = {r};
  }
  t_task.getTaskDefinition(m_params.task_definition);
}

void rtask::commons::Task::clear()
{
  m_params.clear();
}

void rtask::commons::Task::getStatus(Status& t_status) const
{
  t_status.setStatus(m_params.status.getStatus(), m_params.status.getDescription());
}

void rtask::commons::Task::getTaskDefinition(TaskDefinition& t_task_definition) const
{
  t_task_definition.setTaskDefinition(m_params.task_definition.getName(),
                                      m_params.task_definition.getDomainName(),
                                      m_params.task_definition.getEntities(),
                                      m_params.task_definition.getInitialState(),
                                      m_params.task_definition.getGoalState());
}

void rtask::commons::Task::getRequiredCapabilities(std::vector<std::string>& t_req_capabilities) const
{
  t_req_capabilities.reserve(m_params.requirements.size());
  for (auto it = m_params.requirements.begin(); it != m_params.requirements.end(); ++it) {
    t_req_capabilities.emplace_back(it->first);
  }
}

void rtask::commons::Task::getRequirements(std::vector<Capacity>& t_reqs) const
{
  t_reqs.reserve(m_params.requirements.size());
  for (auto it = m_params.requirements.begin(); it != m_params.requirements.end(); ++it) {
    t_reqs.emplace_back(it->second);
  }
}

// ------------
// Status Level
// ------------

void rtask::commons::Task::setStatus(const Status t_status)
{
  m_params.status.setStatus(t_status.getStatus(), t_status.getDescription());
}

void rtask::commons::Task::setStatusValue(const State t_state)
{
  m_params.status.setStatus(t_state, m_params.status.getDescription());
}

void rtask::commons::Task::setStatusDescription(const std::string& t_descr)
{
  m_params.status.setStatus(m_params.status.getStatus(), t_descr);
}

// --------------
// Capacity Level
// --------------
bool rtask::commons::Task::requiresCapacity(const std::string& t_capacity_name) const
{
  if (m_params.requirements.count(t_capacity_name) != 1) {
    return false;
  }
  return true;
}

bool rtask::commons::Task::removeRequiredCapacity(const std::string& t_capacity_name)
{
  if (!requiresCapacity(t_capacity_name)) {
    return false;
  }
  m_params.requirements.erase(t_capacity_name);
  return true;
}

bool rtask::commons::Task::getRequiredCapacity(const std::string& t_capacity_name, Capacity& t_capacity) const
{
  if (!requiresCapacity(t_capacity_name)) {
    return false;
  }
  t_capacity = {m_params.requirements.at(t_capacity_name)};
  return true;
}

bool rtask::commons::Task::getRequiredCapacityProperties(const std::string& t_capacity_name,
                                                         std::vector<Property>& t_props) const
{
  if (!requiresCapacity(t_capacity_name)) {
    return false;
  }
  m_params.requirements.at(t_capacity_name).getProperties(t_props);
  return true;
}

bool rtask::commons::Task::addRequiredCapacity(const Capacity& t_capacity)
{
  if (requiresCapacity(t_capacity.getCapabilityName())) {
    return false;
  }
  setRequiredCapacity(t_capacity);
  return true;
}

void rtask::commons::Task::setRequiredCapacity(const Capacity& t_capacity)
{
  m_params.requirements[t_capacity.getCapabilityName()] = {t_capacity};
}

// --------------
// Property Level
// --------------
bool rtask::commons::Task::requiresCapacityProperty(const std::string& t_capacity_name,
                                                    const std::string& t_prop_name) const
{
  if (!requiresCapacity(t_capacity_name)) {
    return false;
  }
  if (!m_params.requirements.at(t_capacity_name).hasProperty(t_prop_name)) {
    return false;
  }
  return true;
}

bool rtask::commons::Task::removeRequiredCapacityProperty(const std::string& t_capacity_name,
                                                          const std::string& t_prop_name)
{
  if (!requiresCapacity(t_capacity_name)) {
    return false;
  }
  return m_params.requirements[t_capacity_name].removeProperty(t_prop_name);
}

bool rtask::commons::Task::getRequiredCapacityProperty(const std::string& t_capacity_name,
                                                       const std::string& t_prop_name,
                                                       Property& t_prop) const
{
  if (!requiresCapacity(t_capacity_name)) {
    return false;
  }
  return m_params.requirements.at(t_capacity_name).getProperty(t_prop_name, t_prop);
}

bool rtask::commons::Task::addRequiredCapacityProperty(const std::string& t_capacity_name, const Property& t_prop)
{
  if (requiresCapacityProperty(t_capacity_name, t_prop.getName())) {
    return false;
  }

  if (!requiresCapacity(t_capacity_name)) {
    m_params.requirements[t_capacity_name] = {t_capacity_name};
  }
  return m_params.requirements[t_capacity_name].addProperty(t_prop);
}

void rtask::commons::Task::setRequiredCapacityProperty(const std::string& t_capacity_name, const Property& t_prop)
{
  {
    if (!requiresCapacity(t_capacity_name)) {
      m_params.requirements[t_capacity_name] = {t_capacity_name};
    }
    m_params.requirements[t_capacity_name].setProperty(t_prop);
  }
}

// ---------------------
// Task Definition level
// ---------------------

bool rtask::commons::Task::hasTaskDefinition(const TaskDefinition& t_task_definition) const
{
  return hasTaskDefinition(t_task_definition.getName(),
                           t_task_definition.getDomainName(),
                           t_task_definition.getEntities(),
                           t_task_definition.getInitialState(),
                           t_task_definition.getGoalState(),
                           t_task_definition.getTypingRequirement(),
                           t_task_definition.getEqualityRequirement(),
                           t_task_definition.getStripsRequirement());
}

bool rtask::commons::Task::hasTaskDefinition(const std::string& t_name,
                                             const std::string& t_domain_name,
                                             const std::vector<Entity>& t_entities,
                                             const std::vector<Command>& t_initial_state,
                                             const std::vector<Command>& t_goal_state,
                                             const bool t_req_typing,
                                             const bool t_req_equality,
                                             const bool t_req_strips) const
{
  if (m_params.task_definition.getName() == t_name && m_params.task_definition.getDomainName() == t_domain_name
      && m_params.task_definition.getEntities() == t_entities
      && m_params.task_definition.getInitialState() == t_initial_state
      && m_params.task_definition.getGoalState() == t_goal_state
      && m_params.task_definition.getTypingRequirement() == t_req_typing
      && m_params.task_definition.getEqualityRequirement() == t_req_equality
      && m_params.task_definition.getStripsRequirement() == t_req_strips) {
    return true;
  }
  return false;
}

void rtask::commons::Task::setTaskDefinition(const TaskDefinition t_task_definition)
{
  m_params.task_definition.setTaskDefinition(t_task_definition.getName(),
                                             t_task_definition.getDomainName(),
                                             t_task_definition.getEntities(),
                                             t_task_definition.getInitialState(),
                                             t_task_definition.getGoalState(),
                                             t_task_definition.getTypingRequirement(),
                                             t_task_definition.getEqualityRequirement(),
                                             t_task_definition.getStripsRequirement());
}

void rtask::commons::Task::setTaskDefinition(const std::string& t_name,
                                             const std::string& t_domain_name,
                                             const std::vector<Entity>& t_entities,
                                             const std::vector<Command>& t_initial_state,
                                             const std::vector<Command>& t_goal_state,
                                             const bool t_req_typing,
                                             const bool t_req_equality,
                                             const bool t_req_strips)
{
  m_params.task_definition.setTaskDefinition(
    t_name, t_domain_name, t_entities, t_initial_state, t_goal_state, t_req_typing, t_req_equality, t_req_strips);
}

bool rtask::commons::Task::removeTaskDefinition(const TaskDefinition& t_task_definition)
{

  if (!hasTaskDefinition(t_task_definition))
    return false;

  m_params.task_definition.clear();
  return true;
}

bool rtask::commons::Task::removeTaskDefinition(const std::string& t_name,
                                                const std::string& t_domain_name,
                                                const std::vector<Entity>& t_entities,
                                                const std::vector<Command>& t_initial_state,
                                                const std::vector<Command>& t_goal_state,
                                                const bool t_req_typing,
                                                const bool t_req_equality,
                                                const bool t_req_strips)
{
  if (!hasTaskDefinition(
        t_name, t_domain_name, t_entities, t_initial_state, t_goal_state, t_req_typing, t_req_equality, t_req_strips))
    return false;

  m_params.task_definition.clear();
  return true;
}
