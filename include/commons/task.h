#ifndef rtask_commons_task_h
#define rtask_commons_task_h

#include <limits>
#include <map>
#include <string>
#include <vector>

#include "ros/duration.h"
#include "rtask_msgs/Task.h"

#include "commons/capacity.h"
#include "commons/status.h"
#include "commons/taskDefinition.h"

namespace rtask {
  namespace commons {
    class Task
    {
    public:
      Task() {}
      Task(const unsigned int t_id,
           const std::string t_name,
           const std::string t_ref_frame = "",
           const std::string t_description = "",
           const ros::Duration t_timeout = {},
           const std::string t_type = {},
           const Status& t_status = {},
           const std::vector<Capacity>& t_requirements = {},
           const TaskDefinition& t_task_definition = {});

      Task(const rtask_msgs::Task& t_msg);
      Task(const rtask_msgs::TaskConstPtr t_msg_ptr);
      ~Task() {}

      rtask_msgs::TaskPtr toTaskMsg() const;

      void setFromTaskMsg(const rtask_msgs::Task& t_msg);
      void setFromTaskMsg(const rtask_msgs::TaskConstPtr t_msg_ptr);
      void setTask(const unsigned int t_id,
                   const std::string t_name,
                   const std::string t_ref_frame = "",
                   const std::string t_description = "",
                   const ros::Duration t_timeout = {},
                   const std::string t_type = "",
                   const Status t_status = {},
                   const std::vector<Capacity> t_requirements = {},
                   const TaskDefinition& t_task_definition = {});
      void setTask(const Task& t_task);
      void setReferenceFrame(const std::string& t_ref_frame) { m_params.ref_frame = t_ref_frame; }
      void setDescription(const std::string& t_value) { m_params.description = t_value; }
      void setTimeout(const ros::Duration t_timeout) { m_params.timeout = t_timeout; }
      void setType(const std::string& t_type) { m_params.type = t_type; }

      void clear();

      unsigned int getId() const { return m_params.id; }
      std::string getName() const { return m_params.name; }
      std::string getReferenceFrame() const { return m_params.ref_frame; }
      std::string getDescription() const { return m_params.description; }
      ros::Duration getTimeout() const { return m_params.timeout; }
      std::string getType() const { return m_params.type; }
      void getStatus(Status& t_status) const;
      void getRequiredCapabilities(std::vector<std::string>& t_req_capabilities) const;
      void getRequirements(std::vector<Capacity>& t_reqs) const;
      void getTaskDefinition(TaskDefinition& t_task_definition) const;

      // ------------
      // Status Level
      // ------------
      void setStatus(const Status t_status);
      void setStatusValue(const State t_state);
      void setStatusDescription(const std::string& t_descr);
      State getStatusValue() const { return m_params.status.getStatus(); }
      std::string getStatusDescription() const { return m_params.status.getDescription(); }

      // --------------
      // Capacity Level
      // --------------
      bool requiresCapacity(const std::string& t_capacity_name) const;
      bool removeRequiredCapacity(const std::string& t_capacity_name);
      bool getRequiredCapacity(const std::string& t_capacity_name, Capacity& t_capacity) const;
      bool getRequiredCapacityProperties(const std::string& t_capacity_name, std::vector<Property>& t_props) const;
      bool addRequiredCapacity(const Capacity& t_capacity);
      void setRequiredCapacity(const Capacity& t_capacity);

      // --------------
      // Property level
      // --------------
      bool requiresCapacityProperty(const std::string& t_capacity_name, const std::string& t_prop_name) const;
      bool removeRequiredCapacityProperty(const std::string& t_capacity_name, const std::string& t_prop_name);
      bool getRequiredCapacityProperty(const std::string& t_capacity_name,
                                       const std::string& t_prop_name,
                                       Property& t_prop) const;
      bool addRequiredCapacityProperty(const std::string& t_capacity_name, const Property& t_prop);
      void setRequiredCapacityProperty(const std::string& t_capacity_name, const Property& t_prop);

      // ---------------------
      // Task Definition level
      // ---------------------

      bool hasTaskDefinition(const TaskDefinition& t_task_definition) const;
      bool hasTaskDefinition(const std::string& t_name,
                             const std::string& t_domain_name,
                             const std::vector<Entity>& t_entities = {},
                             const std::vector<Command>& t_initial_state = {},
                             const std::vector<Command>& t_goal_state = {},
                             const bool t_req_typing = false,
                             const bool t_req_equality = false,
                             const bool t_req_strips = false) const;
      void setTaskDefinition(const TaskDefinition t_task_definition);
      void setTaskDefinition(const std::string& t_name,
                             const std::string& t_domain_name,
                             const std::vector<Entity>& t_entities = {},
                             const std::vector<Command>& t_initial_state = {},
                             const std::vector<Command>& t_goal_state = {},
                             const bool t_req_typing = false,
                             const bool t_req_equality = false,
                             const bool t_req_strips = false);
      bool removeTaskDefinition(const TaskDefinition& t_task_definition);

      bool removeTaskDefinition(const std::string& t_name,
                                const std::string& t_domain_name,
                                const std::vector<Entity>& t_entities = {},
                                const std::vector<Command>& t_initial_state = {},
                                const std::vector<Command>& t_goal_state = {},
                                const bool t_req_typing = false,
                                const bool t_req_equality = false,
                                const bool t_req_strips = false);

    private:
      struct Parameters
      {
        unsigned int id = std::numeric_limits<unsigned int>::quiet_NaN();
        std::string name = "";
        std::string ref_frame = "";
        std::string description = "";
        ros::Duration timeout{};
        std::string type = "";

        Status status{};

        std::map<std::string, Capacity> requirements{};

        TaskDefinition task_definition{};

        void clear()
        {
          id = std::numeric_limits<unsigned int>::quiet_NaN();
          name = "";
          ref_frame = "";
          description = "";
          timeout = {};
          type = "";
          status.clear();
          requirements.clear();
          task_definition.clear();
        }
      };
      Parameters m_params;
    };
  } // namespace commons
} // namespace rtask

#endif
