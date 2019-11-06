#ifndef rtask_commons_mission_h
#define rtask_commons_mission_h

#include <limits>
#include <map>
#include <string>
#include <vector>

#include "commons/task.h"
#include "rtask_msgs/Mission.h"

namespace rtask {
  namespace commons {
    class Mission
    {
    public:
      Mission() {}

      Mission(const unsigned int t_id,
              const std::string t_name,
              const std::string t_ref_frame = "",
              const std::vector<Task> t_tasks = {},
              const std::string t_description = "",
              const ros::Duration t_timeout = {},
              const Status t_status = {});
      Mission(const rtask_msgs::Mission& t_msg);
      Mission(const rtask_msgs::MissionConstPtr t_msg_ptr);
      ~Mission() {}

      rtask_msgs::MissionPtr toMissionMsg() const;

      void setFromMissionMsg(const rtask_msgs::Mission& t_msg);
      void setFromMissionMsg(const rtask_msgs::MissionConstPtr t_msg_ptr);
      void setMission(const unsigned int t_id,
                      const std::string t_name,
                      const std::string t_ref_frame = "",
                      const std::vector<Task> t_tasks = {},
                      const std::string t_description = "",
                      const ros::Duration t_timeout = {},
                      const Status t_status = {});
      void setReferenceFrame(const std::string& t_ref_frame) { m_params.ref_frame = t_ref_frame; }
      void setDescription(const std::string& t_value) { m_params.description = t_value; }
      void setTimeout(const ros::Duration t_timeout) { m_params.timeout = t_timeout; }
      void setStatus(const Status t_status);
      void clear();

      unsigned int getId() const { return m_params.id; }
      std::string getName() const { return m_params.name; }
      std::string getReferenceFrame() const { return m_params.ref_frame; }
      std::string getDescription() const { return m_params.description; }
      ros::Duration getTimeout() const { return m_params.timeout; }
      void getStatus(Status& t_status) const;
      void getTaskNames(std::vector<std::string>& t_task_names) const;
      void getTasks(std::vector<Task>& t_tasks) const;

      // ------------
      // Status Level
      // ------------
      void setStatusValue(const State t_state);
      void setStatusDescription(std::string& t_descr);
      State getStatusValue() const { return m_params.status.getStatus(); }
      std::string getStatusDescription() const { return m_params.status.getDescription(); }

      // ----------
      // Task Level
      // ----------
      bool hasTask(const std::string& t_task_name) const;
      bool removeTask(const std::string& t_task_name);
      bool getTask(const std::string& t_task_name, Task& t_task) const;
      bool addTask(const Task& t_task);
      void setTask(const Task& t_task);

      bool getTaskId(const std::string& t_task_name, unsigned int& t_id) const;
      bool getTaskReferenceFrame(const std::string& t_task_name, std::string& t_ref_frame) const;
      bool getTaskDescription(const std::string& t_task_name, std::string& t_descr) const;
      bool getTaskTimeout(const std::string& t_task_name, ros::Duration& t_timeout) const;
      bool getTaskStatus(const std::string& t_task_name, Status& t_status) const;
      bool getTaskRequiredCapabilities(const std::string& t_task_name,
                                       std::vector<std::string>& t_req_capabilities) const;
      bool getTaskRequirements(const std::string& t_task_name, std::vector<Capacity> t_reqs) const;

      bool setTaskReferenceFrame(const std::string& t_task_name, const std::string& t_ref_frame);
      bool setTaskStatus(const std::string& t_task_name, const Status& t_status);
      bool setTaskStatusValue(const std::string& t_task_name, const State& t_state);
      bool setTaskStatusDescription(const std::string& t_task_name, const std::string& t_status_decr);
      bool setTaskDescription(const std::string& t_task_name, const std::string& t_task_descr);
      bool setTaskTimeout(const std::string& t_task_name, const ros::Duration& t_timeout);

      // --------------
      // Capacity Level
      // --------------
      bool requiresTaskCapacity(const std::string& t_task_name, const std::string& t_capacity_name) const;
      bool removeTaskRequiredCapacity(const std::string& t_task_name, const std::string& t_capacity_name);
      bool getRequiredTaskCapacity(const std::string& t_task_name,
                                   const std::string& t_capacity_name,
                                   Capacity& t_capacity) const;
      bool getRequiredTaskCapacityProperties(const std::string& t_task_name,
                                             const std::string& t_capacity_name,
                                             std::vector<Property>& t_props) const;
      bool addRequiredTaskCapacity(const std::string& t_task_name, const Capacity& t_capacity);
      bool setRequiredTaskCapacity(const std::string& t_task_name, const Capacity& t_capacity);

      // --------------
      // Property level
      // --------------
      bool requiresTaskCapacityProperty(const std::string& t_task_name,
                                        const std::string& t_capacity_name,
                                        const std::string& t_prop_name) const;
      bool removeRequiredTaskCapacityProperty(const std::string& t_task_name,
                                              const std::string& t_capacity_name,
                                              const std::string& t_prop_name);
      bool getRequiredTaskCapacityProperty(const std::string& t_task_name,
                                           const std::string& t_capacity_name,
                                           const std::string& t_prop_name,
                                           Property& t_prop) const;
      bool addRequiredTaskCapacityProperty(const std::string& t_task_name,
                                           const std::string& t_capacity_name,
                                           const Property& t_prop);
      bool setRequiredTaskCapacityProperty(const std::string& t_task_name,
                                           const std::string& t_capacity_name,
                                           const Property& t_prop);

    private:
      struct Parameters
      {
        unsigned int id = std::numeric_limits<unsigned int>::quiet_NaN();
        std::string name = "";
        std::string ref_frame = "";
        std::string description = "";
        ros::Duration timeout{};

        Status status{};

        std::map<std::string, Task> tasks{};

        void clear()
        {
          id = std::numeric_limits<unsigned int>::quiet_NaN();
          name = "";
          ref_frame = "";
          description = "";
          timeout = {};
          status.clear();
          tasks.clear();
        }
      };
      Parameters m_params;
    };
  } // namespace commons
} // namespace rtask

#endif
