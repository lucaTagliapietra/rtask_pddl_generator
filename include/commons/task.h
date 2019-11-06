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

namespace rtask {
  namespace commons {
    class Task
    {
    public:
      Task() {}
      Task(const unsigned int t_id,
           const std::string t_name,
           const std::string t_ref_frame = "",
           const std::vector<Capacity> t_capacities = {},
           const std::string t_description = "",
           const ros::Duration t_timeout = {},
           const Status t_status = {});
      Task(const rtask_msgs::Task& t_msg);
      Task(const rtask_msgs::TaskConstPtr t_msg_ptr);
      ~Task() {}

      rtask_msgs::TaskPtr toTaskMsg() const;

      void setFromTaskMsg(const rtask_msgs::Task& t_msg);
      void setFromTaskMsg(const rtask_msgs::TaskConstPtr t_msg_ptr);
      void setTask(const unsigned int t_id,
                   const std::string t_name,
                   const std::string t_ref_frame = "",
                   const std::vector<Capacity> t_requirements = {},
                   const std::string t_description = "",
                   const ros::Duration t_timeout = {},
                   const Status t_status = {});
      void setTask(const Task& t_task);
      void setDescription(const std::string& t_value) { m_params.description = t_value; }
      void setTimeout(const ros::Duration t_timeout) { m_params.timeout = t_timeout; }
      void setReferenceFrame(const std::string& t_ref_frame) { m_params.ref_frame = t_ref_frame; }
      void setStatus(const Status t_status);
      void clear();

      unsigned int getId() const { return m_params.id; }
      std::string getName() const { return m_params.name; }
      std::string getReferenceFrame() const { return m_params.ref_frame; }
      std::string getDescription() const { return m_params.description; }
      ros::Duration getTimeout() const { return m_params.timeout; }
      void getStatus(Status& t_status) const;
      void getRequiredCapabilities(std::vector<std::string>& t_req_capabilities) const;
      void getRequirements(std::vector<Capacity>& t_reqs) const;

      // ------------
      // Status Level
      // ------------
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

    private:
      struct Parameters
      {
        unsigned int id = std::numeric_limits<unsigned int>::quiet_NaN();
        std::string name = "";
        std::string description = "";
        std::string ref_frame = "";
        ros::Duration timeout{};

        Status status{};

        std::map<std::string, Capacity> requirements{};

        void clear()
        {
          id = std::numeric_limits<unsigned int>::quiet_NaN();
          name = "";
          ref_frame = "";
          description = "";
          timeout = {};
          status.clear();
          requirements.clear();
        }
      };
      Parameters m_params;
    };
  } // namespace commons
} // namespace rtask

#endif
