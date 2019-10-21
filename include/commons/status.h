#ifndef rtask_commons_status_h
#define rtask_commons_status_h

#include "rtask_msgs/Status.h"

namespace rtask {
  namespace commons {
    enum class State
    {
      NULLVALUE = -1,
      UNKNOWN = 0,
      DEFINED = 1,
      RUNNING = 2,
      SUCCEDED = 3,
      FAILED = 4,
      TIMED_OUT = 5,
      CANCELED = 6
    };
    class Status
    {
    public:
      Status(const State t_status = State::NULLVALUE, const std::string t_descr = "");
      Status(const rtask_msgs::Status& t_msg);
      Status(const rtask_msgs::StatusConstPtr t_msg_ptr);
      ~Status() {}

      rtask_msgs::StatusPtr toStatusMsg() const;

      void setFromStatusMsg(const rtask_msgs::Status& t_msg);
      void setFromStatusMsg(const rtask_msgs::StatusConstPtr t_msg_ptr);

      void setStatus(const State t_status, const std::string& t_descr = "");
      void setDescription(const std::string& t_descr) { m_description = t_descr; }
      void clear();

      State getStatus() const { return m_status; }
      std::string getDescription() const { return m_description; }

      constexpr State toRTaskState(unsigned char t_state) const
      {
        if (state_map.count(t_state) != 1) {
          return State::NULLVALUE;
        }
        return state_map.at(t_state);
      }
      constexpr unsigned char toRTaskMsgStatus(State t_state) const { return reverse_state_map.at(t_state); }

    private:
      std::string m_description = "";
      State m_status = State::NULLVALUE;

      const std::map<unsigned char, State> state_map = {{rtask_msgs::Status::UNKNOWN, State::UNKNOWN},
                                                        {rtask_msgs::Status::DEFINED, State::DEFINED},
                                                        {rtask_msgs::Status::RUNNING, State::RUNNING},
                                                        {rtask_msgs::Status::SUCCEED, State::SUCCEDED},
                                                        {rtask_msgs::Status::FAILED, State::FAILED},
                                                        {rtask_msgs::Status::TIMED_OUT, State::TIMED_OUT},
                                                        {rtask_msgs::Status::CANCELED, State::CANCELED}};

      const std::map<State, unsigned char> reverse_state_map = {{State::UNKNOWN, rtask_msgs::Status::UNKNOWN},
                                                                {State::DEFINED, rtask_msgs::Status::DEFINED},
                                                                {State::RUNNING, rtask_msgs::Status::RUNNING},
                                                                {State::SUCCEDED, rtask_msgs::Status::SUCCEED},
                                                                {State::FAILED, rtask_msgs::Status::FAILED},
                                                                {State::TIMED_OUT, rtask_msgs::Status::TIMED_OUT},
                                                                {State::CANCELED, rtask_msgs::Status::CANCELED}};
    };

  } // namespace commons

} // namespace rtask

#endif
