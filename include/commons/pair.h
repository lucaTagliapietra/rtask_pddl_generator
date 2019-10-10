#ifndef rtask_commons_pair_h
#define rtask_commons_pair_h

#include <string>
#include <vector>

#include "rtask_msgs/Pair.h"

namespace rtask {
  namespace commons {

    class Pair
    {
    public:
      Pair() {}
      Pair(const std::string& t_capability, const std::vector<unsigned int>& t_ids = {});
      Pair(const rtask_msgs::Pair& t_msg);
      Pair(const rtask_msgs::PairConstPtr t_msg_ptr);

      ~Pair() {}

      rtask_msgs::PairPtr toPairMsg() const;

      void setFromPairMsg(const rtask_msgs::Pair& t_msg);
      void setFromPairMsg(const rtask_msgs::PairConstPtr t_msg_ptr);
      void setPair(const std::string& t_capability, const std::vector<unsigned int>& t_ids = {});
      void clear();

      std::string getCapability() const { return m_capability; }
      void getComponentIds(std::vector<unsigned int>& t_ids) const { t_ids = m_component_ids; }

      bool hasId(const unsigned int t_id);
      bool addId(const unsigned int t_id);

    private:
      std::string m_capability = "";
      std::vector<unsigned int> m_component_ids{};

      void setCapability(const std::string& t_capability) { m_capability = t_capability; }
      void setIds(const std::vector<unsigned int>& t_ids);
    };
  } // namespace commons
} // namespace rtask

#endif
