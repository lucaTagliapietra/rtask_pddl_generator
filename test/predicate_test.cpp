#include "commons/predicate.h"
#include <chrono>
#include <gtest/gtest.h>
#include <ros/ros.h>
#include <thread>

#include <iostream> // std::cout
#include <sstream> // std::stringstream

class PredicateTest : public ::testing::Test
{
public:
  std::string pred_name_{"predicate"};

  std::string param_name_0_{"param_0"};
  std::string param_name_1_{"param_1"};
  std::string param_name_2_{"param_2"};
  std::string param_name_3_{"param_3"};
  std::string param_name_in_{"param_in"};

  std::string param_type_0_{"type_0"};
  std::string param_type_1_{"type_1"};
  std::string param_type_in_{"type_in"};

  bool pred_typing_ = true;

  rtask::commons::Parameter param_0_{param_name_0_, param_type_0_};
  rtask::commons::Parameter param_1_{param_name_1_, param_type_1_};
  rtask::commons::Parameter param_2_{param_name_0_, param_type_0_};
  rtask::commons::Parameter param_3_{param_name_3_, param_type_1_};
  rtask::commons::Parameter param_in_{param_name_in_, param_type_in_};

  std::vector<rtask::commons::Parameter> pred_params_{param_0_, param_1_};

  std::vector<rtask::commons::Parameter> equal_pred_params_{param_0_, param_1_};
  std::vector<rtask::commons::Parameter> equiv_pred_params_{param_2_, param_3_};

  rtask::commons::Predicate pred_, pred_equal_, pred_equiv_;

  ros::NodeHandle m_nh;

  PredicateTest()
    : m_nh("~")
  {
    pred_ = {pred_name_, pred_params_};
    pred_equal_ = {pred_name_, equal_pred_params_};
    pred_equiv_ = {pred_name_, equiv_pred_params_};
  }

  ~PredicateTest() {}
};

TEST_F(PredicateTest, isEqual)
{
  ASSERT_TRUE(pred_.isEqual(pred_equal_));
}

TEST_F(PredicateTest, isEquivalent)
{
  ASSERT_TRUE(pred_.isEquivalent(pred_equiv_));
}

TEST_F(PredicateTest, fromXmlRpc)
{
  XmlRpc::XmlRpcValue pred_descr_;
  rtask::commons::Predicate pred_check_;

  m_nh.getParam("predicate", pred_descr_);

  pred_check_ = rtask::commons::Predicate(pred_descr_);

  ASSERT_TRUE(pred_check_.isEqual(pred_descr_));
}

TEST_F(PredicateTest, toMsg)
{
  rtask_msgs::Predicate pred_msg_;
  rtask::commons::Predicate pred_check_;

  pred_msg_ = pred_.toMsg();
  pred_check_ = {pred_msg_};

  ASSERT_TRUE(pred_check_.isEqual(pred_));
}

TEST_F(PredicateTest, set)
{
  rtask::commons::Predicate pred_check_;

  pred_check_.set(pred_name_, pred_params_);

  ASSERT_TRUE(pred_.isEqual(pred_check_));
}

TEST_F(PredicateTest, getName)
{
  ASSERT_EQ(pred_.getName(), pred_name_);
}

TEST_F(PredicateTest, getParams)
{
  std::vector<rtask::commons::Parameter> pred_params_check_ = pred_.getParameters();

  bool eq = true;

  // same order
  for (unsigned int i = 0; i < pred_params_check_.size() && eq; ++i) {
    eq &= pred_params_check_.at(i).isEqual(pred_.getParameters().at(i));
  }

  ASSERT_TRUE(eq);
}

TEST_F(PredicateTest, isValid)
{
  ASSERT_EQ(pred_.isValid(), true);
}

TEST_F(PredicateTest, getParameterList)
{
  std::vector<std::string> pred_param_list_check_;

  for (auto& p_ : pred_.getParameters()) {
    pred_param_list_check_.push_back(p_.getName());
  }

  ASSERT_EQ(pred_.getParameterList(), pred_param_list_check_);
}

TEST_F(PredicateTest, toPddl)
{
  std::string pred_pddl_check_ = "(" + pred_.getName();
  for (auto& p : pred_.getParameters())
    pred_pddl_check_ += " " + p.toPddl(pred_typing_);
  pred_pddl_check_ += ")";

  ASSERT_EQ(pred_.toPddl(pred_typing_), pred_pddl_check_);
}

TEST_F(PredicateTest, hasParameter)
{
  std::string pred_param_name_check_ = pred_.getParameterList().back();
  ASSERT_EQ(pred_.hasParameter(pred_param_name_check_), true);
}

TEST_F(PredicateTest, deleteParameter)
{
  std::vector<rtask::commons::Parameter> pred_param_check_ = pred_.getParameters();
  pred_param_check_.pop_back();

  pred_.deleteParameter(pred_.getParameterList().back());

  bool eq = true;

  // same order
  for (unsigned int i = 0; i < pred_param_check_.size() && eq; ++i) {
    eq &= pred_param_check_.at(i).isEqual(pred_.getParameters().at(i));
  }

  ASSERT_TRUE(eq);
}

TEST_F(PredicateTest, isParameterValid)
{
  bool valid_ = true;

  for (auto& p_ : pred_.getParameters())
    valid_ &= pred_.isParameterValid(p_.getName());

  ASSERT_EQ(valid_, true);
}

TEST_F(PredicateTest, getParameter)
{
  std::vector<rtask::commons::Parameter> pred_param_check_{};
  std::vector<std::string> param_names_check_ = pred_.getParameterList();

  for (auto& n_ : param_names_check_)
    pred_param_check_.push_back(pred_.getParameter(n_).second);

  bool eq = true;

  // same order
  for (unsigned int i = 0; i < pred_param_check_.size() && eq; ++i) {
    eq &= pred_param_check_.at(i).isEqual(pred_.getParameters().at(i));
  }

  ASSERT_TRUE(eq);
}

TEST_F(PredicateTest, setParameter)
{
  pred_.setParameter(param_in_.getName(), param_in_.getType());

  rtask::commons::Parameter pred_param_check_ = pred_.getParameter(param_in_.getName()).second;

  ASSERT_TRUE(param_in_.isEqual(pred_param_check_));
}

int main(int argc, char* argv[])
{
  testing::InitGoogleTest(&argc, argv);

  ros::init(argc, argv, "predicate_test_node");

  std::thread t([] {
    while (ros::ok())
      ros::spin();
  });
  auto res = RUN_ALL_TESTS();
  ros::shutdown();
  return res;
}
