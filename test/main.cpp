#include "pddl_generator/AndExpression.h"
#include "pddl_generator/ExistsExpression.h"
#include "pddl_generator/ForAllExpression.h"
#include "pddl_generator/Helpers.h"
#include "pddl_generator/LiteralExpression.h"
#include "pddl_generator/LiteralTerm.h"
#include "pddl_generator/NotExpression.h"
#include "pddl_generator/NumericalFunction.h"
#include "pddl_generator/NumericalTerm.h"
#include "pddl_generator/OrExpression.h"
#include "pddl_generator/WhenExpression.h"

#include <ros/ros.h>

using namespace rtask::commons::pddl_generator;

int main(int argc, char* argv[])
{
  std::string node_name = "rtask_test";
  ros::init(argc, argv, node_name);
  ros::NodeHandle nh("~");

  XmlRpc::XmlRpcValue xml;
  nh.getParam("domain", xml);

  //  std::cout << action << std::endl << std::endl;
  //  std::cout << and_expr << std::endl << std::endl;
  //  std::cout << lt_xml << std::endl << std::endl;

  //  auto be_type = helpers::getBooleanExprTypeFromXmlRpc(and_expr);
  //  std::cout << static_cast<int>(be_type) << " (expect 3)" << std::endl;

  //  LiteralTerm lt(lt_xml);
  //  std::cout << lt << std::endl;
  //  std::shared_ptr<Term> t = std::make_shared<LiteralTerm>(lt);
  //  auto test = helpers::getAsChild(t);
  //  std::cout << test.type().name() << std::endl;
  //  std::cout << *std::any_cast<std::shared_ptr<LiteralTerm>>(test).get() << std::endl;

  auto and_xml = xml["actions"][0]["preconditions"]["and"][0]["or"][0]["not"];
  auto lt_xml = xml["actions"][0]["params"][0];
  auto not_xml = and_xml["and"][0];
  auto or_xml = xml["actions"][0]["preconditions"]["and"][0]["or"][1];
  auto when_xml = xml["actions"][0]["effects"]["and"][0];
  auto exists_xml = xml["actions"][0]["preconditions"]["and"][0]["or"][3];
  auto forall_xml = xml["actions"][0]["preconditions"]["and"][0]["or"][4];
  auto num_fnc_xml = xml["actions"][0]["preconditions"]["and"][3]["compare"]["lhs"]["num_op"]["lhs"];
  auto num_val_xml = xml["actions"][0]["preconditions"]["and"][3]["compare"]["lhs"]["num_op"]["rhs"];

  std::cout << "and_xml: " << and_xml << std::endl;
  std::cout << "and_xml type: " << and_xml.getType() << std::endl;
  std::cout << "lt_xml: " << lt_xml << std::endl;
  std::cout << "lt_xml type: " << lt_xml.getType() << std::endl;
  std::cout << "not_xml: " << not_xml << std::endl;
  std::cout << "not_xml type: " << not_xml.getType() << std::endl;
  std::cout << "or_xml: " << or_xml << std::endl;
  std::cout << "or_xml type: " << or_xml.getType() << std::endl;
  std::cout << "when_xml: " << when_xml << std::endl;
  std::cout << "when_xml type: " << when_xml.getType() << std::endl;

  std::cout << std::endl << std::endl;
  std::cout << "******** NOT ********" << std::endl;
  NotExpression not_expr(not_xml);
  //  std::cout << "not_expr:" << std::endl << not_expr << std::endl << std::endl;
  std::cout << "not_expr pddl:" << std::endl << not_expr.toPddl() << std::endl << std::endl;
  std::cout << "not_expr pddl:" << std::endl << not_expr.toPddl(true, -1) << std::endl << std::endl;

  //  auto not_child = helpers::getAsChild(not_expr.getExpression());
  //  std::cout << not_child.type().name() << std::endl << std::endl;

  std::cout << std::endl << std::endl;
  std::cout << "******** AND ********" << std::endl;
  AndExpression and_expr(and_xml);
  //  std::cout << "and_expr:" << std::endl << and_expr << std::endl << std::endl;
  std::cout << "and_expr pddl:" << std::endl << and_expr.toPddl() << std::endl << std::endl;
  std::cout << "and_expr pddl:" << std::endl << and_expr.toPddl(true, -1) << std::endl << std::endl;

  std::cout << std::endl << std::endl;
  std::cout << "******** OR ********" << std::endl;
  OrExpression or_expr(or_xml);
  //  std::cout << "or_expr:" << std::endl << or_expr << std::endl << std::endl;
  std::cout << "or_expr pddl:" << std::endl << or_expr.toPddl() << std::endl << std::endl;
  std::cout << "or_expr pddl:" << std::endl << or_expr.toPddl(true, -1) << std::endl << std::endl;

  std::cout << std::endl << std::endl;
  std::cout << "******** WHEN ********" << std::endl;
  WhenExpression when_expr(when_xml);
  //  std::cout << "when_expr:" << std::endl << when_expr << std::endl << std::endl;
  std::cout << "when_expr pddl:" << std::endl << when_expr.toPddl() << std::endl << std::endl;
  std::cout << "when_expr pddl:" << std::endl << when_expr.toPddl(true, -1) << std::endl << std::endl;

  std::cout << std::endl << std::endl;
  std::cout << "******** EXISTS ********" << std::endl;
  ExistsExpression exists_expr(exists_xml);
  //  std::cout << "when_expr:" << std::endl << when_expr << std::endl << std::endl;
  std::cout << "exists_expr pddl:" << std::endl << exists_expr.toPddl() << std::endl << std::endl;
  std::cout << "exists_expr pddl:" << std::endl << exists_expr.toPddl(true, -1) << std::endl << std::endl;

  std::cout << std::endl << std::endl;
  std::cout << "******** FORALL ********" << std::endl;
  ForAllExpression forall_expr(forall_xml);
  //  std::cout << "when_expr:" << std::endl << when_expr << std::endl << std::endl;
  std::cout << "forall_expr pddl:" << std::endl << forall_expr.toPddl() << std::endl << std::endl;
  std::cout << "forall_expr pddl:" << std::endl << forall_expr.toPddl(true, -1) << std::endl << std::endl;

  std::cout << std::endl << std::endl;
  std::cout << "******** NUMERICAL_FUNCTION ********" << std::endl;
  NumericalFunction numerical_fnc(num_fnc_xml);
  //  std::cout << "when_expr:" << std::endl << when_expr << std::endl << std::endl;
  std::cout << "numerical_fnc pddl:" << std::endl << numerical_fnc.toPddl() << std::endl << std::endl;
  std::cout << "numerical_fnc pddl:" << std::endl << numerical_fnc.toPddl(true, -1) << std::endl << std::endl;

  ros::shutdown();
}
