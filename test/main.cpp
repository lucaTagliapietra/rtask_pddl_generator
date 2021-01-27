#include "pddl_generator/AndExpression.h"
#include "pddl_generator/Helpers.h"
#include "pddl_generator/LiteralExpression.h"
#include "pddl_generator/LiteralTerm.h"
#include "pddl_generator/NotExpression.h"

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
  auto not_xml = and_xml["and"][0]["not"];

  std::cout << "and_xml: " << and_xml << std::endl;
  std::cout << "and_xml type: " << and_xml.getType() << std::endl;
  std::cout << "lt_xml: " << lt_xml << std::endl;
  std::cout << "lt_xml type: " << lt_xml.getType() << std::endl;
  std::cout << "not_xml: " << not_xml << std::endl;
  std::cout << "not_xml type: " << not_xml.getType() << std::endl;

  std::cout << std::endl << std::endl;
  std::cout << "******** NOT ********" << std::endl;
  NotExpression not_expr(not_xml);
  std::cout << "not_expr:" << std::endl << not_expr << std::endl << std::endl;
  std::cout << "not_expr pddl:" << std::endl << not_expr.toPddl() << std::endl << std::endl;

  //  auto not_child = helpers::getAsChild(not_expr.getExpression());
  //  std::cout << not_child.type().name() << std::endl << std::endl;

  std::cout << std::endl << std::endl;
  std::cout << "******** AND ********" << std::endl;
  AndExpression and_expr(and_xml);
  std::cout << "and_expr:" << std::endl << and_expr << std::endl << std::endl;
  std::cout << "and_expr pddl:" << std::endl << and_expr.toPddl() << std::endl << std::endl;

  ros::shutdown();
}