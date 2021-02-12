#include "pddl_generator/Problem.h"
#include "pddl_generator/Helpers.h"

#include <algorithm>

using namespace rtask::commons::pddl_generator;

Problem::Problem(const std::string& t_name,
                 const std::string& t_belonging_domain_name,
                 const LiteralTermVector& t_objects,
                 const LiteralExprVector& t_initial_state,
                 const LogicalExprPtr t_goal_ptr)
{
  set(t_name, t_belonging_domain_name, t_objects, t_initial_state, t_goal_ptr);
}

Problem::Problem(XmlRpc::XmlRpcValue& t_rpc_val)
{
  if (!(t_rpc_val.getType() == XmlRpc::XmlRpcValue::Type::TypeStruct
        && helpers::checkXmlRpcSanity("name", t_rpc_val, XmlRpc::XmlRpcValue::Type::TypeString, false))) {
    std::cerr << "Fatal: Invalid Problem Structure, should be a Struct with, at least, a name" << std::endl;
    exit(EXIT_FAILURE);
  }

  setName(static_cast<std::string>(t_rpc_val["name"]));

  if (helpers::hasValidXmlRpcTag("belonging_domain", t_rpc_val, XmlRpc::XmlRpcValue::Type::TypeString)) {
    setBelongingDomainName(static_cast<std::string>(t_rpc_val["belonging_domain"]));
  }

  if (helpers::hasValidXmlRpcTag("objects", t_rpc_val, XmlRpc::XmlRpcValue::Type::TypeArray)) {
    if (t_rpc_val["objects"].size() == 0) {
      std::cout << "Empty OBJECTS vector for current PROBLEM: " << name_ << std::endl;
    }
    else {
      LiteralTermVector objs;
      for (int i = 0; i < t_rpc_val["objects"].size(); ++i) {
        if (t_rpc_val["objects"][i].getType() != XmlRpc::XmlRpcValue::Type::TypeStruct) {
          std::cerr << "Invalid OBJECT " << i << " of current PROBLEM: " << name_ << std::endl;
          exit(EXIT_FAILURE);
        }
        objs.emplace_back(t_rpc_val["objects"][i]);
        objs.back().setIsAConstantTerm(true);
      }
      if (!setObjects(objs)) {
        std::cerr << "Errors in OBJECTS of current PROBLEM: " << name_ << std::endl;
        exit(EXIT_FAILURE);
      }
    }
  }

  if (helpers::hasValidXmlRpcTag("init", t_rpc_val, XmlRpc::XmlRpcValue::Type::TypeArray)
      && t_rpc_val["init"].size() != 0) {
    LiteralExprVector init_s;
    for (int i = 0; i < t_rpc_val["init"].size(); ++i) {
      init_s.emplace_back(t_rpc_val["init"][i]);
      init_s.back().setAsState();
    }
    if (!setInitialState(init_s)) {
      std::cerr << "Errors in INIT of current PROBLEM: " << name_ << std::endl;
      exit(EXIT_FAILURE);
    }
  }

  if (helpers::hasValidXmlRpcTag("goal", t_rpc_val, XmlRpc::XmlRpcValue::Type::TypeStruct)) {
    const auto& goal = helpers::getLogicalExprFromXmlRpc(t_rpc_val["goal"]);
    if (!goal) {
      std::cerr << "Fatal: Invalid GOAL of current PROBLEM: " << name_ << std::endl;
      exit(EXIT_FAILURE);
    }
    setGoal(goal);
  }
}

void Problem::clear()
{
  name_.clear();
  belonging_domain_name_.clear();
  objects_.clear();
  objects_map_.clear();
  initial_state_.clear();
  goal_.reset();
}

bool Problem::set(const std::string& t_name,
                  const std::string& t_belonging_domain_name,
                  const LiteralTermVector& t_objects,
                  const LiteralExprVector& t_initial_state,
                  const LogicalExprPtr t_goal_ptr)
{
  bool error_free = setName(t_name);
  error_free &= setBelongingDomainName(t_belonging_domain_name);
  error_free &= setObjects(t_objects);
  error_free &= setInitialState(t_initial_state);
  error_free &= setGoal(t_goal_ptr);
  return error_free;
}
bool Problem::setName(const std::string& t_name)
{
  if (t_name.empty()) {
    std::cerr << "Fatal: Empty NAME for current PROBLEM" << std::endl;
    return false;
  }
  name_ = std::move(t_name);
  return true;
}

bool Problem::setBelongingDomainName(const std::string& t_name)
{
  belonging_domain_name_ = std::move(t_name);
  return true;
}

bool Problem::setObjects(const LiteralTermVector& t_objs)
{
  bool error_free = true;
  objects_.clear();
  objects_map_.clear();

  for (auto& obj : t_objs) {
    if (belonging_domain_) {
      if (!obj.isValid(belonging_domain_->getTypes())) {
        std::cerr << "Validation Error: invalid OBJ: " << obj.getName() << " in current PROBLEM: " << name_
                  << std::endl;
        error_free = false;
        continue;
      }
    }

    // Two CONST cannot have the same name (regardless from type), otherwise it will be not known which one to use
    const auto& it = std::find_if(
      objects_.begin(), objects_.end(), [obj](const LiteralTerm& o) { return obj.getName() == o.getName(); });
    if (objects_map_.count(obj.getName()) > 0) {
      std::cerr << "Validation Error: OBJ: " << obj.getName() << " already defined in current PROBLEM: " << name_
                << std::endl;
      error_free = false;
      continue;
    }
    objects_.push_back(obj);
    objects_.back().setIsAConstantTerm(true);
    objects_map_.emplace(obj.getName(), obj.getType());
  }
  return error_free;
}

bool Problem::setInitialState(const LiteralExprVector& t_initial_state)
{
  bool error_free = true;

  for (const auto& le : t_initial_state) {
    if (belonging_domain_) {
      if (!le.isValid({}, belonging_domain_->getTypes(), objects_, belonging_domain_->getPredicates(), {})) {
        std::cerr << "Fatal: Invalid INIT " << name_ << " in current PROBLEM: " << name_ << std::endl;
        error_free = false;
        continue;
      }
    }

    const auto f =
      std::find_if(initial_state_.begin(), initial_state_.end(), [le](const auto& ole) { return operator==(le, ole); });
    if (f != initial_state_.end()) {
      std::cerr << "Fatal: INIT " << name_ << "already defined as: " << *f << " in current PROBLEM: " << name_
                << std::endl;
      error_free = false;
      continue;
    }
    initial_state_.push_back(le);
  }
  return error_free;
}

bool Problem::setGoal(LogicalExprPtr t_ptr)
{
  goal_ = t_ptr;
  return true;
}

Problem& Problem::operator=(const Problem& t_other)
{
  if (!set(t_other.getName(),
           t_other.getBelongingDomainName(),
           t_other.getObjects(),
           t_other.getInitialState(),
           t_other.getGoal())) {
    std::cerr << "PROBLEM: copy operator returned at least one error, check the log" << std::endl;
  }
  return *this;
}

std::string Problem::toPddl(bool t_typing, int t_pad_lv) const
{
  std::string out;
  auto pad_aligners_define = helpers::getPddlAligners(t_pad_lv);
  out += pad_aligners_define.second[0] + "(define" + pad_aligners_define.second[1];

  // increase PAD level by 1
  auto pad_aligners_prob = helpers::getPddlAligners(pad_aligners_define.first);
  out += pad_aligners_prob.second[0] + "(problem " + name_ + ")\n";

  if (!belonging_domain_name_.empty()) {
    out += pad_aligners_prob.second[0] + "(:domain " + belonging_domain_name_ + ")\n";
  }

  if (!objects_.empty()) {
    out += pad_aligners_prob.second[0] + "(:objects\n";
    auto pad_aligners_objs = helpers::getPddlAligners(pad_aligners_prob.first);
    UmapStrVecStr obj_groups;
    for (const auto& [n, t] : objects_map_) {
      obj_groups[t].push_back(n);
    }
    for (const auto& [t, ns] : obj_groups) {
      out += pad_aligners_objs.second[0];
      for (const auto& n : ns) {
        out += n + " ";
      }
      if (t_typing) {
        out += "- " + t;
      }
      else {
        out.pop_back();
      }
      out += "\n";
    }
    out.pop_back();
    out += pad_aligners_prob.second[2] + ")\n";
  }

  if (!initial_state_.empty()) {
    out += pad_aligners_prob.second[0] + "(:init" + pad_aligners_prob.second[1];
    for (const auto& le : initial_state_) {
      out += le.toPddl(t_typing, pad_aligners_prob.first) + pad_aligners_prob.second[1];
    }
    out.pop_back();
    out += pad_aligners_prob.second[2] + ")\n";
  }

  out += pad_aligners_prob.second[0] + "(:goal" + pad_aligners_prob.second[1];
  out += goal_->toPddl(t_typing, pad_aligners_prob.first);
  out += pad_aligners_prob.second[2] + ")";
  out += pad_aligners_define.second[2] + ")";
  return out;
}

bool rtask::commons::pddl_generator::operator==(const Problem& t_first, const Problem& t_second)
{

  if (t_first.getName() != t_second.getName()
      || t_first.getBelongingDomainName() != t_second.getBelongingDomainName()) {
    return false;
  }

  const auto& f_objs = t_first.getObjects();
  const auto& s_objs = t_second.getObjects();
  if (f_objs.size() != s_objs.size()) {
    return false;
  }
  for (const auto& f_obj : f_objs) {
    if (std::find(s_objs.begin(), s_objs.end(), f_obj) == s_objs.end()) {
      return false;
    }
  }

  const auto& f_inits = t_first.getInitialState();
  const auto& s_inits = t_second.getInitialState();
  if (f_inits.size() != s_inits.size()) {
    return false;
  }
  for (const auto& f_init : f_inits) {
    if (std::find(s_inits.begin(), s_inits.end(), f_init) == s_inits.end()) {
      return false;
    }
  }
  return (helpers::operator==(t_first.getGoal(), t_second.getGoal()));
}

bool rtask::commons::pddl_generator::operator!=(const Problem& t_first, const Problem& t_second)
{
  return !(t_first == t_second);
}

std::ostream& rtask::commons::pddl_generator::operator<<(std::ostream& t_out, const Problem& t_prob)
{
  return t_out << " ## PROBLEM ## " << std::endl << t_prob.toPddl(true, 0);
}

std::ostream& rtask::commons::pddl_generator::operator<<(std::ostream& t_out, std::shared_ptr<Problem> t_ptr)
{
  t_out << (t_ptr ? *t_ptr : Problem());
  return t_out;
}
