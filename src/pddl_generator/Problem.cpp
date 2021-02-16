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
  if (belonging_domain_ && !hasValidUniqueObjects(t_objs)) {
    std::cerr << "Fatal: setObjects() failure. Invalid or duplicated object(s)" << std::endl;
    return false;
  }

  objects_.clear();
  objects_map_.clear();
  for (const auto& obj : t_objs) {
    if (hasObject(obj)) {
      std::cerr << "Fatal: setObjects() failure. Duplicated object(s): " << obj << std::endl;
      objects_.clear();
      objects_map_.clear();
      return false;
    }
    objects_.push_back(obj);
    objects_.back().setIsAConstantTerm(true);
    objects_map_.emplace(obj.getName(), obj.getType());
  }
  return true;
}

bool Problem::setInitialState(const LiteralExprVector& t_initial_state)
{
  if (belonging_domain_ && !hasValidUniqueInitialStates(t_initial_state)) {
    std::cerr << "Fatal: setInitialState() failure. Invalid or duplicated initial state" << std::endl;
    return false;
  }
  initial_state_.clear();
  for (const auto& is : t_initial_state) {
    if (hasInitialState(is)) {
      std::cerr << "Fatal: setInitialState() failure. Duplicated initial state: " << is << std::endl;
      initial_state_.clear();
      return false;
    }
    else {
      initial_state_.push_back(is);
    }
  }

  return true;
}

bool Problem::setGoal(LogicalExprPtr t_ptr)
{
  goal_ = t_ptr;
  return true;
}

bool Problem::setBelongingDomain(DomainPtr t_ptr)
{
  if (!t_ptr) {
    belonging_domain_.reset();
    std::cout << "Problem: Belonging Domain reset!" << std::endl;
    return true;
  }

  if (!belonging_domain_name_.empty() && t_ptr->getName() != belonging_domain_name_) {
    std::cout << "Problem: Belonging Domain name mismatch! Required: " << belonging_domain_name_
              << " Got: " << t_ptr->getName() << ". Doing nothing" << std::endl;
    return false;
  }

  belonging_domain_name_ = t_ptr->getName();
  belonging_domain_ = t_ptr;
  return true;
}

bool Problem::hasObject(const LiteralTerm& t_obj) const
{
  return std::find_if(objects_.begin(), objects_.end(), [t_obj](const auto& obj) { return t_obj == obj; })
         != objects_.end();
}

bool Problem::hasInitialState(const LiteralExpression& t_state) const
{
  return std::find_if(initial_state_.begin(),
                      initial_state_.end(),
                      [t_state, *this](const auto& s) { return helpers::operator==(s, t_state); })
         != initial_state_.end();
}

bool Problem::hasValidUniqueObjects() const
{
  return hasValidUniqueObjects(objects_);
}

bool Problem::hasValidUniqueInitialStates() const
{
  return hasValidUniqueInitialStates(initial_state_);
}

bool Problem::addObject(const LiteralTerm& t_obj)
{
  if (belonging_domain_ && !isObjectValid(t_obj)) {
    std::cerr << "Fatal: addObject() failure. Invalid object" << std::endl;
    return false;
  }

  if (hasObject(t_obj)) {
    std::cerr << "Fatal: addObject() failure. Duplicated object" << std::endl;
    return false;
  }

  objects_.emplace_back(t_obj);
  return true;
}

bool Problem::addInitialState(const LiteralExpression& t_state)
{
  if (belonging_domain_ && !isInitialStateValid(t_state)) {
    std::cerr << "Fatal: addInitialState() failure. Invalid InitialState" << std::endl;
    return false;
  }
  if (hasInitialState(t_state)) {
    std::cerr << "Fatal: addInitialState() failure. Duplicated InitialState" << std::endl;
    return false;
  }

  initial_state_.emplace_back(t_state);
  return true;
}

bool Problem::hasValidGoal() const
{
  if (!belonging_domain_) {
    return false;
  }
  return helpers::isValid(goal_,
                          objects_map_,
                          belonging_domain_->getTypes(),
                          belonging_domain_->getConstants(),
                          belonging_domain_->getPredicates(),
                          belonging_domain_->getTimeless(),
                          true);
}

bool Problem::hasValidBelongingDomain() const
{
  if (!belonging_domain_) {
    return false;
  }
  return belonging_domain_->isValid();
}

bool Problem::checkNameValidity(const std::string& t_name) const
{
  if (t_name.empty()) {
    std::cerr << "Fatal: Empty name for current PROBLEM" << std::endl;
    return false;
  }
  return true;
}

bool Problem::checkBelongingDomainNameValidity(const std::string& t_name) const
{
  if (t_name.empty()) {
    std::cerr << "Fatal: Empty BelongingDomainName for current PROBLEM" << std::endl;
    return false;
  }
  return true;
}

bool Problem::isObjectValid(const LiteralTerm& t_obj) const
{
  if (!belonging_domain_) {
    std::cerr << "Validation Error: BELONGING DOMAIN not set, unable to validate" << std::endl;
    return false;
  }

  return t_obj.isValid(belonging_domain_->getTypes());
}

bool Problem::hasValidUniqueObjects(const LiteralTermVector& t_objs) const
{
  int objs_idx = 0, lambda_idx = 0;
  for (const auto& obj : t_objs) {
    if (!isObjectValid(obj)) {
      return false;
    }

    lambda_idx = 0;
    auto lambda = [obj, objs_idx, &lambda_idx](const auto& o) { return (lambda_idx++ != objs_idx && obj == o); };

    if (std::find_if(t_objs.begin(), t_objs.end(), lambda) != t_objs.end()) {
      return false;
    }
    ++objs_idx;
  }
  return true;
}

bool Problem::isInitialStateValid(const LiteralExpression& t_state) const
{
  if (!belonging_domain_) {
    std::cerr << "Validation Error: BELONGING DOMAIN not set, unable to validate" << std::endl;
    return false;
  }

  return t_state.isValid(objects_map_,
                         belonging_domain_->getTypes(),
                         belonging_domain_->getConstants(),
                         belonging_domain_->getPredicates(),
                         belonging_domain_->getTimeless());
}

bool Problem::hasValidUniqueInitialStates(const LiteralExprVector& t_states) const
{
  int st_idx = 0, lambda_idx = 0;
  for (const auto& st : t_states) {
    if (!isInitialStateValid(st)) {
      return false;
    }

    lambda_idx = 0;
    auto lambda = [st, *this, st_idx, &lambda_idx](const auto& s) {
      return (lambda_idx++ != st_idx && helpers::operator==(s, st));
    };

    if (std::find_if(t_states.begin(), t_states.end(), lambda) != t_states.end()) {
      return false;
    }
    ++st_idx;
  }
  return true;
}

bool Problem::isValid() const
{
  bool is_valid = true;
  is_valid &= checkNameValidity(name_);
  is_valid &= checkBelongingDomainNameValidity(belonging_domain_name_);
  is_valid &= hasValidUniqueObjects(objects_);
  is_valid &= hasValidUniqueInitialStates(initial_state_);
  is_valid &= hasValidGoal();
  is_valid &= hasValidBelongingDomain();
  return is_valid;
}

bool Problem::isEquivalentTo(const Problem& t_other) const
{
  // TODO: to be properly implemented
  return operator==(*this, t_other);
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
