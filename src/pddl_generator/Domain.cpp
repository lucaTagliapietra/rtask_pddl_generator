#include "pddl_generator/Domain.h"
#include "pddl_generator/Helpers.h"

#include <algorithm>

using namespace rtask::commons::pddl_generator;

Domain::Domain(const std::string& t_name,
               const std::string& t_extends_domain_name,
               const std::vector<std::string>& t_requirements,
               const UmapStrStr& t_types,
               const std::vector<LiteralTerm>& t_constants,
               const std::vector<Predicate>& t_predicates,
               const std::vector<LiteralExpression>& t_timeless,
               const std::vector<Action>& t_actions)
{
  set(t_name, t_extends_domain_name, t_requirements, t_types, t_constants, t_predicates, t_timeless, t_actions);
}

Domain::Domain(XmlRpc::XmlRpcValue& t_rpc_val)
{
  if (!(t_rpc_val.getType() == XmlRpc::XmlRpcValue::Type::TypeStruct
        && helpers::checkXmlRpcSanity("name", t_rpc_val, XmlRpc::XmlRpcValue::Type::TypeString, false))) {
    std::cerr << "Fatal: Invalid Domain Structure, should be a Struct with, at least, a name" << std::endl;
    exit(EXIT_FAILURE);
  }

  setName(static_cast<std::string>(t_rpc_val["name"]));

  if (helpers::hasValidXmlRpcTag("extends", t_rpc_val, XmlRpc::XmlRpcValue::Type::TypeString)) {
    setExtendedDomainName(static_cast<std::string>(t_rpc_val["extends"]));
  }

  if (helpers::hasValidXmlRpcTag("requirements", t_rpc_val, XmlRpc::XmlRpcValue::Type::TypeArray)) {
    if (t_rpc_val["requirements"].size() == 0) {
      std::cout << "Empty REQUIREMENTS vector for current DOMAIN: " << name_ << std::endl;
    }
    else {
      std::vector<std::string> reqs;
      for (int i = 0; i < t_rpc_val["requirements"].size(); ++i) {
        if (t_rpc_val["requirements"][i].getType() != XmlRpc::XmlRpcValue::Type::TypeString) {
          std::cerr << "Invalid REQUIREMENTS " << i << " of current DOMAIN: " << name_ << std::endl;
          exit(EXIT_FAILURE);
        }
        reqs.emplace_back(static_cast<std::string>(t_rpc_val["requirements"][i]));
      }

      if (!setRequirements(reqs)) {
        std::cerr << "Errors in REQUIREMENTS of current DOMAIN: " << name_ << std::endl;
        exit(EXIT_FAILURE);
      }
    }
  }

  if (helpers::hasValidXmlRpcTag("types", t_rpc_val, XmlRpc::XmlRpcValue::Type::TypeArray)) {
    if (t_rpc_val["types"].size() == 0) {
      std::cout << "Empty TYPES declaration for current DOMAIN: " << name_ << std::endl;
    }
    else {
      UmapStrStr types;
      for (int i = 0; i < t_rpc_val["types"].size(); ++i) {
        if (t_rpc_val["types"][i].getType() != XmlRpc::XmlRpcValue::Type::TypeStruct
            || !helpers::hasValidXmlRpcTag("type_names", t_rpc_val["types"][i], XmlRpc::XmlRpcValue::Type::TypeArray)
            || t_rpc_val["types"][i]["type_names"].size() == 0) {
          std::cerr << "Invalid TYPES " << i << " of current DOMAIN: " << name_ << std::endl;
          exit(EXIT_FAILURE);
        }
        else {
          for (int k = 0; k < t_rpc_val["types"][i]["type_names"].size(); ++k) {
            if (t_rpc_val["types"][i]["type_names"][k].getType() != XmlRpc::XmlRpcValue::Type::TypeString) {
              std::cerr << "Invalid TYPE [" << i << "," << k << "] of current DOMAIN: " << name_ << std::endl;
            }
            if (!helpers::hasValidXmlRpcTag(
                  "parent_type", t_rpc_val["types"][i], XmlRpc::XmlRpcValue::Type::TypeString)) {
              types.emplace(static_cast<std::string>(t_rpc_val["types"][i]["type_names"][k]), "object");
            }
            else {
              types.emplace(static_cast<std::string>(t_rpc_val["types"][i]["type_names"][k]),
                            t_rpc_val["types"][i]["parent_type"]);
            }
          }
        }
      }
      if (!setTypes(types)) {
        std::cerr << "Errors in TYPES of current DOMAIN: " << name_ << std::endl;
        exit(EXIT_FAILURE);
      }
    }
  }

  if (helpers::hasValidXmlRpcTag("constants", t_rpc_val, XmlRpc::XmlRpcValue::Type::TypeArray)
      && t_rpc_val["constants"].size() != 0) {
    std::vector<LiteralTerm> constants;
    for (int i = 0; i < t_rpc_val["constants"].size(); ++i) {
      constants.emplace_back(t_rpc_val["constants"][i]);
      constants.back().setIsAConstantTerm(true);
    }
    if (!setConstants(constants)) {
      std::cerr << "Errors in CONSTANTS of current DOMAIN: " << name_ << std::endl;
      exit(EXIT_FAILURE);
    }
  }

  if (helpers::hasValidXmlRpcTag("predicates", t_rpc_val, XmlRpc::XmlRpcValue::Type::TypeArray)
      && t_rpc_val["predicates"].size() != 0) {
    std::vector<Predicate> preds;
    for (int i = 0; i < t_rpc_val["predicates"].size(); ++i) {
      preds.emplace_back(t_rpc_val["predicates"][i]);
    }
    if (!setPredicates(preds)) {
      std::cerr << "Errors in PREDICATES of current DOMAIN: " << name_ << std::endl;
      exit(EXIT_FAILURE);
    }
  }

  if (helpers::hasValidXmlRpcTag("timeless", t_rpc_val, XmlRpc::XmlRpcValue::Type::TypeArray)
      && t_rpc_val["timeless"].size() != 0) {
    std::vector<LiteralExpression> timeless;
    for (int i = 0; i < t_rpc_val["timeless"].size(); ++i) {
      timeless.emplace_back(t_rpc_val["timeless"][i]);
    }
    if (!setTimeless(timeless)) {
      std::cerr << "Errors in TIMELESS of current DOMAIN: " << name_ << std::endl;
      exit(EXIT_FAILURE);
    }
  }

  if (helpers::hasValidXmlRpcTag("actions", t_rpc_val, XmlRpc::XmlRpcValue::Type::TypeArray)
      && t_rpc_val["actions"].size() != 0) {
    std::vector<Action> actions;
    for (int i = 0; i < t_rpc_val["actions"].size(); ++i) {
      actions.emplace_back(t_rpc_val["actions"][i]);
    }
    if (!setActions(actions)) {
      std::cerr << "Errors in ACTIONS of current DOMAIN: " << name_ << std::endl;
      //      exit(EXIT_FAILURE);
    }
  }
}

void Domain::clear()
{
  name_.clear();
  extends_domain_name_.clear();
  requirements_.clear();
  types_.clear();
  constants_.clear();
  predicates_.clear();
  timeless_.clear();
  actions_.clear();
}

bool Domain::set(const std::string& t_name,
                 const std::string& t_extends_domain_name,
                 const std::vector<std::string>& t_requirements,
                 const UmapStrStr& t_types,
                 const std::vector<LiteralTerm>& t_constants,
                 const std::vector<Predicate>& t_predicates,
                 const std::vector<LiteralExpression>& t_timeless,
                 const std::vector<Action>& t_actions)
{
  bool error_free = setName(t_name);
  error_free &= setExtendedDomainName(t_extends_domain_name);
  error_free &= setRequirements(t_requirements);
  error_free &= setTypes(t_types);
  error_free &= setConstants(t_constants);
  error_free &= setPredicates(t_predicates);
  error_free &= setTimeless(t_timeless);
  error_free &= setActions(t_actions);
  return error_free;
}

bool Domain::checkNameValidity(const std::string& t_name) const
{
  if (t_name.empty()) {
    std::cerr << "Fatal: Empty name for current DOMAIN" << std::endl;
    return false;
  }
  return true;
}

bool Domain::setName(const std::string& t_name)
{
  if (!checkNameValidity(t_name)) {
    return false;
  }
  name_ = std::move(t_name);
  return true;
}

bool Domain::checkExtendedDomainNameValidity(const std::string& t_name) const
{
  // TODO
  return true;
}

bool Domain::setExtendedDomainName(const std::string& t_name)
{
  extends_domain_name_ = std::move(t_name);
  return true;
}

bool Domain::hasRequirement(const std::string& t_requirement) const
{
  return std::find_if(
           requirements_.begin(), requirements_.end(), [t_requirement](const auto& r) { return t_requirement == r; })
         != requirements_.end();
}

bool Domain::isRequirementValid(const std::string& t_requirement) const
{
  return SupportedRequirements.count(t_requirement) != 0 && SupportedRequirements.at(t_requirement);
}

bool Domain::addRequirement(const std::string& t_requirement)
{
  if (!isRequirementValid(t_requirement)) {
    std::cerr << "Fatal: addRequirement() failure. Invalid requirement" << std::endl;
    return false;
  }
  if (hasRequirement(t_requirement)) {
    std::cerr << "Fatal: addRequirement() failure. Duplicated requirement" << std::endl;
    return false;
  }

  requirements_.emplace_back(t_requirement);
  return true;
}

bool Domain::hasValidUniqueRequirements() const
{
  return hasValidUniqueRequirements(requirements_);
}

bool Domain::hasValidUniqueRequirements(const std::vector<std::string>& t_requirements) const
{
  int req_idx = 0, lambda_idx = 0;
  for (const auto& req : t_requirements) {
    if (!isRequirementValid(req)) {
      return false;
    }

    lambda_idx = 0;
    auto lambda = [req, req_idx, &lambda_idx](const auto& r) { return (lambda_idx++ != req_idx && req == r); };

    if (std::find_if(t_requirements.begin(), t_requirements.end(), lambda) != t_requirements.end()) {
      return false;
    }
    ++req_idx;
  }
  return true;
}

bool Domain::setRequirements(const std::vector<std::string>& t_requirements)
{
  if (!hasValidUniqueRequirements(t_requirements)) {
    std::cerr << "Fatal: setRequirements() failure. Invalid or duplicated requirement(s)" << std::endl;
    return false;
  }

  requirements_ = std::move(t_requirements);
  return true;
}

bool Domain::hasType(const std::pair<std::string, std::string>& t_type) const
{
  return types_.count(t_type.first) > 0;
}

bool Domain::isTypeValid(const std::pair<std::string, std::string>& t_type) const
{
  return !(t_type.first.empty() || t_type.second.empty());
}

bool Domain::addType(const std::pair<std::string, std::string> t_type)
{
  if (!isTypeValid(t_type)) {
    std::cerr << "Fatal: addType() failure. Invalid type" << std::endl;
    return false;
  }
  if (hasType(t_type)) {
    std::cerr << "Fatal: addType() failure. Duplicated type" << std::endl;
    return false;
  }

  types_.emplace(t_type);
  return true;
}

bool Domain::hasValidUniqueTypes() const
{
  return hasValidUniqueTypes(types_);
}

bool Domain::hasValidUniqueTypes(const UmapStrStr& t_types) const
{
  for (const auto& type : t_types) {
    if (!isTypeValid(type)) {
      return false;
    }
  }

  return helpers::buildTypesHierarchy(t_types).count("object") > 0;
}

bool Domain::setTypes(const UmapStrStr& t_types)
{
  if (!hasValidUniqueTypes(t_types)) {
    std::cerr << "Fatal: setTypes() failure. Invalid or duplicated types" << std::endl;
    return false;
  }

  types_ = std::move(t_types);
  helpers::fixTypesHierarchy(types_); // TODO: useless?
  return true;
}

bool Domain::hasConstant(const LiteralTerm& t_constant) const
{
  return std::find_if(constants_.begin(),
                      constants_.end(),
                      [t_constant](const auto& c) { return t_constant.getName() == c.getName(); })
         != constants_.end();
}

bool Domain::isConstantValid(const LiteralTerm& t_constant) const
{
  return !(extends_domain_name_.empty() && !t_constant.isValid(types_));
}

bool Domain::addConstant(const LiteralTerm& t_constant)
{
  if (!isConstantValid(t_constant)) {
    std::cerr << "Fatal: addConstant() failure. Invalid constant" << std::endl;
    return false;
  }
  if (hasConstant(t_constant)) {
    std::cerr << "Fatal: addConstant() failure. Duplicated constant" << std::endl;
    return false;
  }

  constants_.emplace_back(t_constant);
  return true;
}

bool Domain::hasValidUniqueConstants() const
{
  return hasValidUniqueConstants(constants_);
}

bool Domain::hasValidUniqueConstants(const std::vector<LiteralTerm>& t_constants) const
{
  int cons_idx = 0, lambda_idx = 0;
  for (const auto& cons : t_constants) {
    if (!isConstantValid(cons)) {
      return false;
    }

    lambda_idx = 0;
    auto lambda = [cons, cons_idx, &lambda_idx](const auto& c) {
      return (lambda_idx++ != cons_idx && cons.getName() == c.getName());
    };

    if (std::find_if(t_constants.begin(), t_constants.end(), lambda) != t_constants.end()) {
      return false;
    }
    ++cons_idx;
  }
  return true;
}

bool Domain::setConstants(const std::vector<LiteralTerm>& t_constants)
{
  if (!hasValidUniqueConstants(t_constants)) {
    std::cerr << "Fatal: setConstants() failure. Invalid or duplicated constant(s)" << std::endl;
    return false;
  }

  constants_ = std::move(t_constants);
  return true;
}

bool Domain::hasPredicate(const Predicate& t_predicate) const
{
  return std::find_if(predicates_.begin(),
                      predicates_.end(),
                      [t_predicate, *this](const auto& p) { return t_predicate.isEquivalentTo(p, this->getTypes()); })
         != predicates_.end();
}

bool Domain::isPredicateValid(const Predicate& t_predicate) const
{
  return !(extends_domain_name_.empty() && !t_predicate.isValid(types_));
}

bool Domain::addPredicate(const Predicate& t_predicate)
{
  if (!isPredicateValid(t_predicate)) {
    std::cerr << "Fatal: addPredicate() failure. Invalid predicate" << std::endl;
    return false;
  }
  if (hasPredicate(t_predicate)) {
    // TODO: implement the "keep the parent one" policy
    std::cerr << "Fatal: addPredicate() failure. Duplicated predicate" << std::endl;
    return false;
  }

  predicates_.emplace_back(t_predicate);
  return true;
}

bool Domain::hasValidUniquePredicates() const
{
  return hasValidUniquePredicates(predicates_);
}

bool Domain::hasValidUniquePredicates(const std::vector<Predicate>& t_predicates) const
{
  int pred_idx = 0, lambda_idx = 0;
  for (const auto& pred : t_predicates) {
    if (!isPredicateValid(pred)) {
      return false;
    }

    lambda_idx = 0;
    auto lambda = [pred, *this, pred_idx, &lambda_idx](const auto& p) {
      return (lambda_idx++ != pred_idx && pred.isEquivalentTo(p, this->getTypes()));
    };

    if (std::find_if(t_predicates.begin(), t_predicates.end(), lambda) != t_predicates.end()) {
      // TODO: implement the "keep the parent one" policy
      return false;
    }
    ++pred_idx;
  }
  return true;
}

bool Domain::setPredicates(const std::vector<Predicate>& t_predicates)
{
  if (!hasValidUniquePredicates(t_predicates)) {
    std::cerr << "Fatal: setPredicates() failure. Invalid or duplicated predicate(s)" << std::endl;
    return false;
  }

  predicates_ = std::move(t_predicates);
  return true;
}

bool Domain::hasTimeless(const LiteralExpression& t_timeless) const
{
  return std::find_if(timeless_.begin(),
                      timeless_.end(),
                      [t_timeless, *this](const auto& tl) { return helpers::operator==(tl, t_timeless); })
         != timeless_.end();
}

bool Domain::isTimelessValid(const LiteralExpression& t_timeless) const
{
  return !(extends_domain_name_.empty() && !t_timeless.isValid({}, types_, constants_, predicates_, {}));
}

bool Domain::addTimeless(const LiteralExpression& t_timeless)
{
  if (!isTimelessValid(t_timeless)) {
    std::cerr << "Fatal: addTimeless() failure. Invalid timeless" << std::endl;
    return false;
  }
  if (hasTimeless(t_timeless)) {
    std::cerr << "Fatal: addTimeless() failure. Duplicated timeless" << std::endl;
    return false;
  }

  timeless_.emplace_back(t_timeless);
  return true;
}

bool Domain::hasValidUniqueTimeless() const
{
  return hasValidUniqueTimeless(timeless_);
}

bool Domain::hasValidUniqueTimeless(const std::vector<LiteralExpression>& t_timeless) const
{
  int tim_idx = 0, lambda_idx = 0;
  for (const auto& tim : t_timeless) {
    if (!isTimelessValid(tim)) {
      return false;
    }

    lambda_idx = 0;
    auto lambda = [tim, *this, tim_idx, &lambda_idx](const auto& tl) {
      return (lambda_idx++ != tim_idx && helpers::operator==(tl, tim));
    };

    if (std::find_if(t_timeless.begin(), t_timeless.end(), lambda) != t_timeless.end()) {
      return false;
    }
    ++tim_idx;
  }
  return true;
}

bool Domain::setTimeless(const std::vector<LiteralExpression>& t_timeless)
{
  if (!hasValidUniqueTimeless(t_timeless)) {
    std::cerr << "Fatal: setTimeless() failure. Invalid or duplicated timeless" << std::endl;
    return false;
  }

  timeless_ = std::move(t_timeless);
  return true;
}

bool Domain::hasAction(const Action& t_action) const
{
  return std::find_if(
           actions_.begin(), actions_.end(), [t_action, *this](const auto& a) { return t_action.isEquivalentTo(a); })
         != actions_.end();
}

bool Domain::isActionValid(const Action& t_action) const
{
  return !(extends_domain_name_.empty() && !t_action.isValid(types_, constants_, predicates_, timeless_));
}

bool Domain::addAction(const Action& t_action)
{
  if (!isActionValid(t_action)) {
    std::cerr << "Fatal: addAction() failure. Invalid action" << std::endl;
    return false;
  }
  if (hasAction(t_action)) {
    std::cerr << "Fatal: addAction() failure. Duplicated action" << std::endl;
    return false;
  }

  actions_.emplace_back(t_action);
  return true;
}

bool Domain::hasValidUniqueActions() const
{
  return hasValidUniqueActions(actions_);
}

bool Domain::hasValidUniqueActions(const std::vector<Action>& t_actions) const
{
  int act_idx = 0, lambda_idx = 0;
  for (const auto& act : t_actions) {
    if (!isActionValid(act)) {
      return false;
    }

    lambda_idx = 0;
    auto lambda = [act, act_idx, &lambda_idx](const auto& a) {
      return (lambda_idx++ != act_idx && act.isEquivalentTo(a));
    };

    if (std::find_if(t_actions.begin(), t_actions.end(), lambda) != t_actions.end()) {
      return false;
    }
    ++act_idx;
  }
  return true;
}

bool Domain::setActions(const std::vector<Action>& t_actions)
{
  if (!hasValidUniqueActions(t_actions)) {
    std::cerr << "Fatal: setActions() failure. Invalid or duplicated action(s)" << std::endl;
    return false;
  }

  actions_ = std::move(t_actions);
  return true;
}

bool Domain::isValid() const
{
  bool is_valid = true;
  is_valid &= checkNameValidity(name_);
  is_valid &= checkExtendedDomainNameValidity(extends_domain_name_);
  is_valid &= hasValidUniqueRequirements(requirements_);
  is_valid &= hasValidUniqueConstants(constants_);
  is_valid &= hasValidUniqueTypes(types_);
  is_valid &= hasValidUniquePredicates();
  is_valid &= hasValidUniqueTimeless(timeless_);
  is_valid &= hasValidUniqueActions(actions_);

  return is_valid;
}

bool Domain::isEquivalentTo(const Domain& t_other) const
{
  // TODO
  return *this == t_other;
}

Domain& Domain::operator=(const Domain& t_other)
{
  if (!set(t_other.getName(),
           t_other.getExtendedDomainName(),
           t_other.getRequirements(),
           t_other.getTypes(),
           t_other.getConstants(),
           t_other.getPredicates(),
           t_other.getTimeless(),
           t_other.getActions())) {
    std::cerr << "DOMAIN: copy operator returned at least one error, check the log" << std::endl;
  }
  return *this;
}

std::string Domain::toPddl(bool t_typing, int t_pad_lv) const
{
  std::string out;
  if (!isValid()) {
    out += "; THIS DOMAIN HAS BEEN JUDGED AS INVALID, CHECK BEFORE USAGE\n";
  }

  auto pad_aligners_define = helpers::getPddlAligners(t_pad_lv);
  out += pad_aligners_define.second[0] + "(define" + pad_aligners_define.second[1];

  // increase PAD level by 1
  auto pad_aligners_domain = helpers::getPddlAligners(pad_aligners_define.first);
  out += pad_aligners_domain.second[0] + "(domain " + name_ + ")\n";

  if (!extends_domain_name_.empty()) {
    out += pad_aligners_domain.second[0] + "(:extends " + name_ + ")\n";
  }

  if (!requirements_.empty()) {
    out += pad_aligners_domain.second[0] + "(:requirements";
    for (const auto& r : requirements_) {
      out += " :" + r;
    }
    out += ")\n";
  }

  if (types_.size() > 1 && t_typing) { // I add object by default
    out += pad_aligners_domain.second[0] + "(:types" + pad_aligners_define.second[1];
    auto pad_aligners_type = helpers::getPddlAligners(pad_aligners_domain.first);
    const auto& th = helpers::aggregateByParentType(types_);
    for (const auto& [par, childs] : th) {
      out += pad_aligners_type.second[0];
      for (const auto& ch : childs) {
        out += ch + " ";
      }
      out += "- " + par + "\n";
    }
    out.pop_back();
    out += pad_aligners_domain.second[2] + ")\n";
  }

  if (!constants_.empty()) {
    out += pad_aligners_domain.second[0] + "(:constants" + pad_aligners_domain.second[1];
    auto pad_aligners_const = helpers::getPddlAligners(pad_aligners_domain.first);
    for (const auto& c : constants_) {
      auto cs = c.toPddl(t_typing, pad_aligners_const.first);
      out += pad_aligners_const.second[0] + cs;
    }
    out += pad_aligners_domain.second[2] + ")\n";
  }

  if (!predicates_.empty()) {
    out += pad_aligners_domain.second[0] + "(:predicates" + pad_aligners_domain.second[1];
    auto pad_aligners_pred = helpers::getPddlAligners(pad_aligners_domain.first);
    for (const auto& pred : predicates_) {
      out += pred.toPddl(t_typing, pad_aligners_domain.first) + "\n";
    }
    out.pop_back();
    out += pad_aligners_domain.second[2] + ")\n";
  }

  if (!timeless_.empty()) {
    out += pad_aligners_domain.second[0] + "(:timeless" + pad_aligners_domain.second[1];
    auto pad_aligners_tim = helpers::getPddlAligners(pad_aligners_domain.first);
    for (const auto& tim : timeless_) {
      out += tim.toPddl(t_typing, pad_aligners_domain.first) + "\n";
    }
    out.pop_back();
    out += pad_aligners_domain.second[2] + ")\n";
  }

  if (!actions_.empty()) {
    for (const auto& act : actions_) {
      out += act.toPddl(t_typing, pad_aligners_define.first) + "\n";
    }
  }
  out.pop_back();
  out += pad_aligners_define.second[2] + ")";
  return out;
}

bool rtask::commons::pddl_generator::operator==(const Domain& t_first, const Domain& t_second)
{

  if (t_first.getName() != t_second.getName() || t_first.getExtendedDomainName() != t_second.getExtendedDomainName()) {
    return false;
  }

  auto f_req = t_first.getRequirements();
  auto s_req = t_second.getRequirements();
  if (f_req.size() != s_req.size()) {
    return false;
  }

  std::sort(f_req.begin(), f_req.end());
  std::sort(s_req.begin(), s_req.end());
  for (size_t i = 0; i < f_req.size(); ++i) {
    if (f_req.at(i) != s_req.at(i)) {
      return false;
    }
  }

  const auto& f_types = t_first.getTypes();
  const auto& s_types = t_second.getTypes();
  if (f_types.size() != s_types.size()) {
    return false;
  }
  for (const auto& [f_key, f_val] : f_types) {
    if (s_types.count(f_key) == 0 || f_val != s_types.at(f_key)) {
      return false;
    }
  }

  const auto& f_consts = t_first.getConstants();
  const auto& s_consts = t_second.getConstants();
  if (f_consts.size() != s_consts.size()) {
    return false;
  }
  for (const auto& f_const : f_consts) {
    if (std::find(s_consts.begin(), s_consts.end(), f_const) == s_consts.end()) {
      return false;
    }
  }

  const auto& f_preds = t_first.getPredicates();
  const auto& s_preds = t_second.getPredicates();
  if (f_preds.size() != s_preds.size()) {
    return false;
  }
  for (const auto& f_pred : f_preds) {
    if (std::find(s_preds.begin(), s_preds.end(), f_pred) == s_preds.end()) {
      return false;
    }
  }

  const auto& f_tims = t_first.getTimeless();
  const auto& s_tims = t_second.getTimeless();
  if (f_tims.size() != s_tims.size()) {
    return false;
  }
  for (const auto& f_tim : f_tims) {
    if (std::find(s_tims.begin(), s_tims.end(), f_tim) == s_tims.end()) {
      return false;
    }
  }

  const auto& f_acts = t_first.getActions();
  const auto& s_acts = t_second.getActions();
  if (f_acts.size() != s_acts.size()) {
    return false;
  }
  for (const auto& f_act : f_acts) {
    if (std::find(s_acts.begin(), s_acts.end(), f_act) == s_acts.end()) {
      return false;
    }
  }

  return true;
}

bool rtask::commons::pddl_generator::operator!=(const Domain& t_first, const Domain& t_second)
{
  return !(t_first == t_second);
}

std::ostream& rtask::commons::pddl_generator::operator<<(std::ostream& t_out, const Domain& t_dom)
{
  t_out << " ## DOMAIN ## " << std::endl;
  std::cout << t_dom.toPddl();
  return t_out;
}

std::ostream& rtask::commons::pddl_generator::operator<<(std::ostream& t_out, std::shared_ptr<Domain> t_ptr)
{
  t_out << (t_ptr ? *t_ptr : Domain());
  return t_out;
}
