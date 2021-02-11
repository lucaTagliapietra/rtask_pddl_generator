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
      exit(EXIT_FAILURE);
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

bool Domain::setName(const std::string& t_name)
{
  if (t_name.empty()) {
    std::cerr << "Fatal: Empty name for current Domain" << std::endl;
    return false;
  }
  name_ = std::move(t_name);
  return true;
}

bool Domain::setExtendedDomainName(const std::string& t_name)
{
  extends_domain_name_ = std::move(t_name);
  return true;
}

bool Domain::setRequirements(const std::vector<std::string>& t_requirements)
{
  requirements_.clear();
  for (const auto& req : t_requirements) {
    if (SupportedRequirements.count(req) == 0 || !SupportedRequirements.at(req)) {
      std::cerr << "Fatal: Invalid or Unsupported Requirement for current Domain" << name_ << std::endl;
      return false;
    }
  }
  requirements_ = std::move(t_requirements);
  return true;
}

bool Domain::setTypes(const UmapStrStr& t_types)
{
  types_ = std::move(t_types);
  helpers::fixTypesHierarchy(types_);
  return true;
}

bool Domain::setConstants(const std::vector<LiteralTerm>& t_constants)
{
  bool error_free = true;
  constants_.clear();

  for (auto& cons : t_constants) {
    if (!cons.isValid(types_)) {
      std::cerr << "Validation Error: invalid CONST: " << cons.getName() << " in current Domain: " << name_
                << std::endl;
      error_free = false;
      continue;
    }
    // Two CONST cannot have the same name (regardless from type), otherwise it will be not known which one to use
    const auto& it = std::find_if(
      constants_.begin(), constants_.end(), [cons](const LiteralTerm& oc) { return cons.getName() == oc.getName(); });
    if (it != constants_.end()) {
      std::cerr << "Validation Error: CONST: " << cons.getName() << " already defined in current Domain: " << name_
                << std::endl;
      error_free = false;
      continue;
    }
    constants_.push_back(cons);
    constants_.back().setIsAConstantTerm(true);
  }
  return error_free;
}

bool Domain::setPredicates(const std::vector<Predicate>& t_predicates)
{
  bool error_free = true;
  for (const auto& pred : t_predicates) {
    if (!pred.isValid(types_)) {
      std::cerr << "Fatal: Invalid predicate in current domain" << std::endl;
      error_free = false;
      continue;
    }
    const auto f = std::find_if(predicates_.begin(), predicates_.end(), [pred, *this](const auto& p) {
      return pred.isEquivalentTo(p, this->getTypes());
    });
    if (f != predicates_.end()) {
      // TODO: implement the "keep the parent one" policy
      std::cerr << "Fatal: Predicate " << name_ << "is equivalent to: " << f->getName() << " in current domain"
                << std::endl;
      error_free = false;
      continue;
    }
    predicates_.emplace_back(pred);
  }
  return error_free;
}

bool Domain::setTimeless(const std::vector<LiteralExpression>& t_timeless)
{
  bool error_free = true;
  for (const auto& tim : t_timeless) {
    if (!tim.isValid({}, types_, constants_, predicates_, {})) {
      std::cerr << "Fatal: Invalid TIMELESS " << name_ << " in current domain" << std::endl;
      error_free = false;
      continue;
    }

    const auto f =
      std::find_if(timeless_.begin(), timeless_.end(), [tim](const auto& tl) { return helpers::operator==(tl, tim); });
    if (f != timeless_.end()) {
      std::cerr << "Fatal: Timeless " << name_ << "is already defined as: " << *f << " in current domain" << std::endl;
      error_free = false;
      continue;
    }
    timeless_.push_back(tim);
  }
  return error_free;
}

bool Domain::setActions(const std::vector<Action>& t_actions)
{
  actions_.clear();
  bool error_free = true;
  for (const auto& act : t_actions) {
    if (act.isValid(types_, constants_, predicates_, timeless_)) {
      // Todo: duplicates check to be implemented
      actions_.push_back(act);
    }
    else {
      error_free = false;
    }
  }
  return error_free;
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
  // if(!isValid()){out += "; THIS DOMAIN HAS BEEN JUDGED AS INVALID, CHECK BEFORE USAGE\n" }
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
