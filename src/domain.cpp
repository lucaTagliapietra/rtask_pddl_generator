#include <algorithm>
#include <map>

#include "boost/make_shared.hpp"

#include "commons/domain.h"
#include "commons/utils.h"

rtask::commons::Domain::Domain(const std::string& t_name,
                               const std::vector<SemanticEntity>& t_semantic_entities,
                               const std::vector<Predicate>& t_predicates,
                               const std::vector<Action>& t_actions,
                               const bool t_req_typing,
                               const bool t_req_equality,
                               const bool t_req_strips)
{
  setDomain(t_name, t_semantic_entities, t_predicates, t_actions, t_req_typing, t_req_equality, t_req_strips);
}

rtask::commons::Domain::Domain(const rtask_msgs::Domain& t_msg)
{
  setFromDomainMsg(t_msg);
}
rtask::commons::Domain::Domain(const rtask_msgs::DomainConstPtr t_msg_ptr)
{
  setFromDomainMsg(t_msg_ptr);
}
rtask::commons::Domain::Domain(XmlRpc::XmlRpcValue& t_node)
{
  setDomainFromXmlRpc(t_node);
}

bool rtask::commons::Domain::isValid() const
{
  return !m_name.empty() && !m_semantic_entities.empty() && !m_predicates.empty() && !m_actions.empty();
}

rtask_msgs::DomainPtr rtask::commons::Domain::toDomainMsg() const
{
  rtask_msgs::DomainPtr t_msg = boost::make_shared<rtask_msgs::Domain>();
  t_msg->name = m_name;
  t_msg->requires_strips = m_requirements.strips;
  t_msg->requires_typing = m_requirements.typing;
  t_msg->requires_equalty = m_requirements.equality;
  for (const auto& semantic_entity : m_semantic_entities) {
    t_msg->semantic_entities.emplace_back(semantic_entity.toMsg());
  }
  for (const auto& predicate : m_predicates) {
    t_msg->predicates.emplace_back(predicate.toMsg());
  }
  for (const auto& action : m_actions) {
    t_msg->actions.emplace_back(*action.toActionMsg());
  }

  return t_msg;
}

bool rtask::commons::Domain::toPddl(std::string out) const
{

  out = "(define (domain " + m_name + ")\n";

  if (m_requirements.strips || m_requirements.typing || m_requirements.equality) {
    out += "\t(:requirements";
    m_requirements.typing ? out += " :typing" : "";
    m_requirements.strips ? out += " :strips" : "";
    m_requirements.equality ? out += " :equality" : "";
    out += ")\n";
  }

  if (m_requirements.typing) {
    out += "(\t(:types";
    for (const auto& semantic_entity : m_semantic_entities)
      out += (" " + semantic_entity.type);
    out += ")\n";
  }

  std::map<std::string, std::string> symbol_map;
  for (const auto& entity : m_semantic_entities) {

    // map di simbolo e classe
    for (const auto& symbol : entity.symbols)
      symbol_map[symbol] = entity.semantic_class;
  }

  std::vector<Predicate> temporary_predicates;
  std::vector<SemanticEntity> temporary_entities;
  std::vector<std::string> exploited_symbols = {};

  for (const auto& pred : m_predicates) {

    Predicate p;
    p.name = pred.name;

    exploited_symbols.clear();

    // Error! Not enough symbols for input arguments
    if (pred.args.size() > symbol_map.size())
      return false;

    while (unsigned int i = 0 < pred.args.size()) {

      for (auto iter = symbol_map.begin(); iter != symbol_map.end();) {
        if (pred.args.at(i) == iter->second
            && std::find(exploited_symbols.begin(), exploited_symbols.end(), iter->first) == exploited_symbols.end()) {
          p.args.push_back(iter->first);
          exploited_symbols.push_back(iter->first);
          break;
        }

        // Error! Not enough symbols for the class of current argument
        if (pred.args.at(i) == iter->second
            && std::find(exploited_symbols.begin(), exploited_symbols.end(), iter->first) != exploited_symbols.end()
            && pred.args.at(i) != std::next(iter)->second)
          return false;

        // Error! Not enough classes for input arguments
        if (pred.args.at(i) != iter->second && std::next(iter) == symbol_map.end())
          return false;
      }
    }
    temporary_predicates.push_back(p);
  }

  out += "\t(:predicates ";

  unsigned int count = 0;

  for (const auto& temp : temporary_predicates) {

    out += " (" + temp.name + " ";

    for (const auto& arg : temp.args) {
      count == 0 ? out += "?" : out += " ?";
      out += arg;
      if (m_requirements.typing) {
        std::string type;
        getSemanticEntityType(arg, type);
        out += (" - " + type);
      }
    }
    out += ")";
  }

  out += ")\n";

  for (unsigned int i = 0; i < m_actions.size(); ++i) {
    out += m_actions.at(i).toPddl(m_requirements.typing, m_requirements.equality, m_requirements.strips);
  }

  out += "))\n";

  return true;
}

bool rtask::commons::Domain::setDomainFromXmlRpc(XmlRpc::XmlRpcValue& t_node)
{

  // Clear all just to be sure
  clear();

  // Name
  if (commons::utils::checkXmlRpcSanity("name", t_node, XmlRpc::XmlRpcValue::TypeString)) {
    m_name = static_cast<std::string>(t_node["name"]);
  }

  // Requirements
  if (commons::utils::checkXmlRpcSanity("requirements", t_node, XmlRpc::XmlRpcValue::TypeStruct)) {
    if (commons::utils::checkXmlRpcSanity("equality", t_node["requirements"], XmlRpc::XmlRpcValue::TypeBoolean)) {
      m_requirements.equality = static_cast<bool>(t_node["requirements"]["equality"]);
    }
    if (commons::utils::checkXmlRpcSanity("requirements", t_node["requirements"], XmlRpc::XmlRpcValue::TypeBoolean)) {
      m_requirements.strips = static_cast<bool>(t_node["requirements"]["strips"]);
    }
    if (commons::utils::checkXmlRpcSanity("requirements", t_node["requirements"], XmlRpc::XmlRpcValue::TypeBoolean)) {
      m_requirements.typing = static_cast<bool>(t_node["requirements"]["typing"]);
    }
  }

  // Semantic objects
  if (commons::utils::checkXmlRpcSanity("semantic_objects", t_node, XmlRpc::XmlRpcValue::TypeArray)) {
    if (t_node["semantic_objects"].size() == 0) {
      std::cout << "Empty semantic object vector for domain " << m_name << std::endl;
      return isValid();
    }
    for (auto& entities_same_type : t_node["semantic_objects"]) {

      std::string type = commons::NULL_TYPE;
      if (commons::utils::checkXmlRpcSanity("type", entities_same_type.second, XmlRpc::XmlRpcValue::TypeString)) {
        type = static_cast<std::string>(entities_same_type.second["type"]);
      }

      std::string semantic_class{};
      if (commons::utils::checkXmlRpcSanity("class", entities_same_type.second, XmlRpc::XmlRpcValue::TypeString)) {
        semantic_class = static_cast<std::string>(entities_same_type.second["class"]);
      }

      std::vector<std::string> symbols;
      if (commons::utils::checkXmlRpcSanity("symbols", entities_same_type.second, XmlRpc::XmlRpcValue::TypeArray)) {
        if (entities_same_type.second["symbols"].size() == 0) {
          std::cout << "Empty object symbols vector for type " << type << " for domain " << m_name << std::endl;
          return isValid();
        }
        for (auto& symbol : entities_same_type.second["symbols"]) {
          if (symbol.second.getType() == XmlRpc::XmlRpcValue::TypeString
              && !static_cast<std::string>(symbol.second).empty()) {
            symbols.emplace_back(static_cast<std::string>(symbol.second));
          }
        }
      }

      m_semantic_entities.emplace_back(SemanticEntity(symbols, type, semantic_class));
    }
  }

  if (commons::utils::checkXmlRpcSanity("predicates", t_node, XmlRpc::XmlRpcValue::TypeArray)) {
    if (t_node["predicates"].size() == 0) {
      std::cout << "No predicates for domain " << m_name << std::endl;
      return false;
    }
    for (auto& pred : t_node["predicates"]) {
      if (!commons::utils::checkXmlRpcSanity("cmd", pred.second, XmlRpc::XmlRpcValue::TypeString)) {
        std::cout << "Empty or invalid cmd in predicate for domain " << m_name << std::endl;
        return false;
      }
      std::string name = pred.second["cmd"];
      std::vector<std::string> args{};

      if (commons::utils::checkXmlRpcSanity("args", pred.second, XmlRpc::XmlRpcValue::TypeArray)) {
        for (auto& arg : pred.second["args"]) {

          if (arg.second.getType() == XmlRpc::XmlRpcValue::TypeString
              && !static_cast<std::string>(arg.second).empty()) {
            args.emplace_back(static_cast<std::string>(arg.second));
          }
        }
      }

      m_predicates.push_back({name, args});
    }
  }

  if (commons::utils::checkXmlRpcSanity("actions", t_node, XmlRpc::XmlRpcValue::TypeArray)) {
    if (t_node["actions"].size() == 0) {
      std::cout << "No actions for domain " << m_name << std::endl;
      return false;
    }
    for (auto& a : t_node["actions"]) {
      Action action;
      action.setActionFromXmlRpc(a.second);
      m_actions.push_back(action);
    }
  }
  return isValid();
}

bool rtask::commons::Domain::setFromDomainMsg(const rtask_msgs::Domain& t_msg)
{
  m_name = t_msg.name;
  m_requirements.strips = t_msg.requires_strips;
  m_requirements.typing = t_msg.requires_typing;
  m_requirements.equality = t_msg.requires_equalty;

  for (const auto& semantic_entity : t_msg.semantic_entities) {
    m_semantic_entities.emplace_back(semantic_entity);
  }
  for (const auto& predicate : t_msg.predicates) {
    m_predicates.emplace_back(predicate);
  }
  for (const auto& action : t_msg.actions) {
    m_actions.emplace_back(action);
  }

  return isValid();
}
bool rtask::commons::Domain::setFromDomainMsg(const rtask_msgs::DomainConstPtr& t_msg_ptr)
{
  return setFromDomainMsg(*t_msg_ptr);
}
bool rtask::commons::Domain::setDomain(const std::string& t_name,
                                       const std::vector<SemanticEntity>& t_semantic_entities,
                                       const std::vector<Predicate>& t_predicates,
                                       const std::vector<Action>& t_actions,
                                       const bool t_req_typing,
                                       const bool t_req_equality,
                                       const bool t_req_strips)
{
  m_name = t_name;
  m_requirements.strips = t_req_strips;
  m_requirements.typing = t_req_typing;
  m_requirements.equality = t_req_equality;

  for (const auto& semantic_entity : t_semantic_entities) {
    m_semantic_entities.emplace_back(semantic_entity);
  }

  for (const auto& predicate : t_predicates) {
    m_predicates.emplace_back(predicate);
  }

  for (const auto& action : t_actions) {
    m_actions.emplace_back(action);
  }

  return isValid();
}

void rtask::commons::Domain::clear()
{
  m_name.clear();
  m_semantic_entities.clear();
  m_predicates.clear();
  m_actions.clear();
  m_requirements = {};
}

// ---------------------
// Semantic Entity Level
// ---------------------

bool rtask::commons::Domain::hasSemanticEntity(const std::string& t_symbol, const std::string& t_type) const
{
  if (t_type.empty()) {

    auto equal_name = [t_symbol](const SemanticEntity& a) {
      bool eq = false;
      for (unsigned int i = 0; i < a.symbols.size(); ++i) {
        eq = a.symbols[i] == t_symbol;
      }
      return eq;
    };
    return std::find_if(m_semantic_entities.begin(), m_semantic_entities.end(), equal_name)
           != m_semantic_entities.end();
  }
  auto equal_name_type = [t_symbol, t_type](const SemanticEntity& a) {
    bool eq = false;
    for (unsigned int i = 0; i < a.symbols.size(); ++i) {
      eq = a.symbols[i] == t_symbol && a.type == t_type;
    }
    return eq;
  };
  return std::find_if(m_semantic_entities.begin(), m_semantic_entities.end(), equal_name_type)
         != m_semantic_entities.end();
}

bool rtask::commons::Domain::getSemanticEntityType(const std::string& t_symbol, std::string& t_type) const
{
  if (!hasSemanticEntity(t_symbol)) {
    return false;
  }
  auto equal_name = [t_symbol](const SemanticEntity& a) {
    bool eq = false;
    for (unsigned int i = 0; i < a.symbols.size(); ++i) {
      eq = a.symbols[i] == t_symbol;
    }
    return eq;
  };
  auto const it = std::find_if(m_semantic_entities.begin(), m_semantic_entities.end(), equal_name);
  t_type = it->type;
  return true;
}

bool rtask::commons::Domain::getSemanticEntityClass(const std::string& t_symbol, std::string& t_class) const
{
  if (!hasSemanticEntity(t_symbol)) {
    return false;
  }
  auto equal_name = [t_symbol](const SemanticEntity& a) {
    bool eq = false;
    for (unsigned int i = 0; i < a.symbols.size(); ++i) {
      eq = a.symbols[i] == t_symbol;
    }
    return eq;
  };
  auto const it = std::find_if(m_semantic_entities.begin(), m_semantic_entities.end(), equal_name);
  t_class = it->semantic_class;
  return true;
}

bool rtask::commons::Domain::getSemanticEntity(const std::string& t_symbol, SemanticEntity& t_semantic_entity) const
{

  std::string t_type{};
  std::string t_class{};
  std::vector<std::string> t_properties{};

  if (!getSemanticEntityType(t_symbol, t_type) || !getSemanticEntityClass(t_symbol, t_class)) {
    return false;
  }

  t_semantic_entity = {{t_symbol}, t_type, t_class};
  return true;
}

bool rtask::commons::Domain::addSemanticEntity(const std::string& t_symbol,
                                               const std::string& t_type,
                                               const std::string& t_class)
{
  if (!hasSemanticEntity(t_symbol, t_type)) {
    if (hasSemanticEntity(t_symbol)) {
      return false;
    }
    m_semantic_entities.push_back({{t_symbol}, t_type, t_class});
  }
  return true;
}

bool rtask::commons::Domain::removeSemanticEntity(const std::string& t_symbol, const std::string& t_type)
{
  if (hasSemanticEntity(t_symbol)) {
    if (!t_type.empty() && !hasSemanticEntity(t_symbol, t_type)) {
      return false;
    }
    auto equal_name = [t_symbol](const SemanticEntity& a) {
      bool eq = false;
      for (unsigned int i = 0; i < a.symbols.size(); ++i) {
        eq = a.symbols[i] == t_symbol;
      }
      return eq;
    };
    auto const it = std::find_if(m_semantic_entities.begin(), m_semantic_entities.end(), equal_name);
    m_semantic_entities.erase(it);
  }
  return true;
}

// ---------------
// Predicate Level
// ---------------
bool rtask::commons::Domain::hasPredicate(const std::string& t_name, const std::vector<std::string>& t_args) const
{

  auto equal_name_and_args = [t_name, t_args](const Predicate& a) { return a.name == t_name && a.args == t_args; };
  return std::find_if(m_predicates.begin(), m_predicates.end(), equal_name_and_args) != m_predicates.end();
}

bool rtask::commons::Domain::hasPredicateName(const std::string& t_name) const
{

  auto equal_name = [t_name](const Predicate& a) { return a.name == t_name; };
  return std::find_if(m_predicates.begin(), m_predicates.end(), equal_name) != m_predicates.end();
}

bool rtask::commons::Domain::getPredicates(const std::string& t_name, std::vector<Predicate>& t_predicates) const
{

  std::vector<std::string> t_args;
  if (!hasPredicateName(t_name))
    return false;

  for (unsigned int i = 0; i < m_predicates.size(); ++i) {
    if (m_predicates.at(i).name == t_name) {
      t_predicates.push_back(m_predicates.at(i));
    }
  }
  return true;
}

bool rtask::commons::Domain::addPredicate(const std::string& t_name, const std::vector<std::string>& t_args)
{
  if (hasPredicate(t_name, t_args))
    return false;

  m_predicates.push_back({t_name, t_args});
  return true;
}
bool rtask::commons::Domain::removePredicate(const std::string& t_name, const std::vector<std::string>& t_args)
{
  if (!hasPredicate(t_name, t_args))
    return false;

  auto equal_name_and_args = [t_name, t_args](const Predicate& a) { return a.name == t_name && a.args == t_args; };
  auto const it = std::find_if(m_predicates.begin(), m_predicates.end(), equal_name_and_args);
  m_predicates.erase(it);

  return true;
}

// ------------
// Action Level
// ------------
bool rtask::commons::Domain::hasAction(const Action& t_act) const
{
  return hasAction(
    t_act.getName(), t_act.getType(), t_act.getParameters(), t_act.getPreconditions(), t_act.getEffects());
}

bool rtask::commons::Domain::hasAction(const std::string& t_name,
                                       const std::string& t_type,
                                       const std::vector<Entity>& t_params,
                                       const std::vector<Command>& t_preconditions,
                                       const std::vector<Command>& t_effects) const
{
  bool same_action = false;
  for (const auto& act : m_actions)
    same_action &= equalActions(act, {t_name, t_type, t_params, t_preconditions, t_effects});

  return same_action;

  //    // Different name
  //    if (act.getName() != t_name)
  //      continue;
  //    // Different type
  //    if (act.getType() != t_type)
  //      continue;
  //    // Different number of arguments
  //    if (act.getParameters().size() != t_params.size())
  //      continue;
  //    // Different number of preconditions
  //    if (act.getPreconditions().size() != t_preconditions.size())
  //      continue;
  //    // Different number of effects
  //    if (act.getEffects().size() != t_effects.size())
  //      continue;

  //    std::map<std::string, std::string> param_map;
  //    for (const auto& t_param : t_params) {
  //      for (const auto& param : act.getParameters()) {
  //        if (t_param.semantic_entity.semantic_class == param.semantic_entity.semantic_class) {
  //          param_map[t_param.semantic_entity.symbols.front()] = param.semantic_entity.symbols.front();
  //          break;
  //        }
  //      }
  //    }

  //    // Same number of parameters but some of different class
  //    if (param_map.size() != act.getParameters().size())
  //      continue;

  //    bool same_preconditions = true;
  //    for (const auto& t_prec : t_preconditions) {
  //      std::vector<std::string> args;
  //      for (const auto& a : t_prec.predicate.args) {
  //        args.push_back(param_map.at(a));
  //      }
  //      Command t_cmd{{t_prec.predicate.name, args}, t_prec.negate};
  //      same_preconditions &= act.hasPrecondition(t_cmd);
  //      if (!same_preconditions) {
  //        break;
  //      }
  //    }

  //    // Different preconditions, try with another action from available ones
  //    if (!same_preconditions) {
  //      continue;
  //    }

  //    bool same_effects = true;
  //    for (const auto& t_prec : t_preconditions) {
  //      std::vector<std::string> args;
  //      for (const auto& a : t_prec.predicate.args) {
  //        args.push_back(param_map.at(a));
  //      }
  //      Command t_cmd{{t_prec.predicate.name, args}, t_prec.negate};
  //      same_effects &= act.hasEffect(t_cmd);
  //      if (!same_effects) {
  //        break;
  //      }
  //    }

  //    // Different preconditions, try with another action from available ones
  //    if (!same_effects) {
  //      continue;
  //    }

  //    //      bool precondition_found = false;
  //    //      for (const auto& prec : act.getPreconditions()) {

  //    //        if (t_prec.predicate.name != prec.predicate.name)
  //    //          continue;
  //    //        if (t_prec.predicate.args.size() != prec.predicate.args.size())
  //    //          continue;
  //    //        if (t_prec.negate != prec.negate)
  //    //          continue;
  //    //        bool same_args = true;
  //    //        for (const auto& t_arg : t_prec.predicate.args) {
  //    //          if (std::find(prec.predicate.args.begin(), prec.predicate.args.end(), param_map.at(t_arg))
  //    //              == prec.predicate.args.end()) {
  //    //            same_args = false;
  //    //            break;
  //    //          }
  //    //        }
  //    //        if (!same_args)
  //    //          continue;
  //    //        precondition_found = true;
  //    //      }
  //    //      if (!precondition_found) {
  //    //        same_preconditions = false;
  //    //        break;
  //    //      }
  //    //  }
  //    // Different preconditions
  //    //    if (!same_preconditions) {
  //    //      continue;
  //    //    }

  //    //  bool same_effects = true;
  //    //  for (const auto& t_prec : t_effects) {
  //    //    bool effect_found = false;
  //    //    for (const auto& prec : act.getEffects()) {

  //    //      if (t_prec.predicate.name != prec.predicate.name)
  //    //        continue;
  //    //      if (t_prec.predicate.args.size() != prec.predicate.args.size())
  //    //        continue;
  //    //      if (t_prec.negate != prec.negate)
  //    //        continue;
  //    //      bool same_args = true;
  //    //      for (const auto& t_arg : t_prec.predicate.args) {
  //    //        if (std::find(prec.predicate.args.begin(), prec.predicate.args.end(), param_map.at(t_arg))
  //    //            == prec.predicate.args.end()) {
  //    //          same_args = false;
  //    //          break;
  //    //        }
  //    //      }
  //    //      if (!same_args)
  //    //        continue;
  //    //      effect_found = true;
  //    //    }
  //    //    if (!effect_found) {
  //    //      same_effects = false;
  //    //      break;
  //    //    }
  //    //  }
  //    //  // Different effects
  //    //  if (!same_effects) {
  //    //    continue;
  //    //  }

  //    same_action = true;
  //    break;
  //  }
}

bool rtask::commons::Domain::addAction(const std::string& t_name,
                                       const std::string& t_type,
                                       const std::vector<Entity>& t_params,
                                       const std::vector<Command>& t_preconditions,
                                       const std::vector<Command>& t_effects)
{
  if (!hasAction(t_name, t_type, t_params, t_preconditions, t_effects)) {
    m_actions.emplace_back(t_name, t_type, t_params, t_preconditions, t_effects);
    return true;
  }
  return false;
}

bool rtask::commons::Domain::removeAction(const Action& t_act)
{
  return removeAction(
    t_act.getName(), t_act.getType(), t_act.getParameters(), t_act.getPreconditions(), t_act.getEffects());
}

bool rtask::commons::Domain::removeAction(const std::string& t_name,
                                          const std::string& t_type,
                                          const std::vector<Entity>& t_params,
                                          const std::vector<Command>& t_preconditions,
                                          const std::vector<Command>& t_effects)
{

  if (!hasAction(t_name, t_type, t_params, t_preconditions, t_effects))
    return false;

  for (const auto& act : m_actions) {

    if (equalActions(act, {t_name, t_type, t_params, t_preconditions, t_effects})) {
      // auto const it = std::find_if(m_actions.begin(), m_actions.end(), act);
      // m_actions.erase(it);
    }
  }

  return true;
}

// -------------
// Private Level
// -------------

bool rtask::commons::Domain::equalActions(const Action& first_action, const Action& second_action) const
{

  // Different name
  if (first_action.getName() != second_action.getName())
    return false;

  // Different type
  if (first_action.getType() != second_action.getType())
    return false;

  // Different number of arguments
  if (first_action.getParameters().size() != second_action.getParameters().size())
    return false;

  // Different number of preconditions
  if (first_action.getPreconditions().size() != second_action.getPreconditions().size())
    return false;

  // Different number of effects
  if (first_action.getEffects().size() != second_action.getEffects().size())
    return false;

  std::map<std::string, std::string> param_map;
  for (const auto& second_param : second_action.getParameters()) {
    for (const auto& first_param : first_action.getParameters()) {
      if (second_param.semantic_entity.semantic_class == first_param.semantic_entity.semantic_class) {
        param_map[second_param.semantic_entity.symbols.front()] = first_param.semantic_entity.symbols.front();
        break;
      }
    }
  }

  // Same number of parameters but some of them are of a different class
  if (param_map.size() != first_action.getParameters().size())
    return false;

  bool same_preconditions = true;
  for (const auto& second_prec : second_action.getPreconditions()) {
    std::vector<std::string> args;
    for (const auto& a : second_prec.predicate.args) {
      args.push_back(param_map.at(a));
    }
    Command t_cmd{{second_prec.predicate.name, args}, second_prec.negate};
    same_preconditions &= first_action.hasPrecondition(t_cmd);
    if (!same_preconditions)
      break;
  }

  // Different preconditions
  if (!same_preconditions)
    return false;

  bool same_effects = true;
  for (const auto& second_eff : second_action.getEffects()) {
    std::vector<std::string> args;
    for (const auto& a : second_eff.predicate.args) {
      args.push_back(param_map.at(a));
    }
    Command t_cmd{{second_eff.predicate.name, args}, second_eff.negate};
    same_effects &= first_action.hasEffect(t_cmd);
    if (!same_effects)
      break;
  }

  // Different effects
  if (!same_effects)
    return false;

  return true;
}
