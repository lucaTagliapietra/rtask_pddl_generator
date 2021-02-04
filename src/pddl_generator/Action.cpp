#include "pddl_generator/Action.h"
#include "pddl_generator/Helpers.h"

using namespace rtask::commons::pddl_generator;

Action::Action(const std::string& t_name,
               const LiteralTermVector& t_params,
               LogicalExprPtr t_precondition,
               LogicalExprPtr t_effect)
{
  set(t_name, t_params, t_precondition, t_effect);
}

Action::Action(XmlRpc::XmlRpcValue& t_rpc_val)
{
  if (!(t_rpc_val.getType() == XmlRpc::XmlRpcValue::Type::TypeStruct
        && helpers::checkXmlRpcSanity("name", t_rpc_val, XmlRpc::XmlRpcValue::Type::TypeString, false)
        && helpers::checkXmlRpcSanity("params", t_rpc_val, XmlRpc::XmlRpcValue::Type::TypeArray)
        && helpers::checkXmlRpcSanity("precondition", t_rpc_val, XmlRpc::XmlRpcValue::Type::TypeStruct)
        && helpers::checkXmlRpcSanity("effect", t_rpc_val, XmlRpc::XmlRpcValue::Type::TypeStruct))) {
    std::cerr
      << "Fatal: Invalid Action Structure, should be a Struct with name, params, precondition, and effect fields"
      << std::endl;
    exit(EXIT_FAILURE);
  }

  name_ = static_cast<std::string>(t_rpc_val["name"]);

  if (t_rpc_val["params"].size() == 0) {
    std::cout << "Empty params vector for Action " << name_ << std::endl;
  }
  else {
    for (int i = 0; i < t_rpc_val["params"].size(); ++i) {
      params_.emplace_back(t_rpc_val["params"][i]);
    }
  }

  precondition_ = helpers::getLogicalExprFromXmlRpc(t_rpc_val["precondition"]);
  if (!precondition_) {
    std::cerr << "Fatal: Invalid Precondition of current Action: " << name_ << std::endl;
    exit(EXIT_FAILURE);
  }

  effect_ = helpers::getLogicalExprFromXmlRpc(t_rpc_val["effect"]);
  if (!effect_) {
    std::cerr << "Fatal: Invalid Effect of current Action: " << name_ << std::endl;
    exit(EXIT_FAILURE);
  }
}

void Action::clear()
{
  name_.clear();
  params_.clear();
  precondition_.reset();
  effect_.reset();
}

bool Action::set(const std::string& t_name,
                 const LiteralTermVector& t_params,
                 LogicalExprPtr t_precondition,
                 LogicalExprPtr t_effect)
{
  setName(t_name);
  setParameters(t_params);
  setPrecondition(t_precondition);
  setEffect(t_effect);
  return true;
}

bool Action::setName(const std::string& t_name)
{

  if (t_name.empty()) {
    std::cerr << "Fatal: Empty name for current Action: " << t_name << std::endl;
    return false;
  }
  name_ = t_name;
  return true;
}

Action& Action::operator=(const Action& t_other)
{
  set(t_other.getName(), t_other.getParameters(), t_other.getPrecondition(), t_other.getEffect());
  return *this;
}

std::string Action::toPddl(bool t_typing, int t_pad_lv) const
{
  auto pad_aligners = helpers::getPddlAligners(t_pad_lv);
  const auto& pad_lv = pad_aligners.first;
  const auto& aligners = pad_aligners.second;

  std::string out = aligners[0] + "(" + ":action " + name_ + aligners[1];

  auto pad_aligners_n = helpers::getPddlAligners(pad_lv);
  const auto& pad_lv_n = pad_aligners_n.first;
  const auto& aligners_n = pad_aligners_n.second;

  out += aligners_n[0] + ":parameters (";
  for (const auto& arg : params_) {
    out += arg.toPddl(t_typing) + " ";
  }
  out.pop_back();
  out += ")";

  out += aligners_n[1] + aligners_n[0] + ":precondition" + aligners[1];
  out += precondition_->toPddl(t_typing, pad_lv_n);

  out += aligners_n[1] + aligners_n[0] + ":effect" + aligners[1];
  out += effect_->toPddl(t_typing, pad_lv_n);
  out += aligners_n[2] + ")";

  out += aligners[2] + ")";
  return out;
}

bool rtask::commons::pddl_generator::operator==(const Action& t_first, const Action& t_second)
{
  if (t_first.getName() != t_second.getName() || t_first.getNumParameters() != t_second.getNumParameters()) {
    return false;
  }

  const auto& f_params = t_first.getParameters();
  const auto& s_params = t_second.getParameters();
  for (unsigned int ix = 0; ix < f_params.size(); ++ix) {
    if (f_params.at(ix).getType() != s_params.at(ix).getType())
      return false;
  }

  if (!(t_first.getPrecondition().get() == t_second.getPrecondition().get()
        && !(t_first.getEffect().get() == t_second.getEffect().get()))) {
    return false;
  }
  return true;
}

std::ostream& rtask::commons::pddl_generator::operator<<(std::ostream& t_out, const Action& t_act)
{
  t_out << "Action: " << t_act.getName() << std::endl;
  unsigned int i = 0;
  for (const auto& param : t_act.getParameters()) {
    t_out << " - Param[" << i++ << "] : " << param << std::endl;
  }
  t_out << " - Precondition" << t_act.getPrecondition() << std::endl;
  t_out << " - Effect" << t_act.getPrecondition() << std::endl;
  return t_out;
}

std::ostream& rtask::commons::pddl_generator::operator<<(std::ostream& t_out, std::shared_ptr<Action> t_expr_ptr)
{
  t_out << (t_expr_ptr ? *t_expr_ptr : Action());
  return t_out;
}
