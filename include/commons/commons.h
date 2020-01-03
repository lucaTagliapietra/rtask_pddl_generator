#ifndef rtask_commons_commons_h
#define rtask_commons_commons_h

#include <string>
#include <vector>

#include "rtask_msgs/Command.h"
#include "rtask_msgs/Entity.h"
#include "rtask_msgs/Predicate.h"
#include "rtask_msgs/SemanticEntity.h"

namespace rtask {
  namespace commons {

    const std::string NULL_TYPE = "null_type";

    struct SemanticEntity
    {
      std::vector<std::string> symbols;
      std::string type{};
      std::string semantic_class{};

      SemanticEntity() = default;
      SemanticEntity(const std::vector<std::string>& t_symbols, const std::string& t_type, const std::string& t_class)
        : symbols(t_symbols)
        , type(t_type)
        , semantic_class(t_class)
      {}

      SemanticEntity(const rtask_msgs::SemanticEntity& t_msg)
        : type(t_msg.type)
        , semantic_class(t_msg.semantic_class)
      {
        for (const auto& symbol : t_msg.symbols) {
          symbols.push_back({symbol});
        }
      }

      bool operator==(const SemanticEntity& t_other) const
      {

        bool eq = (symbols.size() == t_other.symbols.size()) && (type == t_other.type)
                  && (semantic_class == t_other.semantic_class);
        if (eq) {
          for (unsigned int i = 0; i < symbols.size(); ++i) {
            eq &= symbols[i] == t_other.symbols[i];
          }
        }
        return eq;
      }

      bool operator!=(const SemanticEntity& t_other) { return !operator==(t_other); }

      rtask_msgs::SemanticEntity toMsg() const
      {
        rtask_msgs::SemanticEntity msg;
        for (const auto& symbol : symbols) {
          msg.symbols.emplace_back(symbol);
        }

        msg.type = type;
        msg.semantic_class = semantic_class;

        return msg;
      }
    };

    struct Entity
    {
      SemanticEntity semantic_entity{};
      std::vector<std::string> properties{};

      Entity() = default;
      Entity(const SemanticEntity& t_semantic_entity, const std::vector<std::string>& t_properties)
        : semantic_entity(t_semantic_entity)
        , properties(t_properties)
      {}

      Entity(const rtask_msgs::Entity& t_msg)
        : semantic_entity(t_msg.semantic_entity)
      {
        for (const auto& property : t_msg.properties) {
          properties.push_back({property});
        }
      }

      bool operator==(const Entity& t_other) const
      {

        bool eq = (semantic_entity == t_other.semantic_entity) && (properties.size() == t_other.properties.size());

        if (eq) {
          for (unsigned int i = 0; i < properties.size(); ++i) {
            eq &= properties[i] == t_other.properties[i];
          }
        }
        return eq;
      }

      bool operator!=(const Entity& t_other) { return !operator==(t_other); }

      rtask_msgs::Entity toMsg() const
      {
        rtask_msgs::Entity msg;

        msg.semantic_entity = semantic_entity.toMsg();

        for (const auto& property : properties) {
          msg.properties.emplace_back(property);
        }

        return msg;
      }
    };

    struct Predicate
    {
      std::string name{};
      std::vector<std::string> args{};

      Predicate() = default;
      Predicate(const std::string& t_name, const std::vector<std::string>& t_args)
        : name(t_name)
        , args(t_args)
      {}

      Predicate(const rtask_msgs::Predicate& t_msg)
        : name(t_msg.name)
      {
        for (const auto& arg : t_msg.args) {
          args.push_back({arg});
        }
      }

      bool operator==(const Predicate& t_other) const
      {
        bool eq = (name == t_other.name) && (args.size() == t_other.args.size());
        if (eq) {
          for (unsigned int i = 0; i < args.size(); ++i) {
            eq &= args[i] == t_other.args[i];
          }
        }
        return eq;
      }

      bool operator!=(const Predicate& t_other) { return !operator==(t_other); }

      rtask_msgs::Predicate toMsg() const
      {
        rtask_msgs::Predicate msg;
        msg.name = name;
        for (const auto& arg : args) {
          msg.args.emplace_back(arg);
        }
        return msg;
      }
    };

    struct Command
    {
      Predicate predicate{};
      bool negate = false;

      Command() = default;
      Command(const Predicate& t_pred, const bool t_negate)
        : predicate(t_pred)
        , negate(t_negate)
      {}

      Command(const rtask_msgs::Command& t_msg)
        : predicate(t_msg.predicate)
        , negate(t_msg.negate)
      {}

      bool operator==(const Command& t_other) const
      {
        return (predicate == t_other.predicate && (negate == t_other.negate));
      }
      bool operator!=(const Command& t_other) { return !operator==(t_other); }

      rtask_msgs::Command toMsg() const
      {
        rtask_msgs::Command msg;
        msg.predicate = predicate.toMsg();
        msg.negate = negate;
        return msg;
      }
    };

  } // namespace commons
} // namespace rtask

#endif
