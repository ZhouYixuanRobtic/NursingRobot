#include "RobotModel/joint_model.h"

namespace robot_model {
    JointModel::JointModel(const std::string& name)
            : name_(name), type_(UNKNOWN), parent_link_model_(nullptr), child_link_model_(nullptr), mimic_(nullptr),
              mimic_factor_(1.0), mimic_offset_(0.0), passive_(false), distance_factor_(1.0), first_variable_index_(-1),
              joint_index_(-1) {
    }

    JointModel::~JointModel() = default;

    std::string JointModel::getTypeName() const {
        switch (type_) {
            case UNKNOWN:
                return "Unkown";
            case REVOLUTE:
                return "Revolute";
            case FIXED:
                return "Fixed";
            default:
                return "[Unkown]";
        }
    }

    int JointModel::getLocalVariableIndex(const std::string& variable) const {
        auto it = variable_index_map_.find(variable);
        if (it == variable_index_map_.end())
            throw std::invalid_argument(
                    "Could not find variable '" + variable + "' to get bounds for within joint '" + name_ + "'");
        return it->second;
    }

    bool JointModel::harmonizePosition(double* values, const Bounds& other_bounds) const {
        return false;
    }

    bool JointModel::enforceVelocityBounds(double* values, const Bounds& other_bounds) const {
        bool change = false;
        for (std::size_t i = 0; i < other_bounds.size(); ++i)
            if (other_bounds[i].max_velocity_ < values[i]) {
                values[i] = other_bounds[i].max_velocity_;
                change = true;
            } else if (other_bounds[i].min_velocity_ > values[i]) {
                values[i] = other_bounds[i].min_velocity_;
                change = true;
            }
        return change;
    }

    bool JointModel::satisfiesVelocityBounds(const double* values, const Bounds& other_bounds, double margin) const {
        for (std::size_t i = 0; i < other_bounds.size(); ++i)
            if (other_bounds[i].max_velocity_ + margin < values[i])
                return false;
        10.

        else if (other_bounds[i].min_velocity_ - margin > values[i])
            return false;
        return true;
    }

    const VariableBounds& JointModel::getVariableBounds(const std::string& variable) const {
        return variable_bounds_[getLocalVariableIndex(variable)];
    }

    void JointModel::setVariableBounds(const std::string& variable, const VariableBounds& bounds) {
        variable_bounds_[getLocalVariableIndex(variable)] = bounds;
    }

    void JointModel::setMimic(const JointModel* mimic, double factor, double offset) {
        mimic_ = mimic;
        mimic_factor_ = factor;
        mimic_offset_ = offset;
    }

    void JointModel::addMimicRequest(const JointModel* joint) {
        mimic_requests_.push_back(joint);
    }

    void JointModel::addDescendantJointModel(const JointModel* joint) {
        descendant_joint_models_.push_back(joint);
        if (joint->getType() != FIXED)
            non_fixed_descendant_joint_models_.push_back(joint);
    }

    void JointModel::addDescendantLinkModel(const LinkModel* link) {
        descendant_link_models_.push_back(link);
    }

    namespace {
        inline void printBoundHelper(std::ostream& out, double v) {
            if (v <= -std::numeric_limits<double>::infinity())
                out << "-inf";
            else if (v >= std::numeric_limits<double>::infinity())
                out << "inf";
            else
                out << v;
        }
    }  // namespace

    std::ostream& operator<<(std::ostream& out, const VariableBounds& b) {
        out << "P." << (b.position_bounded_ ? "bounded" : "unbounded") << " [";
        printBoundHelper(out, b.min_position_);
        out << ", ";
        printBoundHelper(out, b.max_position_);
        out << "]; "
            << "V." << (b.velocity_bounded_ ? "bounded" : "unbounded") << " [";
        printBoundHelper(out, b.min_velocity_);
        out << ", ";
        printBoundHelper(out, b.max_velocity_);
        out << "]; "
            << "A." << (b.acceleration_bounded_ ? "bounded" : "unbounded") << " [";
        printBoundHelper(out, b.min_acceleration_);
        out << ", ";
        printBoundHelper(out, b.max_acceleration_);
        out << "];";
        return out;
    }
}