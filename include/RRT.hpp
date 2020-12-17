//
// Created by xcy on 2020/12/15.
//

#ifndef NURSINGROBOT_RRT_HPP
#define NURSINGROBOT_RRT_HPP
#include "Planner.hpp"
#include <random>
namespace planner{
    template <typename T>
    class RRT {
    protected:
        Vertex<T>* _head;
        Vertex<T>* _tail;

        std::deque<Vertex<T>> _nodes;

        std::unordered_map<T, Vertex<T>*, std::function<size_t(T)>> _node_map;

        T _start,_goal;

        bool _forward;

        const int _dimensions;

        int _iter_max{};

        double _step_len{},_max_step_len{};

        double _goal_bias{};

        double _goal_max_dist{};

        double _waypoint_bias{};

        bool _is_ASC_enabled{};

        flann::Index<flann::L2_Simple<double>> _kd_tree;

        std::function<T(double*)> _arrayToT;

        std::function<void(T, double*)> TToArray_;


        virtual Vertex<T> _nearest(const T & current_state,double* distance_out)
        {
            Vertex<T>* best = nullptr;
            //K-NN search
            flann::Matrix<double> query;
            if(NULL == this->TToArray_)
            {
                query = flann::Matrix<double>((double*)&current_state, 1,
                                              sizeof(current_state) / sizeof(0.0));
            }
            else
            {
                Eigen::VectorXd data(this->_dimensions);
                this->TToArray_(current_state, data.data());
                query = flann::Matrix<double>(data.data(), 1,
                                              sizeof(current_state) / sizeof(0.0));
            }
            std::vector<int> i(query.rows);
            flann::Matrix<int> indices(i.data(), query.rows, 1);
            std::vector<double> d(query.rows);
            flann::Matrix<double> dists(d.data(), query.rows, 1);
            int n =
                    this->_kd_tree.knnSearch(query, indices, dists, 1, flann::SearchParams());

            if (distance_out)
                *distance_out = distance(current_state, best->state());
            T point;
            if (NULL == this->_arrayToT) {
                point = (T)this->_kd_tree.getPoint(indices[0][0]);
            } else {
                point = _arrayToT(this->_kd_tree.getPoint(indices[0][0]));
            }
            return this->_node_map[point];
        }
        virtual Vertex<T> _steer(const T & rand_state,Vertex<T>* source)
        {
            if(!source)
            {
                source = _nearest(rand_state, nullptr);
                if(!source) return nullptr;
            }
            //TODO: adaptive step size control
            T intermediate_state = interpolate(source->state(),rand_state,this->StepLen());
            T from = this->_forward ? source->state() : intermediate_state,
                    to = this->_forward ? intermediate_state: source->state();
            if(!_collision_check(from,to)) return nullptr;

            this->_nodes.emplace_back(intermediate_state,source,this->_dimensions,this->_TtoArray);
            this->_kd_tree.addPoints(flann::Matrix<double>(this->_nodes.back().coordinates()->data(), 1, this->_dimensions));
            this->_node_map.insert(std::pair<T,Vertex<T>*>(intermediate_state,&this->_nodes.back()));
            return &this->_nodes.back();
        }
        bool _isGoalReached(Vertex<T>* node_end)
        {
            return planner::distance(node_end->state() - _goal) < _goal_max_dist;
        };

        virtual T _sample()
        {
            //TODO: sample with in bounds;
            return randomState<T>(nullptr);
        }

        virtual bool _collision_check(){return true;};

        virtual void _extract_path(std::vector<T> &vectorOut, bool reverse)
        {
            const Vertex<T>* vertex = LastVertex();
            if(reverse)
            {
                while(vertex)
                {
                    vectorOut.emplace_back(vertex->state());
                    vertex = vertex->parent();
                }
            }
            else
            {
                std::vector<const Vertex<T>* > vertexes;
                while(vertex)
                {
                    _nodes.push_back(vertex);
                    vertex = vertex->parent();
                }
                for (auto itr = vertexes.rbegin(); itr != vertexes.rend(); itr++)
                {
                    vectorOut.emplace_back(vertex->state());
                }
            }
        }



    public:
        RRT(const RRT<T> &) = delete;
        RRT& operator=(const RRT<T> &) = delete;
        RRT(std::function<size_t(T)> hashT, int dimensions, bool forward = true,
            std::function<T(double*)> arrayToT = NULL,
            std::function<void(T, double*)> TToArray = NULL)
            :_kd_tree(flann::KDTreeSingleIndexParams()),
            _dimensions(dimensions),
            _node_map(20, hashT)
        {
            _forward = forward;
            _arrayToT = arrayToT;
            TToArray = TToArray;

            setStepLen(0.1);
            setMaxStepLen(5);
            setMaxIterations(1000);
            setGoalBias(0);
            setGoalMaxDist(0.1);
        };
        virtual ~RRT()=default;

        int MaxIterations() const { return _iter_max; };
        void setMaxIterations(int itr) { _iter_max = itr; };


        double GoalBias() const { return _goal_bias; };
        void setGoalBias(double goalBias) {
            if (goalBias < 0 || goalBias > 1) {
                throw std::invalid_argument(
                        "The goal bias must be a number between 0.0 and 1.0");
            }
            _goal_bias = goalBias;
        };

        double StepLen() const { return _step_len; }
        void setStepLen(double stepSize) { _step_len = stepSize; }

        double MaxStepLen() const { return _max_step_len; }
        void setMaxStepLen(double maxStep) { _max_step_len = maxStep; }

        double GoalMaxDist() const { return _goal_max_dist; }
        void setGoalMaxDist(double maxDist) { _goal_max_dist = maxDist; }

        bool isASCEnabled() const { return _is_ASC_enabled; }
        void setASCEnabled(bool checked) { _is_ASC_enabled = checked; }


        const Vertex<T>* RootVertex() const {
            if (_nodes.empty()) return nullptr;

            return &_nodes.front();
        }


        const Vertex<T>* LastVertex() const {
            if (_nodes.empty()) return nullptr;

            return &_nodes.back();
        }

        const T& GoalState() const { return _goal; }
        void setGoalState(const T& goalState) { _goal = goalState; }

        const T& startState() const {
            if (_nodes.empty())
                throw std::logic_error("No start state specified for RRT");
            else
                return RootVertex()->state();
        }
        void setStartState(const T& startState) {
            reset(true);

            //  create root node from provided start state
            _nodes.emplace_back(startState, nullptr, _dimensions, TToArray_);
            _node_map.insert(std::pair<T, Vertex<T>*>(startState, &_nodes.back()));
            if (TToArray_) {
                std::vector<double> data(_dimensions);
                _TToArray(RootVertex()->state(), data.data());
                _kd_tree.buildIndex(
                        flann::Matrix<double>(data.data(), 1, _dimensions));
            } else {
                _kd_tree.buildIndex(flann::Matrix<double>(
                        (double*)&(RootVertex()->state()), 1, _dimensions));
            }
        }

        void reset(bool eraseRoot = false)
        {
            _kd_tree = flann::Index<flann::L2_Simple<double>>(flann::KDTreeSingleIndexParams());
            if(eraseRoot)
            {
                _nodes.clear();
                _node_map.clear();
            }
            else if(_nodes.size() > 1)
            {
                T root = RootVertex()->state();
                _node_map.clear();
                _nodes.clear();
                _nodes.template emplace_back(root, nullptr, _dimensions, TToArray_);
                _node_map.template insert(std::pair<T, Vertex<T>*>(root,&_nodes.back()));
                if(TToArray_)
                {
                    Eigen::VectorXd data(_dimensions);
                    TToArray_(root,data.data());
                    _kd_tree.buildIndex(flann::Matrix<double>(data.data(), 1, _dimensions));
                }
                else
                {
                    _kd_tree.buildIndex(flann::Matrix<double>((double*)&(RootVertex()->state()), 1, _dimensions));
                }
            }
        }

        virtual bool planning()
        {
            for(int i=0; i<MaxIterations(); ++i)
            {
                Vertex<T>* new_vertex;
                double r = drand48();
                if(r < GoalBias())
                    new_vertex = _steer(GoalState());
                else
                    new_vertex = _steer(_sample());
                if (new_vertex && _isGoalReached(new_vertex->state()))
                    return true;
            }
            return false;
        }

        std::vector<T> GetPath(bool reverse)
        {
            std::vector<T> path;
            _extract_path(path,reverse);
            return path;
        }

    };
}



#endif //NURSINGROBOT_RRT_HPP
