#ifndef NURSINGROBOT_RRT_HPP
#define NURSINGROBOT_RRT_HPP

#include "planner/Planner.hpp"
#include <random>

namespace planner {
    enum PLANNER_TYPE {
        RRT_SIMPLE,
        RRT_CONNECT,
        LAZY_RRT,
        RRT_STAR,
        INFORMED_RRT_STAR
    };

    template<typename T>
    class RRT;

    template<typename T>
    struct PLAN_REQUEST {
    public:
        PLAN_REQUEST(const T &start, const T &goal, double time_limit = 5,
                     double step_len = 0.05, bool is_step_relative=false, double max_goal_dist =0.05)
                : _start(start),
                  _goal(goal),
                  _time_limit(time_limit),
                  _step_len(step_len),
                  _is_step_relative(is_step_relative),
                  _max_goal_dist(max_goal_dist)

        {

        };

        friend RRT<T>;
    private:
        T _start;
        T _goal;
        double _time_limit;
        double _step_len;
        double _max_goal_dist;
        bool _is_step_relative;
    };

    template<typename T>
    class RRT {
    protected:
        std::deque<Vertex < T>>
        _nodes;

        std::unordered_map<T, Vertex < T> *, std::function<size_t(T)>>
        _node_map;

        std::unordered_map<Vertex<T> *, std::size_t> _kd_tree_hash_map;

        Vertex <T> *_tail;
        T _start, _goal;

        bool _forward;

        const std::size_t _dimensions;

        std::size_t _iter_max{};

        double _step_len{}, _max_step_len{};

        bool _is_step_relative{false};

        double _goal_bias{};

        double _goal_max_dist{};

        double _d_min{};

        double _time_limit{};

        bool _is_ASC_enabled{};

        const Eigen::MatrixX2d *_bounds_ptr;

        std::shared_ptr<flann::Index<flann::L2_Simple<double>>> _kd_tree{};

        std::function<T(double *)> _arrayToT;

        std::function<void(T, double *)> TToArray_;

        std::function<bool(const T &, const T &)> _state_validate_func;

        virtual Vertex <T> *_nearest(const T &current_state, double *distance_out)
        {
            //K-NN search
            std::vector<double> data(_dimensions);
            flann::Matrix<double> query;
            if (NULL == TToArray_) {
                memcpy(data.data(), current_state.data(), _dimensions * sizeof(current_state.Vector()[0]));
                query = flann::Matrix<double>(data.data(), 1, _dimensions);
            } else {
                TToArray_(current_state, data.data());
                query = flann::Matrix<double>(data.data(), 1, _dimensions);
            }
            std::vector<std::vector<size_t>> indices(1);
            std::vector<std::vector<double>> dists(1);
            _kd_tree->knnSearch(query, indices, dists, 1, flann::SearchParams());
            if (distance_out)
                *distance_out = std::sqrt(dists[0][0]);
            T point;
            if (NULL == _arrayToT) {
                point = T(_kd_tree->getPoint(indices[0][0]), _dimensions);
            } else {
                point = _arrayToT(_kd_tree->getPoint(indices[0][0]));
            }
            return _node_map[point];
        }

        virtual Vertex <T> *_steer(const T &rand_state, Vertex <T> *source,bool check_collision)
        {
            double distance{};
            if (!source) {
                source = _nearest(rand_state, &distance);
                if (!source) return nullptr;
            }
            if(distance > )
            double steer_length =
                    _is_step_relative ? std::min(distance / _d_min, this->StepLen()) * _d_min : std::min(distance,
                                                                                                         this->StepLen());
            T intermediate_state = planner::extend(source->state(), rand_state, steer_length);
            T from = _forward ? source->state() : intermediate_state;
            T to = _forward ? intermediate_state : source->state();
            if (!_isStateValid(from, to,check_collision)) return nullptr;

            _nodes.template emplace_back(intermediate_state, source, _dimensions, TToArray_);
            _kd_tree->addPoints(flann::Matrix<double>(_nodes.back().data(), 1, _dimensions));
            _kd_tree_hash_map.template insert(std::make_pair(&_nodes.back(),_kd_tree_hash_map.size()));
            _node_map.insert(std::pair<T, Vertex<T> *>(intermediate_state, &_nodes.back()));
            return &_nodes.back();
        }

        bool _isGoalReached(Vertex <T> *node_end)
        {
            return planner::distance(node_end->state(), _goal) < _goal_max_dist;
        };

        virtual T _sample()
        {
            std::shared_ptr<T> sample_ptr(std::make_shared<T>(randomState<T>(_dimensions, _bounds_ptr)));
            return *sample_ptr.get();
        }

        virtual bool _isStateValid(const T &from, const T &to, bool check_collision)
        {
            if (_state_validate_func && check_collision)
                return _state_validate_func(from, to);
            else
                return true;
        };
        void _extract_path(std::vector<Vertex<T> *> &vertex_vector)
        {
            vertex_vector.clear();
            Vertex<T> *vertex = _tail;
            while(vertex)
            {
                vertex_vector.template emplace_back(vertex);
                vertex = vertex->parent();
            }
        }
        virtual void _extract_path(std::vector<T> &vectorOut, bool reverse)
        {
            vectorOut.clear();
            std::vector<Vertex<T> *> vertex_vector;
            _extract_path(vertex_vector);
            if (reverse) {
                for (auto itr = vertex_vector.begin(); itr != vertex_vector.end(); itr++) {
                    vectorOut.template emplace_back((*itr)->state());
                }
            } else {
                for (auto itr = vertex_vector.rbegin(); itr != vertex_vector.rend(); itr++) {
                    vectorOut.template emplace_back((*itr)->state());
                }
            }
        }

        virtual bool argsCheck()
        {
            if (_is_step_relative && (_step_len > 1 || _step_len < 0)) {
                LOG(WARNING)
                        << "receive relative step request while have step len or max step len not match the range of [0,1]";
                return false;
            }
            if (_goal_bias < 0 || _goal_bias > 1) {
                LOG(WARNING) << "The goal bias must be a number between 0.0 and 1.0";
                return false;
            }
            return true;
        }

    public:

        RRT(const RRT<T> &) = delete;

        RRT &operator=(const RRT<T> &) = delete;

        RRT(std::function<size_t(T)> hashT, std::size_t dimensions, bool forward = true,
            std::function<T(double *)> arrayToT = NULL,
            std::function<void(T, double *)> TToArray = NULL)
                : _dimensions(dimensions),
                  _node_map(20, hashT),
                  _state_validate_func(NULL),
                  _d_min(0.1)
        {
            _bounds_ptr = nullptr;
            _kd_tree = std::make_shared<flann::Index<flann::L2_Simple<double>>>(flann::KDTreeSingleIndexParams());
            _forward = forward;
            _arrayToT = arrayToT;
            TToArray_ = TToArray;

            setStepLen(0.1);
            setMaxStepLen(5);
            setMaxIterations(100000);
            setGoalBias(0.1);
            setGoalMaxDist(_step_len);
        };

        virtual ~RRT() = default;

        std::size_t MaxIterations() const
        { return _iter_max; };

        void setMaxIterations(std::size_t itr)
        { _iter_max = itr; };


        double GoalBias() const
        { return _goal_bias; };

        void setGoalBias(double goalBias)
        {
            _goal_bias = goalBias;
        };

        double StepLen() const
        { return _step_len; }

        void setStepLen(double stepSize, bool isRelative = false)
        {
            _step_len = stepSize;
            _is_step_relative = isRelative;
        }

        bool isStepRelative() const
        { return _is_step_relative; }

        double MaxStepLen() const
        { return _max_step_len; }

        void setMaxStepLen(double maxStep)
        { _max_step_len = maxStep; }

        double GoalMaxDist() const
        { return _goal_max_dist; }

        void setGoalMaxDist(double maxDist)
        { _goal_max_dist = maxDist; }

        bool isASCEnabled() const
        { return _is_ASC_enabled; }

        void setASCEnabled(bool checked)
        { _is_ASC_enabled = checked; }

        void setStateValidator(std::function<bool(const T &, const T &)> state_validate_func)
        {
            _state_validate_func = state_validate_func;
        }

        const Vertex <T> *RootVertex() const
        {
            if (_nodes.empty()) return nullptr;

            return &_nodes.front();
        }


        const Vertex <T> *LastVertex() const
        {
            if (_nodes.empty()) return nullptr;

            return &_nodes.back();
        }

        T GoalState() const
        { return _goal; }

        void setGoalState(const T &goalState)
        { _goal = goalState; }

        Eigen::MatrixX2d SampleBounds() const
        { return *_bounds_ptr; }

        void setSampleBounds(const Eigen::MatrixX2d *bounds_ptr)
        { _bounds_ptr = bounds_ptr; }

        T startState() const
        {
            if (_nodes.empty())
                throw std::logic_error("No start state specified for RRT");
            else
                return RootVertex()->state();
        }

        void setStartState(const T &startState)
        {
            reset(true);
            // create root node from provided start state
            _nodes.template emplace_back(startState, nullptr, _dimensions, TToArray_);
            _node_map.insert(std::pair<T, Vertex<T> *>(startState, &_nodes.back()));
            _kd_tree->buildIndex(flann::Matrix<double>(RootVertex()->data(), 1, _dimensions));
            _kd_tree_hash_map.template insert(std::make_pair(&_nodes.back(),0));
            _start = startState;
        }

        void reset(bool eraseRoot = false)
        {
            _kd_tree.reset(new flann::Index<flann::L2_Simple<double>>(flann::KDTreeSingleIndexParams()));
            if (eraseRoot) {
                _nodes.clear();
                _node_map.clear();
                _kd_tree_hash_map.clear();
            } else if (_nodes.size() > 1) {
                //clear all node but left root
                T root = RootVertex()->state();
                _node_map.clear();
                _nodes.clear();
                _kd_tree_hash_map.clear();
                _nodes.template emplace_back(root, nullptr, _dimensions, TToArray_);
                _node_map.insert(std::pair<T, Vertex<T> *>(root, &_nodes.back()));
                _kd_tree->buildIndex(flann::Matrix<double>(RootVertex()->data(), 1, _dimensions));
                _kd_tree_hash_map.template insert(std::make_pair(&_nodes.back(),0));
            }
        }

        void constructPlan(const PLAN_REQUEST<T> &rrtRequest)
        {
            if (_isStateValid(rrtRequest._start, rrtRequest._start,true) &&
                _isStateValid(rrtRequest._goal, rrtRequest._goal,true)) {
                setStartState(rrtRequest._start);
                _step_len = rrtRequest._step_len;
                _goal_max_dist = rrtRequest._max_goal_dist;
                _is_step_relative = rrtRequest._is_step_relative;
                _goal = rrtRequest._goal;
                _d_min = planner::distance(_start, _goal);
                _time_limit = rrtRequest._time_limit;
            } else
                LOG(INFO) << "Start or goal is not valid, please reset";
        }

        virtual bool planning()
        {
            // validity check
            if(!argsCheck())
                return false;
            time_t start(clock());
            double time{};
            for (int i = 0; i < _iter_max; ++i) {
                time = (double) (clock() - start) / CLOCKS_PER_SEC;
                if (time >= _time_limit) {
                    LOG(ERROR) << "No path find within " << _time_limit << " seconds"
                               << " now iterates " << i << "times";
                    return false;
                }
                Vertex<T> *new_vertex;

                double r = fabs(planner::randomState<state_space::Rn>(1, nullptr)[0]);

                if (r < _goal_bias)
                    new_vertex = _steer(_goal, nullptr,true);
                else
                    new_vertex = _steer(_sample(), nullptr,true);
                if (new_vertex && _isGoalReached(new_vertex)) {
                    _tail = new_vertex;
                    return true;
                }

            }
            LOG(ERROR) << "No path find within " << _iter_max << " iterations";
            return false;
        }

        std::vector<T> GetPath(bool reverse = false)
        {
            std::vector<T> path;
            _extract_path(path, reverse);
            path.template emplace_back(_goal);
            return path;
        }

    };


}


#endif //NURSINGROBOT_RRT_HPP
