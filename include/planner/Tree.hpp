#ifndef NURSINGROBOT_TREE_HPP
#define NURSINGROBOT_TREE_HPP

#include "planner/Planner.hpp"
namespace planner{
    template<typename T, typename Distance = flann::L1<double>>
    class Tree {
    protected:
        std::deque<Vertex<T>> _nodes;

        std::unordered_map<T,Vertex<T>*, std::function<size_t(T)>> _node_map;

        std::unordered_map<Vertex<T> *, std::size_t> _kd_tree_hash_map;

        std::size_t  _dimensions;

        std::shared_ptr<flann::Index<Distance>> _kd_tree{};

        std::function<T(double *)> _arrayToT;

        std::function<void(T,double *)> TToArray_;
    public:
        Tree(const Tree<T,Distance> &) = default;
        Tree &operator=(const Tree<T,Distance> &) = default;

        Tree(std::function<size_t(T)> hashT, std::size_t dimensions,
             std::function<T(double *)> arrayToT = NULL,
             std::function<void(T, double *)> TToArray = NULL)
             :_dimensions(dimensions),
             _node_map(20,hashT),
             _kd_tree(std::make_shared<flann::Index<Distance>>(flann::KDTreeSingleIndexParams())),
             _arrayToT(arrayToT),
             TToArray_(TToArray)
        {

        };
        virtual ~Tree() = default;

        //return the root of the tree
        Vertex <T> *RootVertex() const
        {
            if (_nodes.empty()) return nullptr;

            return &_nodes.front();
        }
        Vertex <T> *LastVertex() const
        {
            if (_nodes.empty()) return nullptr;

            return &_nodes.back();
        }

        void reset(bool eraseRoot = false)
        {
            _kd_tree.reset(new flann::Index<Distance>(flann::KDTreeSingleIndexParams()));
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
                _kd_tree_hash_map.template insert(std::make_pair(&_nodes.back(), 0));
            }
        }

        void setStartState(const T &startState)
        {
            reset(true);
            // create root node from provided start state
            _nodes.template emplace_back(startState, nullptr, _dimensions, TToArray_);
            _node_map.insert(std::pair<T, Vertex<T> *>(startState, &_nodes.back()));
            _kd_tree->buildIndex(flann::Matrix<double>(RootVertex()->data(), 1, _dimensions));
            _kd_tree_hash_map.template insert(std::make_pair(&_nodes.back(), 0));
        }

        virtual Vertex <T> *nearest(const T &current_state, double *distance_out)
        {
            //K-NN search
            std::vector<double> data(_dimensions);
            flann::Matrix<double> query;
            if (NULL == TToArray_) {
                memcpy(data.data(), current_state.data(), _dimensions * sizeof(double));
                query = flann::Matrix<double>(data.data(), 1, _dimensions);
            } else {
                TToArray_(current_state, data.data());
                query = flann::Matrix<double>(data.data(), 1, _dimensions);
            }
            std::vector<std::vector<size_t>> indices(1);
            std::vector<std::vector<double>> dists(1);
            _kd_tree->knnSearch(query, indices, dists, 1, flann::SearchParams());
            if (distance_out)
                *distance_out = dists[0][0];
            T point;
            if (NULL == _arrayToT) {
                point = T(_kd_tree->getPoint(indices[0][0]), _dimensions);
            } else {
                point = _arrayToT(_kd_tree->getPoint(indices[0][0]));
            }
            return _node_map[point];
        }


        virtual void addState(const T& state, Vertex<T> *  parent)
        {
            _nodes.template emplace_back(state, parent, _dimensions, TToArray_);
            _kd_tree->addPoints(flann::Matrix<double>(_nodes.back().data(), 1, _dimensions));
            _kd_tree_hash_map.template insert(std::make_pair(&_nodes.back(), _kd_tree_hash_map.size()));
            _node_map.insert(std::pair<T, Vertex<T> *>(state, &_nodes.back()));
        }

        void _extract_path(std::vector<Vertex < T> *> &vertex_vector, const Vertex<T> * tail)
        {
            vertex_vector.clear();
            Vertex <T> *vertex = tail;
            while (vertex) {
                vertex_vector.template emplace_back(vertex);
                vertex = vertex->parent();
            }
        }

        virtual void extract_path(std::vector<T> &vectorOut, const Vertex<T> * tail,bool reverse)
        {
            vectorOut.clear();
            std::vector<Vertex<T> *> vertex_vector;
            _extract_path(vertex_vector,);
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
    };
}






#endif //NURSINGROBOT_TREE_HPP
