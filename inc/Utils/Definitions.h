#pragma once
#ifndef UTILS_DEFINITIONS_H
#define UTILS_DEFINITIONS_H

#include <map>
#include <vector>
#include <array>
#include <list>
#include <iostream>
#include <limits>
#include <functional>
#include <memory>
#include <climits>
#include <unordered_map>
#include <list>
#include "boost/heap/pairing_heap.hpp"
#include "boost/heap/priority_queue.hpp"


#ifndef DEBUG
#define DEBUG 1
#endif


const size_t MAX_COST = std::numeric_limits<size_t>::max();


template<typename T>
using Pair      = std::array<T, 2>;

template<typename T>
std::ostream& operator<<(std::ostream &stream, const Pair<T> pair) {
    stream << "[" << pair[0] << ", " << pair[1] << "]";
    return stream;
}


using Heuristic = std::function<std::vector<size_t>(size_t)>;

// Structs and classes
struct Edge {
    size_t          source;
    size_t          target;
    std::vector<size_t>    cost;

    Edge(size_t source, size_t target, std::vector<size_t> cost) : source(source), target(target), cost(cost) {}
    Edge inverse() {
        return Edge(this->target, this->source, this->cost);
    }
};
std::ostream& operator<<(std::ostream &stream, const Edge &edge);


// Graph representation as adjacency matrix
class AdjacencyMatrix {
private:
    std::vector<std::vector<Edge>> matrix;
    size_t                         graph_size;
    size_t num_of_objectives = 0;

public:
    AdjacencyMatrix() = default;
    AdjacencyMatrix(size_t graph_size, std::vector<Edge> &edges, bool inverse=false);
    void add(Edge edge);
    size_t size(void) const;
    size_t get_num_of_objectives() const;
    const std::vector<Edge>& operator[](size_t vertex_id) const;
  
    friend std::ostream& operator<<(std::ostream &stream, const AdjacencyMatrix &adj_matrix);
};


struct Node;
struct PathPair;
struct ApexPathPair;
using NodePtr       = std::shared_ptr<Node>;
using PathPairPtr   = std::shared_ptr<PathPair>;
using ApexPathPairPtr   = std::shared_ptr<ApexPathPair>;
using SolutionSet   = std::vector<NodePtr>;
using PPSolutionSet = std::vector<PathPairPtr>;
using ApexPathSolutionSet = std::vector<ApexPathPairPtr>;


using EPS = std::vector<double>;

struct Node {
    size_t          id;
    std::vector<size_t>    g;
    std::vector<size_t>    h;
    std::vector<size_t>    f;
    NodePtr         parent;
    size_t t = 0;   // time

    Node(size_t id, std::vector<size_t> g, std::vector<size_t> h, NodePtr parent=nullptr)
        : id(id), g(g), h(h), f(g.size()), parent(parent) {
        for (int i = 0; i < g.size(); i++){
            f[i] = g[i] + h[i];
        }
    };

    Node(size_t id, std::vector<size_t> g, std::vector<size_t> h, size_t t, NodePtr parent=nullptr)
        : id(id), g(g), h(h), f(g.size()), parent(parent), t(t) {
        for (int i = 0; i < g.size(); i++){
            f[i] = g[i] + h[i];
        }
    };

    struct more_than_specific_heurisitic_cost {
        size_t cost_idx;

        more_than_specific_heurisitic_cost(size_t cost_idx) : cost_idx(cost_idx) {};
        bool operator()(const NodePtr &a, const NodePtr &b) const;
    };

    struct more_than_combined_heurisitic {
        double factor;

        more_than_combined_heurisitic(double factor) : factor(factor) {};
        bool operator()(const NodePtr &a, const NodePtr &b) const;
    };


    struct more_than_full_cost {
        bool operator()(const NodePtr &a, const NodePtr &b) const;
    };

    enum LEX_ORDER {LEX0, LEX1};
    struct more_than_lex{
        Node::LEX_ORDER order;
        more_than_lex(Node::LEX_ORDER order) : order(order) {};
        bool operator()(const NodePtr &a, const NodePtr &b) const;
    };


    struct compare_lex1
    {
        bool operator()(const NodePtr n1, const NodePtr n2) const
        {
            if (n1->f[0] != n2->f[0]){
                return n1->f[0] > n2->f[0];
            }
            return n1->f[1] > n2->f[1];
        }
    };
    friend std::ostream& operator<<(std::ostream &stream, const Node &node);
};

struct PathPair {
    size_t      id;
    NodePtr     top_left;
    NodePtr     bottom_right;
    NodePtr     parent;
    bool        is_active=true;

    PathPair(const NodePtr &top_left, const NodePtr &bottom_right)
        : id(top_left->id), top_left(top_left), bottom_right(bottom_right), parent(top_left->parent) {};

    bool update_nodes_by_merge_if_bounded(const PathPairPtr &other, const Pair<double> eps);
    bool update_nodes_by_merge_if_bounded_keep_track(const PathPairPtr &other, const Pair<double> eps, std::list<NodePtr>& pruned_list);
    bool update_nodes_by_merge_if_bounded2(const PathPairPtr &other, const Pair<double> eps);

    bool if_merge_bounded(const PathPairPtr &other, const Pair<double> eps)  const;


    struct more_than_full_cost {
 bool operator()(const PathPairPtr &a, const PathPairPtr &b) const;
    };

    friend std::ostream& operator<<(std::ostream &stream, const PathPair &pp);
};

enum MergeStrategy {SMALLER_G2, RANDOM, MORE_SLACK, SMALLER_G2_FIRST, REVERSE_LEX};

struct ApexPathPair {
    size_t      id; // state of the node
    NodePtr     apex;
    NodePtr     path_node;
    NodePtr     parent;
    bool        is_active=true;
    size_t      t;

    Heuristic& h;

    ApexPathPair(const NodePtr &apex, const NodePtr &path_node, Heuristic& h)
        : apex(apex), path_node(path_node) , parent(path_node->parent), h(h), id(apex->id), t(t){};

    ApexPathPair(const ApexPathPairPtr parent, const Edge& egde);


    bool update_nodes_by_merge_if_bounded(const ApexPathPairPtr &other, const EPS eps, MergeStrategy s=MergeStrategy::SMALLER_G2);
    bool update_apex_by_merge_if_bounded(const NodePtr &other_apex, const EPS eps);

    // bool if_merge_bounded(const ApexPathPairPtr &other, const EP S eps)  const;


    struct more_than_full_cost {
        bool operator()(const ApexPathPairPtr &a, const ApexPathPairPtr &b) const;
    };

    friend std::ostream& operator<<(std::ostream &stream, const ApexPathPair &pp);
};

bool is_bounded(NodePtr apex, NodePtr node,  const EPS eps);
bool is_bounded(NodePtr apex, NodePtr node);
bool is_dominated_dr(NodePtr apex, NodePtr node);
bool is_dominated_dr(NodePtr apex, NodePtr node, const EPS eps);

class Interval{
public:
    double eps = 0;
    NodePtr top_left;
    NodePtr bottom_right;
    std::shared_ptr<std::list<NodePtr>> to_expand;

    Interval(){};
    Interval(const NodePtr top_left, const NodePtr bottom_right, std::shared_ptr<std::list<NodePtr>> to_expand);
};

std::ostream& operator<<(std::ostream& os, const Interval& interval);


using IntervalList   = std::vector<Interval>;

typedef boost::heap::priority_queue<NodePtr , boost::heap::compare<Node::compare_lex1> > heap_open_t;


/*******************N E W*******************/
// struct IndividualPath;
// struct JointPath;
class HighLevelNode;
// using JointPathPtr = std::shared_ptr<JointPath>;
using HighLevelNodePtr = std::shared_ptr<HighLevelNode>;
using HSolutionID = std::vector<std::vector<std::vector<size_t>>>;
// using HighLevelSolutionCost = std::vector<std::vector<size_t>>;
using IndividualConstraintSet = std::unordered_map<size_t, std::vector<size_t>>;
using ConstraintSet = std::vector<IndividualConstraintSet>;
using CostVector = std::vector<size_t>;
using PathSet = std::unordered_map<size_t, std::vector<size_t>>;
using CostSet = std::unordered_map<size_t, std::vector<size_t>>;
using JointPathPair = std::pair<CostVector, std::vector<size_t>>;
using JointPathTuple = std::tuple<CostVector, CostVector, std::vector<size_t>>;

inline void add_cost(CostVector& a, const CostVector& b)
{
    assert(a.size() == b.size());
    for(int i = 0; i < a.size(); i ++){
        a.at(i) += b.at(i);
    }
}
// inline CostVector add_cost(CostVector& a, CostVector& b)
// {
//     assert(a.size() == b.size());
//     for(int i = 0; i < a.size(); i ++){
//         a.at(i) += b.at(i);
//     }
// }

// struct IndividualPath
// {
//     CostVector      cost_vector;
//     std::vector<size_t>     node_id;
// };

// struct JointPath
// {
//     std::vector<ApexPathPairPtr>    ap_list;
//     CostVector      cost;
//     CostVector      apex_cost;
    
// //  constructor
//     JointPath(){};
//     JointPath(std::vector<ApexPathPairPtr> solution_list);
//     JointPath(JointPathPtr, ApexPathPairPtr);
// };

class HighLevelNode
{
public:
    size_t  id = 0;     //  no real sense
    std::vector<PathSet>                indiv_paths_list;
    std::vector<CostSet>                indiv_apex_costs;
    std::vector<CostSet>                indiv_real_costs;
    std::vector<size_t>                 rep_id_list;
    CostVector                          rep_apex_cost;
    ConstraintSet                       constraints;
    std::list<JointPathPair>            joint_path_list;
    // std::vector<JointPathPtr>           all_joint_path;
    // JointPathPtr                        rep_path;   // representative path id in Pareto-optimal set
    // std::vector<ApexPathSolutionSet>    all_solutions;

    struct more_than_full_cost{
        bool operator()(HighLevelNodePtr& a, HighLevelNodePtr& b);
    };

//  constructor
    HighLevelNode(size_t agent_num) : indiv_paths_list(std::vector<PathSet>(agent_num)), 
            indiv_apex_costs(std::vector<CostSet>(agent_num)), indiv_real_costs(std::vector<CostSet>(agent_num)),
            rep_id_list(std::vector<size_t>(agent_num)), constraints(ConstraintSet(agent_num)){};
    // HighLevelNode(const HighLevelNode& node);
};

#endif //UTILS_DEFINITIONS_H