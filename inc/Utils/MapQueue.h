#pragma once

#include <set>
#include <vector>
#include <list>
#include "Utils/Definitions.h"

template<class Node, class more_than_full_cost>
class MapQueue
{
private:
    std::vector<Node>                heap;
    more_than_full_cost           more_than;

    std::vector<std::list<Node>>   open_map;

public:
    MapQueue(){}
    MapQueue(size_t graph_size);
    bool empty();
    // Node top();
    Node pop();
    void insert(Node &pp);
    std::list<Node> &get_open(size_t id);
    size_t size(){
        return heap.size();
    }
};

using APQueue = MapQueue<ApexPathPairPtr, ApexPathPair::more_than_full_cost>;
using PPQueue = MapQueue<PathPairPtr, PathPair::more_than_full_cost>;
using NodeQueue = MapQueue<NodePtr, Node::more_than_full_cost>;

using HLQueue = MapQueue<HighLevelNodePtr, HighLevelNode::more_than_full_cost>;