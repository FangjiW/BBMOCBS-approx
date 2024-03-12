#pragma once

#include <set>
#include <vector>
#include <list>
#include "Definitions.h"

template<class Node, class more_than_full_cost>
class MapQueue
{
private:
    std::vector<Node>                heap;
    more_than_full_cost           more_than;

    std::vector<std::unordered_map<int, std::list<Node>>>   open_map;

public:
    MapQueue(){}
    MapQueue(size_t graph_size);
    bool empty();
    // Node top();
    Node pop();
    void insert(Node &pp);
    std::list<Node> &get_open(size_t id, int t);
    size_t size(){
        return heap.size();
    }
};

using APQueue = MapQueue<ApexPathPairPtr, ApexPathPair::more_than_full_cost>;
// using PPQueue = MapQueue<PathPairPtr, PathPair::more_than_full_cost>;
using NodeQueue = MapQueue<NodePtr, Node::more_than_full_cost>;



class HLQueue
{
private:
    std::vector<HighLevelNodePtr>      heap;

public:
    HLQueue(){};
    bool empty(){
        return heap.empty();
    }
    HighLevelNodePtr pop();
    void insert(HighLevelNodePtr& node);
    size_t size(){
        return heap.size();
    }
};

inline bool HLmore_than(HighLevelNodePtr a, HighLevelNodePtr b){
    for(size_t i = 0; i < a->cur_apex.size(); i ++){
        if(a->cur_apex.at(i) != b->cur_apex.at(i)){
            return a->cur_apex.at(i) > b->cur_apex.at(i);
        }
    }
    return false;
}