#include <algorithm>
#include <typeinfo>
#include "Utils/MapQueue.h"

template<class Node, class more_than_full_cost>
MapQueue<Node, more_than_full_cost>::MapQueue(size_t graph_size)
    : open_map(graph_size) {

    std::make_heap(this->heap.begin(), this->heap.end(), this->more_than);
}

template<class Node, class more_than_full_cost>
bool MapQueue<Node, more_than_full_cost>::empty() {
    return this->heap.empty();
}

// template<class Node, class more_than_full_cost>
// Node MapQueue<Node, more_than_full_cost>::top() {
//     return this->heap.back();
// }

template<class Node, class more_than_full_cost>
Node MapQueue<Node, more_than_full_cost>::pop() {
    // Pop from min heap
    std::pop_heap(this->heap.begin(), this->heap.end(), this->more_than);
    Node pp = this->heap.back();
    this->heap.pop_back();

    // Remove from open map
    std::list<Node> &relevant_pps = this->open_map[pp->id][pp->t];
    for (auto iter = relevant_pps.begin(); iter != relevant_pps.end(); ++iter) {
        if (pp == *iter) {
            relevant_pps.erase(iter);
            break;
        }
    }

    return pp;
}

template<class Node, class more_than_full_cost>
void MapQueue<Node, more_than_full_cost>::insert(Node &pp) {
    // Insert to min heap
    this->heap.push_back(pp);
    std::push_heap(this->heap.begin(), this->heap.end(), this->more_than);

    // Insert to open map
    if(!open_map.at(pp->id).count(pp->t)){
        open_map.at(pp->id).insert(std::make_pair(pp->t, std::list<Node>()));
    }
    this->open_map[pp->id][pp->t].push_back(pp);
}

template<class Node, class more_than_full_cost>
std::list<Node> &MapQueue<Node, more_than_full_cost>::get_open(size_t id, int t) {
    if(!open_map.at(id).count(t)){
        open_map.at(id).insert(std::make_pair(t, std::list<Node>()));
    }
    return this->open_map[id][t];
}


template class MapQueue<ApexPathPairPtr, ApexPathPair::more_than_full_cost>;
// template class MapQueue<PathPairPtr, PathPair::more_than_full_cost>;
template class MapQueue<NodePtr, Node::more_than_full_cost>;




HighLevelNodePtr HLQueue::pop()
{
    std::pop_heap(this->heap.begin(), this->heap.end(), HLmore_than);
    HighLevelNodePtr node = this->heap.back();
    this->heap.pop_back();
    return node;
}

void HLQueue::insert(HighLevelNodePtr& node)
{
    this->heap.push_back(node);
    std::push_heap(this->heap.begin(), this->heap.end(), HLmore_than);
}