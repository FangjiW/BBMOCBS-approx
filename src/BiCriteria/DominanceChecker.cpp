#include "DominanceChecker.h"


bool SolutionCheck::is_dominated(ApexPathPairPtr node){
    if (last_solution == nullptr){
        return false;
    }
    // if(last_solution->path_node->conflict_num > node->path_node->conflict_num){
    //     return false;
    // }
    if (is_bounded(node->apex, last_solution->path_node, eps)){ // 不一定只有last_solution能bound
        assert(last_solution->update_apex_by_merge_if_bounded(node->apex, eps));
        // std::cout << "solution dom" << std::endl;
        return true;
    }
    for(auto iter = solutions.begin(); iter != solutions.end();){
        if((*iter)->path_node->conflict_num > node->path_node->conflict_num){
            continue;
        }else if((*iter)->update_apex_by_merge_if_bounded(node->apex, eps)){
            return true;
        }
    }
    return false;
}

// void SolutionCheck::add_node(ApexPathPairPtr ap)
// {
//     for(auto iter = solutions.begin(); iter != solutions.end(); ){
//         if(is_dominated_dr((*iter)->path_node, ap->path_node) && (*iter)->path_node->conflict_num > ap->path_node->conflict_num){
//             iter = solutions.erase(iter);
//         }
//     }
// }


bool LocalCheck::is_dominated(ApexPathPairPtr node){
    if(!min_g2[node->id].count(node->path_node->t)){
        return false;
    }else{
        return node->apex->g[1] >= min_g2[node->id][node->path_node->t];
    }
    // return (node->apex->g[1] >= min_g2[node->id]);
}

void LocalCheck::add_node(ApexPathPairPtr ap){
    auto id = ap->id;
    assert(min_g2[ap->id].count(ap->path_node->t) == 0 || min_g2[ap->id][ap->path_node->t] > ap->apex->g[1]);
    min_g2[ap->id][ap->path_node->t] = ap->apex->g[1];
}

bool LocalCheckLinear::is_dominated(ApexPathPairPtr node){
    if(!min_g2[node->id].count(node->path_node->t)){
        return false;
    }
    for (auto ap:min_g2[node->id][node->path_node->t]){
        if(node->path_node->conflict_num < ap->path_node->conflict_num){
            continue;
        }
        if(if_turn && !same_orientation(ap->path_node, node->path_node)){
            continue;
        }
        if (is_dominated_dr(node->apex, ap->apex)){
            assert(node->apex->f[0] >= ap->apex->f[0]);
            return true;
        }
    }
    return false;
}

void LocalCheckLinear::add_node(ApexPathPairPtr ap){
    auto id = ap->id;
    if(!min_g2[id].count(ap->path_node->t)){
        min_g2[ap->id][ap->path_node->t].push_front(ap);
        return;
    }
    for (auto it = min_g2[id][ap->path_node->t].begin(); it != min_g2[id][ap->path_node->t].end(); ){
        // TODO remove it for performance
        if(if_turn && !same_orientation((*it)->path_node, ap->path_node)){
            it ++;
            continue;
        }
        assert(! is_dominated_dr(ap->apex, (*it)->apex) || ap->path_node->conflict_num < (*it)->path_node->conflict_num);
        if (ap->path_node->conflict_num <= (*it)->path_node->conflict_num && is_dominated_dr((*it)->apex, ap->apex)){    
            it = min_g2[id][ap->path_node->t].erase(it);
        } else {
            it ++;
        }
    }

    min_g2[ap->id][ap->path_node->t].push_front(ap);
}

bool SolutionCheckLinear::is_dominated(ApexPathPairPtr node){
    for (auto& ap: solutions){
        if(node->path_node->conflict_num < ap->path_node->conflict_num){
            continue;
        }
        if (ap->update_apex_by_merge_if_bounded(node->apex, eps)){
            return true;
        }
    }
    return false;
}

void SolutionCheckLinear::add_node(ApexPathPairPtr ap){
    for (auto it = solutions.begin(); it != solutions.end(); ){
        if (ap->path_node->conflict_num <= (*it)->path_node->conflict_num && is_dominated_dr((*it)->path_node, ap->path_node)){
            it = solutions.erase(it);
        } else {
            it ++;
        }
    }
    solutions.push_front(ap);
}
