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
    return false;
}


bool LocalCheck::is_dominated(ApexPathPairPtr node){
    if(!min_g2[node->id].count(node->t)){
        return false;
    }else{
        return node->apex->g[1] >= min_g2[node->id][node->t];
    }
    // return (node->apex->g[1] >= min_g2[node->id]);
}

void LocalCheck::add_node(ApexPathPairPtr ap){
    auto id = ap->id;
    assert(min_g2[ap->id].count(ap->t) == 0 || min_g2[ap->id][ap->t] > ap->apex->g[1]);
    min_g2[ap->id][ap->t] = ap->apex->g[1];
}

bool LocalCheckLinear::is_dominated(ApexPathPairPtr node){
    if(!min_g2[node->id].count(node->t)){
        return false;
    }
    for (auto ap:min_g2[node->id][node->t]){
        if(node->path_node->conflict_num < ap->path_node->conflict_num){
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
    if(!min_g2[id].count(ap->t)){
        min_g2[ap->id][ap->t].push_front(ap);
        return;
    }
    for (auto it = min_g2[id][ap->t].begin(); it != min_g2[id][ap->t].end(); ){
        // TODO remove it for performance
        assert(! is_dominated_dr(ap->apex, (*it)->apex) || ap->path_node->conflict_num < (*it)->path_node->conflict_num);
        if (is_dominated_dr((*it)->apex, ap->apex) && ap->path_node->conflict_num <= (*it)->path_node->conflict_num){    
            it = min_g2[id][ap->t].erase(it);
        } else {
            it ++;
        }
        // assert(! is_dominated_dr(ap->apex, (*it)->apex));
        // if (is_dominated_dr((*it)->apex, ap->apex)){    
        //     it = min_g2[id][ap->t].erase(it);
        // } else {
        //     it ++;
        // }
    }

    min_g2[ap->id][ap->t].push_front(ap);
}

bool SolutionCheckLinear::is_dominated(ApexPathPairPtr node){
    for (auto ap: solutions){
        if(node->path_node->conflict_num < ap->path_node->conflict_num && !is_bounded(node->apex, ap->apex)){
            continue;
        }
        // if (is_bounded(node->apex, ap->apex)){
        if (ap->update_apex_by_merge_if_bounded(node->apex, eps)){
            // assert(ap->update_apex_by_merge_if_bounded(node->apex, eps));
            return true;
        }
    }
    // std::cout << "node id" << node->id << std::endl;
    // std::cout << "conflict num: " << node->path_node->conflict_num << std::endl;
    // std::cout << node->apex->f.at(0) << ", " << node->apex->f.at(1) << ", " << node->apex->f.at(2) << std::endl;
    // std::cout << "solution num: " << solutions.size() << std::endl;
    // for(auto ap : solutions){
    //     std::cout << ap->apex->f.at(0) << ", " << ap->apex->f.at(1) << ", " << ap->apex->f.at(2) << std::endl;  
    //     std::cout << ap->path_node->conflict_num << std::endl;
    // }
    return false;
}

void SolutionCheckLinear::add_node(ApexPathPairPtr ap){
    for (auto it = solutions.begin(); it != solutions.end(); ){
        if (is_dominated_dr((*it)->path_node, ap->path_node) && ap->path_node->conflict_num <= (*it)->path_node->conflict_num){
            it = solutions.erase(it);
        } else {
            it ++;
        }
    }
    // for (auto it = solutions.begin(); it != solutions.end(); ){
    //     if (is_dominated_dr((*it)->path_node, ap->path_node)){
    //         it = solutions.erase(it);
    //     } else {
    //         it ++;
    //     }
    // }
    solutions.push_front(ap);
}
