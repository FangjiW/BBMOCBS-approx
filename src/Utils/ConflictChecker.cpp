#include "Utils/ConflictChecker.h"

std::tuple<int, int, std::vector<size_t>, size_t> ConflictChecker::is_conflict(HighLevelNodePtr node)
{
    size_t agent_num = node->rep_id_list.size();
    //  vertex collision check
    for(size_t i = 0; i < agent_num; i ++){
        for(size_t j = i+1; j < agent_num; j ++){
            size_t id_i = node->rep_id_list.at(i), id_j = node->rep_id_list.at(j);
            std::vector<size_t> node_ids_i = node->indiv_paths_list.at(i)[id_i];
            std::vector<size_t> node_ids_j = node->indiv_paths_list.at(j)[id_j];
            for(size_t k = 0; k < node_ids_i.size() && k < node_ids_j.size(); k ++){
                if(node_ids_i.at(k) == node_ids_j.at(k)){
                    // return std::vector<int>({(int)i, (int)j, (int)node_ids_i.at(k), (int)k});
                    return std::make_tuple(i, j, std::vector<size_t>({node_ids_i.at(k)}), k);
                }
            }

        //  if one agent has reached, another conflict with its target
            if (node_ids_i.size() < node_ids_j.size()) {
                for(size_t k = node_ids_i.size(); k < node_ids_j.size(); k++){
                    if(node_ids_i.back() == node_ids_j.at(k)){
                        // return std::vector<int>({(int)-i-1, (int)j, (int)node_ids_j.at(k), (int)k}); 
                        return std::make_tuple(-i-1, j, std::vector<size_t>({node_ids_j.at(k)}), k);
                    }
                }
            }
            if (node_ids_i.size() > node_ids_j.size()) {
                for(size_t k = node_ids_j.size(); k < node_ids_i.size(); k++){
                    if(node_ids_i.at(k) == node_ids_j.back()){
                        // return std::vector<int>({(int)i, (int)-j-1, (int)node_ids_i.at(k), (int)k}); 
                        return std::make_tuple(i, -j-1, std::vector<size_t>({node_ids_i.at(k)}), k);
                    }
                }
            }
        }
    }

    //  edge collision check
    for(size_t i = 0; i < agent_num; i ++){
        for(size_t j = i+1; j < agent_num; j ++){
            size_t id_i = node->rep_id_list.at(i), id_j = node->rep_id_list.at(j);
            std::vector<size_t> node_ids_i = node->indiv_paths_list.at(i)[id_i];
            std::vector<size_t> node_ids_j = node->indiv_paths_list.at(j)[id_j];
            for(size_t k = 0; k < node_ids_i.size()-1 && k < node_ids_j.size()-1; k ++){
                if(node_ids_i.at(k) == node_ids_j.at(k+1) && node_ids_j.at(k) == node_ids_i.at(k+1)){
                    //  tuple(agent_id1, agent_id2, {node1(agent1 at time k), node2(agent2 at time k)}, time k)
                    return std::make_tuple(i, j, std::vector<size_t>({node_ids_i.at(k), node_ids_j.at(k)}), k);
                }
            }
        }
    }

    //  edge confliction
                
    return std::make_tuple(-1, -1, std::vector<size_t>(), 1);
}

bool ConflictChecker::is_conflict(JointPathPair &joint_path, std::vector<PathSet>& indiv_paths_list, int agent_num)
{
    for(size_t i = 0; i < agent_num; i ++){
        for(size_t j = i+1; j < agent_num; j ++){
            size_t id_i = joint_path.second.at(i), id_j = joint_path.second.at(j);
            std::vector<size_t> node_ids_i = indiv_paths_list.at(i)[id_i];
            std::vector<size_t> node_ids_j = indiv_paths_list.at(j)[id_j];
            for(size_t k = 0; k < node_ids_i.size() && k < node_ids_j.size(); k ++){
                if(node_ids_i.at(k) == node_ids_j.at(k)){
                    return true;
                }
            }

        //  if one agent has reached, another conflict with its target
            if (node_ids_i.size() < node_ids_j.size()) {
                for(size_t k = node_ids_i.size(); k < node_ids_j.size(); k++){
                    if(node_ids_i.back() == node_ids_j.at(k)){
                        return true;
                    }
                }
            }
            if (node_ids_i.size() > node_ids_j.size()) {
                for(size_t k = node_ids_j.size(); k < node_ids_i.size(); k++){
                    if(node_ids_i.at(k) == node_ids_j.back()){
                        return true;
                    }
                }
            }
        //  edge check
            for(size_t k = 0; k < node_ids_i.size()-1 && k < node_ids_j.size()-1; k ++){
                if(node_ids_i.at(k) == node_ids_j.at(k+1) && node_ids_j.at(k) == node_ids_i.at(k+1)){
                    return true;
                }
            }
        }
    }

    return false;
}