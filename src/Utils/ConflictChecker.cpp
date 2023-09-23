#include "Utils/ConflictChecker.h"

std::tuple<int, int, std::vector<size_t>, size_t> ConflictChecker::is_conflict(JointPathPair &joint_path, std::vector<PathSet>& indiv_paths_list)
{
    size_t agent_num = joint_path.second.size();
    //  vertex collision check
    for(size_t i = 0; i < agent_num; i ++){
        for(size_t j = i+1; j < agent_num; j ++){
            size_t id_i = joint_path.second.at(i), id_j = joint_path.second.at(j);
            std::vector<size_t> node_ids_i = indiv_paths_list.at(i)[id_i];
            std::vector<size_t> node_ids_j = indiv_paths_list.at(j)[id_j];
            for(size_t k = 0; k < node_ids_i.size() && k < node_ids_j.size(); k ++){
                if(node_ids_i.at(k) == node_ids_j.at(k)){
                    return std::make_tuple(i, j, std::vector<size_t>({node_ids_i.at(k)}), k);
                }
            }

        //  if one agent has reached, another conflict with its target
            if (node_ids_i.size() < node_ids_j.size()) {
                for(size_t k = node_ids_i.size(); k < node_ids_j.size(); k++){
                    if(node_ids_i.back() == node_ids_j.at(k)){
                        return std::make_tuple(-i-1, j, std::vector<size_t>({node_ids_i.back()}), k);
                    }
                }
            }else{
                for(size_t k = node_ids_j.size(); k < node_ids_i.size(); k++){
                    if(node_ids_i.at(k) == node_ids_j.back()){
                        return std::make_tuple(i, -j-1, std::vector<size_t>({node_ids_j.back()}), k);
                    }
                }
            }

        //  edge check
            for(size_t k = 0; k < node_ids_i.size()-1 && k < node_ids_j.size()-1; k ++){
                if(node_ids_i.at(k) == node_ids_j.at(k+1) && node_ids_i.at(k+1) == node_ids_j.at(k)){
                    return std::make_tuple(i, j, std::vector<size_t>({node_ids_i.at(k), node_ids_j.at(k)}), k);
                }
            }
        }
    }
                
    return std::make_tuple(-1, -1, std::vector<size_t>(), -1);
}