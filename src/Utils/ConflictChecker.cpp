#include "Utils/ConflictChecker.h"

std::vector<int> ConflictChecker::is_conflict(HighLevelNodePtr node)
{
    size_t agent_num = node->rep_id_list.size();
    for(size_t i = 0; i < agent_num; i ++){
        for(size_t j = i+1; j < agent_num; j ++){
            size_t id_i = node->rep_id_list.at(i), id_j = node->rep_id_list.at(j);
            std::vector<size_t> node_ids_i = node->indiv_paths_list.at(i)[id_i];
            std::vector<size_t> node_ids_j = node->indiv_paths_list.at(j)[id_j];
            for(size_t k = 0; k < node_ids_i.size() && k < node_ids_j.size(); k ++){
                if(node_ids_i.at(k) == node_ids_j.at(k)){
                    return std::vector<int>({(int)i, (int)j, (int)node_ids_i.at(k), (int)k});
                }
            }
            if (node_ids_i.size() < node_ids_j.size()) {
                for(size_t k = node_ids_i.size(); k < node_ids_j.size(); k++){
                    if(node_ids_i.back() == node_ids_j.at(k)){
                        return std::vector<int>({(int)-i-1, (int)j, (int)node_ids_j.at(k), (int)k}); 
                    }
                }
            }
            if (node_ids_i.size() > node_ids_j.size()) {
                for(size_t k = node_ids_j.size(); k < node_ids_i.size(); k++){
                    if(node_ids_i.at(k) == node_ids_j.back()){
                        return std::vector<int>({(int)i, (int)-j-1, (int)node_ids_i.at(k), (int)k}); 
                    }
                }
            }
        }
    }
    return std::vector<int>();
}