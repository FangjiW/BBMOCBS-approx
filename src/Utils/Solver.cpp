#include "Utils/Solver.h"
#include <cstdlib>
#include <list>
#include <algorithm>
#include <set>
#include <random>

extern std::unordered_map<size_t, std::vector<int>> id2coord;
extern std::string map_name;
extern std::ofstream output;
int time_limit;
clock_t start_time;
bool if_stop = false;
template <typename T>
bool less_than(const T &a, const T &b, bool lexico=true)
{
    if(lexico == true){  // lexicographic
        auto vec1 = a.first, vec2 = b.first;
        for(int i = 0; i < vec1.size(); i ++){
            if(vec1.at(i) != vec2.at(i)){
                return vec1.at(i) < vec2.at(i);
            }
        }
        return true;
    }else{  // only compare the first component
        return a.first.at(0) <= b.first.at(0);
    }
}

struct Less_Than_
{
    bool operator()(const CostVector &a, const CostVector &b){
        for(int i = 0; i < a.size(); i++){
            if(a.at(i) != b.at(i)){
                return a.at(i) < b.at(i);
            }
        }
        return true;
    }
};

void Solver::add_constraint(std::vector<VertexConstraint>& vertex_constraints, size_t agent_id, size_t node_id, size_t time)
{
    if(!vertex_constraints.at(agent_id).count(time)){
        vertex_constraints.at(agent_id).insert(std::make_pair(time, std::vector<size_t>()));
    }
    vertex_constraints.at(agent_id)[time].push_back(node_id);
}

void Solver::add_constraint(std::vector<EdgeConstraint>& edge_constraints, size_t agent_id, size_t source, size_t target, size_t time)
{
    if(!edge_constraints.at(agent_id).count(time)){
        edge_constraints.at(agent_id).insert(std::make_pair(time, std::unordered_map<size_t, std::vector<size_t>>()));
    }
    if(!edge_constraints.at(agent_id)[time].count(source)){
        edge_constraints.at(agent_id)[time].insert(std::make_pair(source, std::vector<size_t>()));
    }

    edge_constraints.at(agent_id)[time][source].push_back(target);
}


// A*pex version
void Solver::NonDomJointPath(HighLevelNodePtr node, MergeStrategy ms, double eps, int agent_id)
{
    for(auto e : node->indiv_paths_list){
        std::cout << e.size() << " ";
    }
    if(ms == MergeStrategy::MORE_SLACK){
        agent_id = -1;
    }
    // getchar();
    auto _t1 = std::chrono::high_resolution_clock::now();
    int dim = node->indiv_apex_costs.at(0)[0].size();
    std::pair<CostVector, int>  temp_apex_id;
    CostVector     temp_real_cost;
    std::vector<size_t>    temp_id;  // of one joint path

    std::list<std::pair<CostVector, int>>   apex_idx_combos;
    std::vector<CostVector>     real_costs_vector;
    std::vector<int>            conflict_nums_vector;
    std::vector<std::vector<size_t>>    ids_vector;

    node->joint_path_list.clear();    

    apex_idx_combos.push_back(std::make_pair(CostVector(dim, 0), 0));
    real_costs_vector.push_back(CostVector(dim, 0));
    ids_vector.push_back(std::vector<size_t>());

    for(int i = 0; i < node->indiv_paths_list.size(); i++){
        if((std::clock() - start_time)/CLOCKS_PER_SEC > time_limit){
            return;
        }
        if(i == agent_id){
            continue;
        }
        std::list<std::pair<CostVector, int>> _apex_idx_combos(apex_idx_combos);
        std::vector<CostVector>     _real_costs_vector(real_costs_vector);
        std::vector<std::vector<size_t>>    _ids_vector(ids_vector);
        apex_idx_combos.clear();
        real_costs_vector.clear(); real_costs_vector.shrink_to_fit();
        ids_vector.clear(); ids_vector.shrink_to_fit();
        int new_id = 0;
        for(const auto& apex_id_combo : _apex_idx_combos){
            for(const auto& indiv_path : node->indiv_paths_list.at(i)){
                temp_apex_id = apex_id_combo;
                temp_real_cost = _real_costs_vector.at(apex_id_combo.second);
                temp_id = _ids_vector.at(apex_id_combo.second);
                add_cost(temp_apex_id.first, node->indiv_apex_costs.at(i)[indiv_path.first]);
                add_cost(temp_real_cost, node->indiv_real_costs.at(i)[indiv_path.first]);
                temp_id.push_back(indiv_path.first);
                apex_idx_combos.push_back(std::make_pair(temp_apex_id.first, new_id++));
                real_costs_vector.push_back(temp_real_cost);
                ids_vector.push_back(temp_id);
            }
        }
        apex_idx_combos.sort([](const std::pair<CostVector, int>& a, std::pair<CostVector, int>& b){
            return less_than<std::pair<CostVector, int>>(a, b);});        
        NonDomVec(apex_idx_combos, real_costs_vector, ids_vector, conflict_nums_vector, ms, eps);
    }
    if(agent_id != -1){
        std::list<std::pair<CostVector, int>> _apex_idx_combos(apex_idx_combos);
        std::vector<CostVector>     _real_costs_vector(real_costs_vector);
        std::vector<std::vector<size_t>>    _ids_vector(ids_vector);
        apex_idx_combos.clear(); 
        real_costs_vector.clear(); real_costs_vector.shrink_to_fit();
        ids_vector.clear(); ids_vector.shrink_to_fit();
        int new_id = 0;
        for(const auto& apex_id_combo : _apex_idx_combos){
            for(const auto& indiv_path : node->indiv_paths_list.at(agent_id)){
                temp_apex_id = apex_id_combo;
                temp_real_cost = _real_costs_vector.at(apex_id_combo.second);
                temp_id = _ids_vector.at(apex_id_combo.second);

                add_cost(temp_apex_id.first, node->indiv_apex_costs.at(agent_id)[indiv_path.first]);
                add_cost(temp_real_cost, node->indiv_real_costs.at(agent_id)[indiv_path.first]);

                temp_id.insert(temp_id.begin()+agent_id, indiv_path.first);
                apex_idx_combos.push_back(std::make_pair(temp_apex_id.first, new_id++));
                real_costs_vector.push_back(temp_real_cost);
                ids_vector.push_back(temp_id);
                conflict_nums_vector.push_back(node->conflict_num[indiv_path.first]);
            }
        }
        apex_idx_combos.sort([](const std::pair<CostVector, int>& a, std::pair<CostVector, int>& b){
            return less_than<std::pair<CostVector, int>>(a, b);});        
        NonDomVec(apex_idx_combos, real_costs_vector, ids_vector, conflict_nums_vector, ms, eps);
    }
        // int min_conflict_num = std::get<3>(joint_path_vector.front());
        // for(auto iter = joint_path_vector.begin(); iter != joint_path_vector.end(); ){
        //     if(std::get<3>(*iter) < min_conflict_num){
        //         auto temp = *iter;
        //         iter = joint_path_vector.erase(iter);
        //         joint_path_vector.push_front(temp);
        //         min_conflict_num = std::get<3>(temp);
        //     }else if(std::get<3>(*iter) == min_conflict_num){
        //         bool if_change = false;
        //         for(int i = 0; i < std::get<0>(*iter).size(); i++){
        //             if(std::get<0>(*iter).at(i) < std::get<0>(joint_path_vector.front()).at(i)){
        //                 if_change = true;
        //                 break;
        //             }else if(std::get<0>(*iter).at(i) > std::get<0>(joint_path_vector.front()).at(i)){
        //                 break;
        //             }
        //         }
        //         if(if_change){
        //             auto temp = *iter;
        //             iter = joint_path_vector.erase(iter);
        //             joint_path_vector.push_front(temp);
        //         }else{
        //             iter ++;
        //         }
        //     }else{
        //         iter ++;
        //     }
        // }
    apex_idx_combos.sort([](const std::pair<CostVector, int>& a, std::pair<CostVector, int>& b){
        return less_than<std::pair<CostVector, int>>(a, b);});
    for(auto element : apex_idx_combos){
        node->joint_path_list.push_back(std::make_pair(element.first, ids_vector.at(element.second)));
    }
    auto t2 = std::chrono::high_resolution_clock::now(); // for timing.
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(t2 - _t1);
    std::cout << " Non Dom time = " << ((double)duration.count()) / 1000000.0 << std::endl;
    std::cout << node->joint_path_list.size() << std::endl;
    // getchar();
}

// Number of Solution
void Solver::NonDomJointPath(HighLevelNodePtr node, int solution_num, double max_eps)
{
    for(auto e : node->indiv_paths_list){
        std::cout << e.size() << " ";
    }
    auto _t1 = std::chrono::high_resolution_clock::now();
    int dim = node->indiv_apex_costs.at(0)[0].size();
    std::pair<CostVector, int>  temp_apex_id;
    CostVector     temp_real_cost;
    std::vector<size_t>    temp_id;  // of one joint path

    std::list<std::pair<CostVector, int>>   apex_idx_combos;
    std::vector<CostVector>     real_costs_vector;
    std::vector<int>            conflict_nums_vector;
    std::vector<std::vector<size_t>>    ids_vector;

    node->joint_path_list.clear();    

    apex_idx_combos.push_back(std::make_pair(CostVector(dim, 0), 0));
    real_costs_vector.push_back(CostVector(dim, 0));
    ids_vector.push_back(std::vector<size_t>());

    for(int i = 0; i < node->indiv_paths_list.size(); i++){
        if((std::clock() - start_time)/CLOCKS_PER_SEC > time_limit){
            return;
        }
        std::list<std::pair<CostVector, int>> _apex_idx_combos(apex_idx_combos);
        std::vector<CostVector>     _real_costs_vector(real_costs_vector);
        std::vector<std::vector<size_t>>    _ids_vector(ids_vector);
        apex_idx_combos.clear();
        real_costs_vector.clear(); real_costs_vector.shrink_to_fit();
        ids_vector.clear(); ids_vector.shrink_to_fit();
        int new_id = 0;
        for(const auto& apex_id_combo : _apex_idx_combos){
            for(const auto& indiv_path : node->indiv_paths_list.at(i)){
                temp_apex_id = apex_id_combo;
                temp_real_cost = _real_costs_vector.at(apex_id_combo.second);
                temp_id = _ids_vector.at(apex_id_combo.second);
                add_cost(temp_apex_id.first, node->indiv_apex_costs.at(i)[indiv_path.first]);
                add_cost(temp_real_cost, node->indiv_real_costs.at(i)[indiv_path.first]);
                temp_id.push_back(indiv_path.first);
                apex_idx_combos.push_back(std::make_pair(temp_apex_id.first, new_id++));
                real_costs_vector.push_back(temp_real_cost);
                ids_vector.push_back(temp_id);
            }
        }
        apex_idx_combos.sort([](const std::pair<CostVector, int>& a, std::pair<CostVector, int>& b){
            return less_than<std::pair<CostVector, int>>(a, b);});        
        NonDomVec(apex_idx_combos, real_costs_vector, ids_vector, conflict_nums_vector, MergeStrategy::NONE, 0);
        MergeBySmallestEps(apex_idx_combos, real_costs_vector, ids_vector, solution_num, max_eps);
    }

    apex_idx_combos.sort([](const std::pair<CostVector, int>& a, std::pair<CostVector, int>& b){
        return less_than<std::pair<CostVector, int>>(a, b);
    });     

    for(auto& element : apex_idx_combos){
        node->joint_path_list.push_back(std::make_pair(element.first, ids_vector.at(element.second)));
    }
    auto t2 = std::chrono::high_resolution_clock::now(); // for timing.
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(t2 - _t1);
    std::cout << " Non Dom time = " << ((double)duration.count()) / 1000000.0 << std::endl;
    std::cout << node->joint_path_list.size() << std::endl;
    // getchar();
}

//  BOA* and NAMOA*dr version
void Solver::NonDomJointPath(HighLevelNodePtr node)
{
    for(auto e : node->indiv_paths_list){
        std::cout << e.size() << " ";
    }
    auto _t1 = std::chrono::high_resolution_clock::now();
    JointPathPair temp;
    node->joint_path_list.clear();
    std::list<JointPathPair>& joint_path_list = node->joint_path_list;

    for(size_t id = 0; id < node->indiv_paths_list.at(0).size(); id++){
        joint_path_list.push_back(std::make_pair(node->indiv_real_costs.at(0)[id], std::vector<size_t>({id})));
    }
    
    for(int i = 1; i < node->indiv_paths_list.size(); i++){
        if((std::clock() - start_time)/CLOCKS_PER_SEC > time_limit){
            return;
        }
        std::list<JointPathPair> _joint_path_list(joint_path_list);
        joint_path_list.clear();
        for(auto joint_path = _joint_path_list.begin(); joint_path != _joint_path_list.end(); joint_path++){
            for(auto iter = node->indiv_paths_list.at(i).begin(); iter != node->indiv_paths_list.at(i).end(); iter++){
                temp = *joint_path;
                temp.second.push_back(iter->first);
                add_cost(temp.first, node->indiv_real_costs.at(i)[iter->first]);
                // joint_path.second.push_back(indiv_path.first);
                joint_path_list.push_back(std::make_pair(std::move(temp.first), std::move(temp.second)));
                // std::push_heap(_joint_path_vector.begin(), _joint_path_vector.end(), less_than_);
            }
        }
        joint_path_list.sort([](const JointPathPair& a, JointPathPair& b){
            return less_than<JointPathPair>(a, b);});
        NonDomVec(joint_path_list);
    }
    auto t2 = std::chrono::high_resolution_clock::now(); // for timing.
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(t2 - _t1);
    std::cout << " Non Dom time = " << ((double)duration.count()) / 1000000.0 << std::endl;
    std::cout << node->joint_path_list.size();
    std::cout <<std::endl;
}


// A*pex version
void Solver::NonDomVec(std::list<std::pair<CostVector, int>>& apex_idx_combos, std::vector<CostVector>& real_costs_vector, std::vector<std::vector<size_t>>& ids_vector, 
    std::vector<int>& conflict_nums_vector, MergeStrategy ms, double eps)
{
    if(ms != LEAST_CONFLICT){
        if (apex_idx_combos.front().first.size() == 2) {
            size_t g_min = UINT64_MAX;
            for(auto iter = apex_idx_combos.begin(); iter != apex_idx_combos.end(); ) {
                if (iter->first.at(1) < g_min) {
                    g_min = iter->first.at(1);
                    if(ms == NONE || ms == MORE_SLACK){
                        iter ++;
                        continue;
                    }
                    bool is_merge = false;
                    for (auto iter1 = apex_idx_combos.begin(); iter1 != iter; iter1++) {
                        if (HighLevelMerge(*iter1, *iter, real_costs_vector.at(iter1->second), real_costs_vector.at(iter->second), ids_vector.at(iter1->second), 
                        ids_vector.at(iter->second), -1, -1, ms, eps)) {
                            iter = apex_idx_combos.erase(std::next(iter1), std::next(iter));
                            is_merge = true;
                            break;
                        }
                    }
                    if (!is_merge) {
                        iter ++;
                    }
                }else{
                    iter = apex_idx_combos.erase(iter);
                }
            }
        }else if(apex_idx_combos.front().first.size() == 3){
        // dominance check
            std::set<CostVector, Less_Than_>  domination_set;
            for(auto iter = apex_idx_combos.begin(); iter != apex_idx_combos.end(); ){
            //  dominance check: if apex be strictly dominated(not eps-), then prune; 
            //  else erase the behind strictly dominated ones in RB-tree
                std::vector<size_t>  trun_vec({iter->first.at(1), iter->first.at(2)});
                if(domination_set.empty()){
                    domination_set.insert(trun_vec);
                    iter ++;
                    continue;
                }
                auto it = domination_set.upper_bound(trun_vec);
                if(it != domination_set.begin() && (*std::prev(it)).at(1) <= trun_vec.at(1)){
                    iter = apex_idx_combos.erase(iter);
                }else{  
                //  merge
                    bool is_merge = false;
                    if(ms != NONE && ms != MORE_SLACK){
                        for (auto iter1 = apex_idx_combos.begin(); iter1 != iter; iter1++) {
                            // remark: even if merge happens, the RB-tree doesn't need changing
                            if(HighLevelMerge(*iter1, *iter, real_costs_vector.at(iter1->second), real_costs_vector.at(iter->second), ids_vector.at(iter1->second), 
                            ids_vector.at(iter->second), -1, -1, ms, eps)){  
                                trun_vec = std::vector<size_t>({iter1->first.at(1), iter1->first.at(2)});
                                iter = apex_idx_combos.erase(iter);
                                is_merge = true;
                                break;
                            }
                        }
                    }
                    if (!is_merge) {
                        iter ++;
                    }
                //  erase the behind weakly dominated ones
                    it = domination_set.lower_bound(trun_vec);
                    for(auto iter_ = it; iter_ != domination_set.end(); ){
                        if(trun_vec.at(1) <= iter_->at(1)){
                            iter_ = domination_set.erase(iter_);
                        }else{
                            break;
                        }
                    }
                    domination_set.insert(trun_vec);
                }
            }
        }else{
            output << std::endl << "(A*pex) ERROR: non-LEAST_CONFLICT strategy only for g size = 2 or 3" << std::endl;
            exit(1);
        }
    }else{
        for(auto iter = apex_idx_combos.begin(); iter != apex_idx_combos.end(); ){
            bool is_merge = false;
            for (auto iter1 = apex_idx_combos.begin(); iter1 != iter; iter1++) {
                if(HighLevelMerge(*iter1, *iter, real_costs_vector.at(iter1->second), real_costs_vector.at(iter->second), ids_vector.at(iter1->second), 
                ids_vector.at(iter->second), conflict_nums_vector.at(iter1->second), conflict_nums_vector.at(iter->second), ms, eps)){  
                    iter = apex_idx_combos.erase(iter);
                    is_merge = true;
                    break;
                }
            }
            if (!is_merge) {
                iter ++;
            }
        }
    }
    // // merge by smallest epsilon
    // if(ms == MORE_SLACK){
    //     MergeBySmallestEps(joint_path_vector, 0, eps);
    // }
}


//  BOA* and NAMOA* version
void Solver::NonDomVec(std::list<JointPathPair>& joint_path_list)
{
    if (joint_path_list.front().first.size() == 2) {
        size_t g_min = UINT64_MAX;
        for(auto iter = joint_path_list.begin(); iter != joint_path_list.end(); ) {
            if (iter->first.at(1) < g_min) {
                g_min = iter->first.at(1);
                iter ++;
            }else{
                iter = joint_path_list.erase(iter);
            }
        }
    }else if(joint_path_list.front().first.size() == 3){
    // dominance check: if be strictly dominated, then prune; else erase the behind strictly dominated ones in RB-tree
		std::set<CostVector, Less_Than_>  domination_set;
		for(auto iter = joint_path_list.begin(); iter != joint_path_list.end(); ){
            bool is_dominated = false;
			std::vector<size_t>  trun_vec({iter->first.at(1), iter->first.at(2)});
            if(domination_set.empty()){
                domination_set.insert(trun_vec);
                iter ++;
                continue;
            }
			auto it = domination_set.upper_bound(trun_vec);
            if(it != domination_set.begin() && std::prev(it)->at(1) <= trun_vec.at(1)){
                iter = joint_path_list.erase(iter);
            }else{
                //  delete the behind dominated ones (this operation can be removed)
                for(auto iter_ = it; iter_ != domination_set.end(); ){
                    if(trun_vec.at(1) <= (*iter_).at(1)){
                        iter_ = domination_set.erase(iter_);
                    }else{
                        break;
                    }
                }
                domination_set.insert(trun_vec);
                iter ++;
            }
		}
    }
    else {
        output << std::endl << "BOA*/NAMOA* ERROR: only g size = 2 or 3" << std::endl;
        exit(1);
    }
}


bool Solver::HighLevelMerge(std::pair<CostVector, int>& existing_path, std::pair<CostVector, int>& new_path, CostVector& real_cost1, 
    CostVector& real_cost2, std::vector<size_t>& id1, std::vector<size_t>& id2, int conflict_num1, int conflict_num2, MergeStrategy ms, double eps)
{
    CostVector apex = vector_min(std::get<0>(existing_path), std::get<0>(new_path));
    if(ms == SMALLER_G2){
        if(apex.size() != 2){
            std::cerr << "SMALLER_G2 can only used for bi-objectives";
            exit(-1);
        }else{
            if(real_cost1.at(1) < real_cost2.at(1)){
                if(is_dominated(apex, real_cost1, eps)){
                    existing_path.first = apex;
                    return true;
                }else if(is_dominated(apex, real_cost2, eps)){
                    existing_path.first = apex;
                    existing_path.second = new_path.second;
                    return true;
                }
                return false;
            }else{
                if(is_dominated(apex, real_cost2, eps)){
                    existing_path.first = apex;
                    existing_path.second = new_path.second;
                    return true;
                }else if(is_dominated(apex, real_cost1, eps)){
                    existing_path.first = apex;
                    return true;
                }
                return false;
            }
        }
    }else if(ms == RANDOM){
        if(is_dominated(apex, real_cost1, eps)){
            existing_path.first = apex;
            return true;
        }
        if(is_dominated(apex, real_cost2, eps)){
            existing_path.first = apex;
            existing_path.second = new_path.second;
            return true;
        }
        return false;
    }else if(ms == MergeStrategy::LEAST_CONFLICT){
        if(conflict_num1 < conflict_num2){
            if(is_dominated(apex, real_cost1, eps)){
                existing_path.first = apex;
                return true;
            }
            return false;
        }else if(conflict_num2 < conflict_num1){
            if(is_dominated(apex, real_cost2, eps)){
                existing_path.first = apex;
                existing_path.second = new_path.second;
                return true;
            }
            return false;
        }else{
            if(is_dominated(apex, real_cost1, eps)){
                existing_path.first = apex;
                return true;
            }
            if(is_dominated(apex, real_cost2, eps)){
                existing_path.first = apex;
                existing_path.second = new_path.second;
                return true;
            }
            return false;
        }
    }else{
        output << std::endl << "merge strategy only SMALLER_G2 and RANDOM";
        exit(1);
    }
}


void Solver::MergeBySmallestEps(std::list<std::pair<CostVector, int>>& apex_idx_combos, std::vector<CostVector>& real_costs_vector, 
    std::vector<std::vector<size_t>>& ids_vector, int solution_num, double max_eps)
{
    int total_num = apex_idx_combos.size();

    using mutual_eps = std::tuple<double, size_t, size_t>; // eps,apex_idx,real_idx,if_valid
    std::list<mutual_eps>               eps_list;
    std::vector<CostVector>             apex_vector;
    std::vector<bool>                   valid_vector(total_num, true);
    std::vector<int>                    id_vector;

    for(auto ele: apex_idx_combos){
        apex_vector.push_back(ele.first);
        id_vector.push_back(ele.second);
    }

    for(int i = 0; i < total_num; i++){
        for(int j = 0; j < total_num; j++){
            if(j == i){
                continue;
            }
            double eps = calculate_eps(apex_vector.at(i), real_costs_vector.at(id_vector.at(j)));
            eps_list.push_back(std::make_tuple(eps, i, j));
        }
    }

    int  erase_num = 0;
    while(total_num - erase_num > solution_num){
        mutual_eps element;
        double min_eps = INT_MAX;
        for(auto iter = eps_list.begin(); iter != eps_list.end(); iter++){
            if(std::get<0>(*iter) < min_eps){
                element = *iter;
                min_eps = std::get<0>(*iter);
            }
        }
        if(min_eps > max_eps){
            break;
        }
        erase_num ++;
        size_t invalid_id = std::get<1>(element);
        size_t valid_id = std::get<2>(element);
        apex_vector.at(valid_id) = vector_min(apex_vector.at(valid_id), apex_vector.at(invalid_id));
        valid_vector.at(invalid_id) = false;
        for(auto iter = eps_list.begin(); iter != eps_list.end(); ){
            if(std::get<1>(*iter) == invalid_id || std::get<2>(*iter) == invalid_id){
                iter = eps_list.erase(iter);
                continue;
            }
            if(std::get<1>(*iter) == valid_id){
                std::get<0>(*iter) = calculate_eps(apex_vector.at(valid_id), real_costs_vector.at(id_vector.at(std::get<2>(*iter))));
            }
            iter ++;
        }
    }

    apex_idx_combos.clear();
    int j = 0;
    for(int i = 0; i < total_num; i++){
        if(!valid_vector.at(i)){
            continue;
        }
        apex_idx_combos.push_back(std::make_pair(apex_vector.at(i), id_vector.at(i)));
    }
}


void Solver::MergeBySmallestEps(std::vector<CostVector>& apex_vectors, std::vector<CostVector>& real_costs_vector, int solution_num, double max_eps)
{
    int total_num = apex_vectors.size();

    using mutual_eps = std::tuple<double, size_t, size_t>; // eps,apex_idx,real_idx,if_valid
    std::list<mutual_eps>               eps_list;
    std::vector<CostVector>             apex_vector = apex_vectors;
    std::vector<CostVector>             cost_vector = real_costs_vector;
    std::vector<bool>                   valid_vector(total_num, true);

    for(int i = 0; i < total_num; i++){
        for(int j = 0; j < total_num; j++){
            if(j == i){
                continue;
            }
            double eps = calculate_eps(apex_vector.at(i), cost_vector.at(j));
            eps_list.push_back(std::make_tuple(eps, i, j));
        }
    }

    int  erase_num = 0;
    while(total_num - erase_num > solution_num){
        mutual_eps element;
        double min_eps = INT_MAX;
        for(auto iter = eps_list.begin(); iter != eps_list.end(); iter++){
            if(std::get<0>(*iter) < min_eps){
                element = *iter;
                min_eps = std::get<0>(*iter);
            }
        }
        if(min_eps > max_eps){
            break;
        }
        erase_num ++;
        size_t invalid_id = std::get<1>(element);
        size_t valid_id = std::get<2>(element);
        apex_vector.at(valid_id) = vector_min(apex_vector.at(valid_id), apex_vector.at(invalid_id));
        valid_vector.at(invalid_id) = false;
        for(auto iter = eps_list.begin(); iter != eps_list.end(); ){
            if(std::get<1>(*iter) == invalid_id || std::get<2>(*iter) == invalid_id){
                iter = eps_list.erase(iter);
                continue;
            }
            if(std::get<1>(*iter) == valid_id){
                std::get<0>(*iter) = calculate_eps(apex_vector.at(valid_id), cost_vector.at(std::get<2>(*iter)));
            }
            iter ++;
        }
    }

    apex_vectors.clear();
    real_costs_vector.clear();
    
    for(int i = 0; i < total_num; i++){
        if(!valid_vector.at(i)){
            continue;
        }
        apex_vectors.push_back(apex_vector.at(i));
        real_costs_vector.push_back(cost_vector.at(i));
    }
}

// void Solver::MergeByDiv(std::vector<CostVector>& apex_costs, std::vector<CostVector>& real_costs, int solution_num, double max_eps)
// {
//     if(apex_costs.front().size() != 2){
//         output << "MergeByDiv only for dim = 2";
//         exit(1);
//     }
//     size_t max_x = 0, max_y = 0;
//     std::vector<std::pair<int, CostVector>>  id_real_combo;
//     std::list<std::pair<CostVector, CostVector>>  apex_real_combo;
//     for(int i = 0; i < real_costs.size(); i++){
//         max_x = 2*real_costs.at(i).at(0) > max_x ? 2*real_costs.at(i).at(0) : max_x;
//         max_y = 2*real_costs.at(i).at(1) > max_y ? 2*real_costs.at(i).at(1) : max_y;
//         id_real_combo.push_back(std::make_pair(i, real_costs.at(i)));
//     }

//     auto _apex_costs = apex_costs; auto _real_costs = real_costs;
//     apex_costs.clear(); apex_costs.shrink_to_fit();
//     real_costs.clear(); real_costs.shrink_to_fit();
//     std::sort(id_real_combo.begin(), id_real_combo.end(), [](std::pair<int, CostVector>&a, std::pair<int, CostVector>& b){
//         for(int i = 0; i < a.second.size(); i++){
//             if(a.second.at(i) != b.second.at(i)){
//                 return a.second.at(i) < b.second.at(i);
//             }
//         }
//         return true;
//     });
//     size_t min_g2 = INT_MAX;
//     for(int i = 0; i < id_real_combo.size(); i++){
//         if(id_real_combo.at(i).second.at(1) >= min_g2){
//             apex_real_combo.back().first = vector_min(apex_real_combo.back().first, _apex_costs.at(id_real_combo.at(i).first));
//         }else{
//             apex_real_combo.push_back(std::make_pair(_apex_costs.at(id_real_combo.at(i).first), id_real_combo.at(i).second));
//             min_g2 = id_real_combo.at(i).second.at(1);
//         }
//     }
    
//     while(apex_real_combo.size() > solution_num){
//         auto flag = apex_real_combo.begin();  // with smallest contribution
//         int min_contri = INT_MAX;
//         for(auto iter = apex_real_combo.begin(); iter != apex_real_combo.end(); iter++){
//             int tr_x = std::next(iter) == apex_real_combo.end() ? max_x : std::next(iter)->second.at(0);
//             int tr_y = iter == apex_real_combo.begin() ? max_y : std::prev(iter)->second.at(1);
//             int contri = (tr_x - iter->second.at(0)) * (tr_y - iter->second.at(1));
//             if(contri < min_contri){
//                 min_contri = contri;
//                 flag = iter;
//             }
//         }
//         if(flag == apex_real_combo.begin()){
//             std::next(flag)->first = vector_min(flag->first, std::next(flag)->first);
//         }else if(std::next(flag) == apex_real_combo.end()){
//             std::prev(flag)->first = vector_min(flag->first, std::prev(flag)->first);
//         }else{
//             double eps1 = calculate_eps(flag->first, std::prev(flag)->second);
//             double eps2 = calculate_eps(flag->first, std::next(flag)->second);
//             if(eps1 < eps2){
//                 std::prev(flag)->first = vector_min(flag->first, std::prev(flag)->first);
//             }else{
//                 std::next(flag)->first = vector_min(flag->first, std::next(flag)->first);
//             }
//         }
//         apex_real_combo.erase(flag);
//     }
//     for(auto ele: apex_real_combo){
//         apex_costs.push_back(ele.first);
//         real_costs.push_back(ele.second);
//     }
// }


double Solver::CD(CostVector& a, CostVector& b, std::vector<double> box_len)
{
    size_t dist2 = 0;
    int dim = a.size();
    for(int i = 0; i < dim; i++){
        int dist = ceil(abs(double(a.at(i))-double(b.at(i)))/box_len.at(i));
        dist2 += dist*dist;
    }
    return (dist2 < dim+1 ? 1-double(dist2)/double(dim+1) : 0);
}

CostVector Solver::vector_min(CostVector& a, CostVector& b){
    CostVector  min_vector(a.size());
    for(int i = 0; i < a.size(); i++){
        min_vector.at(i) = a.at(i) <= b.at(i) ? a.at(i) : b.at(i);
    }
    return min_vector;
}

double Solver::calculate_eps(CostVector& a, CostVector& b){
    double eps = 0;
    for(int i = 0; i < a.size(); i++){
        eps = double(b.at(i)+0.1)/double(a.at(i)) - 1 > eps ? double(b.at(i)+0.1)/double(a.at(i)) - 1 : eps;
    }
    return eps;
}

void Solver::calculateCAT(HighLevelNodePtr node, CAT& cat, int agent_id)
{
    for(int i = 0; i < node->rep_id_list.size(); i++){
        if(i == agent_id){
            continue;
        }
        for(int j = 0; j < node->indiv_paths_list.at(i)[node->rep_id_list.at(i)].size(); j++){
            size_t node_id = node->indiv_paths_list.at(i)[node->rep_id_list.at(i)].at(j);
            cat.at(node_id).push_back(j);
        }
    }
}

std::tuple<double, double, double, double, int, int, int, int> Solver::search(size_t graph_size, std::vector<Edge>& edges, 
        boost::program_options::variables_map& vm, std::vector<std::pair<size_t, size_t>>& start_end, MergeStrategy& ms, LoggerPtr& logger, 
        HSolutionID& hsolution_ids, std::vector<CostVector>& hsolution_costs)
{
    std::vector<CostVector> hsolution_apex_costs;
    bool is_success = true;
    int turn_mode = vm["turn_mode"].as<int>();
    int turn_cost = vm["turn_cost"].as<int>();
    int agent_num = vm["agent_num"].as<int>();
    double Heps_merge = vm["hem"].as<double>();
    double heps_apex = vm["hep"].as<double>();
    double Heps_nonconflict_prune = 0.0;
    double heps_other = vm["eps"].as<double>();
    double Leps_merge = vm["lem"].as<double>();
    double Leps_prune = vm["lep"].as<double>();
    time_limit = vm["cutoffTime"].as<int>();
    bool if_CAT = vm["CAT"].as<std::string>() == "true" ? true : false;
    bool if_eager = vm["eager"].as<std::string>() == "true" ? true : false;
    std::string algorithm = vm["algorithm"].as<std::string>();
    SolveMode solve_mode;
    MergeStrategy l_ms = MergeStrategy::LEAST_CONFLICT;
    int solution_num = vm["solution_num"].as<int>();
    if(!if_CAT){
        l_ms = edges.front().cost.size() == 2 ? MergeStrategy::SMALLER_G2 : MergeStrategy::MORE_SLACK;
    }else if(algorithm != "Apex"){
        if_CAT = false;
    }
    if(vm["mode"].as<std::string>() == "given_eps"){
        solve_mode = SolveMode::GIVEN_EPSILON;
    }else if(vm["mode"].as<std::string>() == "smallest_eps"){
        solve_mode = SolveMode::SMALLEST_EPS;
        heps_apex = 0;
        if(algorithm != "Apex"){
            output << "smallest_eps only for A*pex";
            exit(1);
        }
    }else if(vm["mode"].as<std::string>() == "diversity"){
        solve_mode = SolveMode::DIVERSITY;
        heps_apex = 0;
        Leps_merge = 0;
        Leps_prune = 0;
        if(algorithm != "Apex"){
            output << "diversity only for A*pex";
            exit(1);
        }
    }else{
        output << "solve mode only \"given_eps\", \"smallest_eps\", \"diversity\" ";
        exit(1);
    }
    
    double NonDomTime = 0, LowLevelTime = 0, DomPruneTime = 0, ConflictionTime = 0, EagerTime;
    int DomPruneNum = 0, solution_number = 0;

    ConflictChecker conflict_checker;

    start_time = std::clock();
    auto t0 = std::chrono::high_resolution_clock::now();    // record time
    HLQueue open_list;

// calculate heuristic
    std::vector<Heuristic> heuristics(agent_num);
    AdjacencyMatrix graph(graph_size, edges);   // can run outside and only once
    AdjacencyMatrix inv_graph(graph_size, edges, true);
    for(int i = 0; i < agent_num; i++){
        ShortestPathHeuristic sp_heuristic(start_end.at(i).second, graph_size, inv_graph, turn_mode, turn_cost);
        heuristics.at(i) = std::bind( &ShortestPathHeuristic::operator(), sp_heuristic, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3);
    }

//  initialize open_list
    HighLevelNodePtr root_node = std::make_shared<HighLevelNode>(agent_num);
    CAT cat(graph_size);

    for(size_t i = 0; i < agent_num; i ++){
    auto _t3 = std::chrono::high_resolution_clock::now();
        single_run_map(graph_size, graph, heuristics.at(i), start_end.at(i).first, start_end.at(i).second, 
            algorithm, l_ms, logger, Leps_merge, Leps_prune, time_limit, root_node->indiv_paths_list.at(i), 
            root_node->indiv_apex_costs.at(i), root_node->indiv_real_costs.at(i), 
            root_node->vertex_constraints.at(i), root_node->edge_constraints.at(i), cat, root_node->conflict_num, turn_mode, turn_cost);
    auto t4 = std::chrono::high_resolution_clock::now(); // for timing.
    auto duration_t = std::chrono::duration_cast<std::chrono::microseconds>(t4 - _t3);
    LowLevelTime += ((double)duration_t.count())/1000000.0;
    std::cout << "Agent ID: " << i << " Low Level Time = " << ((double)duration_t.count())/1000000.0 << "  size = " << root_node->indiv_paths_list.at(i).size() << std::endl;
    }

    auto tt1 = std::chrono::high_resolution_clock::now();
    if(algorithm == "Apex"){
        if(solve_mode == SolveMode::SMALLEST_EPS || solve_mode == SolveMode::DIVERSITY){
            NonDomJointPath(root_node, solution_num);
        }else{
            NonDomJointPath(root_node, ms, Heps_merge);
        }
    }else{
        NonDomJointPath(root_node);
    }

    auto tt2 = std::chrono::high_resolution_clock::now(); // for timing.
    auto duration_tt = std::chrono::duration_cast<std::chrono::microseconds>(tt2 - tt1);
    NonDomTime += ((double)duration_tt.count())/1000000.0;

    root_node->rep_id_list = root_node->joint_path_list.front().second;
    root_node->rep_apex_cost = root_node->joint_path_list.front().first;
    open_list.insert(root_node);


    std::tuple<int, int, CostVector, size_t> cft;
//  main loop    
    size_t constraint_num = 0;
    while(!open_list.empty())
    {
        if((std::clock() - start_time)/CLOCKS_PER_SEC > time_limit){
            is_success = false;
            break;
        }
        auto node = open_list.pop();

        auto current_path = node->joint_path_list.front();

    // eager solution update
        auto _tt1 = std::chrono::high_resolution_clock::now();
        if(if_eager){
            //  exhaust all joint path in high-level node, if no collision, then add to solution set
            //  remark: this cannot ensure non-dominated set
            for(auto iter = node->joint_path_list.begin(); iter != node->joint_path_list.end(); ){
                cft = conflict_checker.is_conflict(*iter, node->indiv_paths_list);
                if(std::get<2>(cft).empty()){   // collision-free, use weakly dominate to prune
                    CostVector  real_cost(iter->first.size(), 0);
                    for(int i = 0; i < agent_num; i++){
                        add_cost(real_cost, node->indiv_real_costs.at(i)[iter->second.at(i)]);
                    }
                    //  dominance check
                    bool if_dominated = false;
                    for(int i = 0; i < hsolution_costs.size(); i++){
                        if(is_dominated(real_cost, hsolution_costs.at(i))){
                            hsolution_apex_costs.at(i) = vector_min(hsolution_apex_costs.at(i), iter->first);
                            if_dominated = true;
                            break;
                        }
                    }

                    if(!if_dominated){   
                        //  add to solution
                        int flag = 0;
                        for(auto iter1 = hsolution_costs.begin(); iter1 != hsolution_costs.end(); ){
                            if(is_dominated(*iter1, real_cost)){
                                iter->first = vector_min(iter->first, hsolution_apex_costs.at(flag));
                                iter1 = hsolution_costs.erase(iter1);
                                hsolution_apex_costs.erase(hsolution_apex_costs.begin()+flag);
                                hsolution_ids.erase(hsolution_ids.begin()+flag);
                            }else{
                                flag ++;
                                iter1 ++;
                            }
                        }

                        std::vector<std::vector<size_t>> new_hsolution;
                        for(int i = 0; i < agent_num; i ++){
                            new_hsolution.push_back(node->indiv_paths_list.at(i)[iter->second.at(i)]);
                        }
                        hsolution_ids.push_back(new_hsolution);
                        hsolution_costs.push_back(real_cost);
                        hsolution_apex_costs.push_back(iter->first);
                        std::cout << "there is a solution" << std::endl;
                        solution_number ++;
                        
                        if(solve_mode == SolveMode::DIVERSITY){
                            MergeBySmallestEps(hsolution_apex_costs, hsolution_costs, solution_num);
                            for(int i = 0; i < hsolution_apex_costs.size(); i++){
                                double _eps = calculate_eps(hsolution_apex_costs.at(i), hsolution_costs.at(i));
                                heps_apex = heps_apex > _eps ? heps_apex : _eps;
                            }
                            Leps_merge = heps_apex;
                            Leps_prune = heps_apex;
                        }
                        // if(solve_mode == SolveMode::SMALLEST_EPS || solve_mode == SolveMode::DIVERSITY){
                        //     double _eps = calculate_eps(iter->first, real_cost);
                        //     heps_apex = heps_apex > _eps ? heps_apex : _eps;
                        // }
                    }

                    iter = node->joint_path_list.erase(iter);
                    continue;
                }else{
                    iter ++;
                }
            }

            if(node->joint_path_list.empty()){
                continue;
            }
        }
        auto _tt2 = std::chrono::high_resolution_clock::now();
        auto durationtt = std::chrono::duration_cast<std::chrono::microseconds>(_tt2-_tt1);
        EagerTime += ((double)durationtt.count())/1000000.0;

    //  DomPrune
        double prune_eps = algorithm == "Apex" ? heps_apex : heps_other;
        while(!node->joint_path_list.empty()){
            bool is_pruned = false;
            for(int i = 0; i < hsolution_costs.size(); i++){
                if(is_dominated(node->joint_path_list.front().first, hsolution_costs.at(i), prune_eps)){
                    hsolution_apex_costs.at(i) = vector_min(hsolution_apex_costs.at(i), node->joint_path_list.front().first);
                    is_pruned = true;
                    break;
                }
            }
            if(is_pruned){
                DomPruneNum ++;
                node->joint_path_list.pop_front();
            }else{
                break;
            }
        }

        if(node->joint_path_list.empty()){
            continue;
        }

        node->rep_id_list = node->joint_path_list.front().second;
        node->rep_apex_cost = node->joint_path_list.front().first;

        if(node->joint_path_list.front() != current_path){
            open_list.insert(node);
            continue;
        }

        cft = conflict_checker.is_conflict(node->joint_path_list.front(), node->indiv_paths_list);

        if(std::get<2>(cft).empty()){
            if(if_eager){
                output << "cft not empty under eager solution update";
                exit(1);
            }else{
                //  solution update
                CostVector  real_cost(node->rep_apex_cost.size(), 0);
                std::vector<std::vector<size_t>> new_hsolution;
                for(int i = 0; i < agent_num; i++){
                    add_cost(real_cost, node->indiv_real_costs.at(i)[node->rep_id_list.at(i)]);
                    new_hsolution.push_back(node->rep_id_list);
                }
                hsolution_ids.push_back(new_hsolution);
                hsolution_costs.push_back(real_cost);
                hsolution_apex_costs.push_back(node->rep_apex_cost);
                std::cout << "there is a solution" << std::endl;
                solution_number ++;

                node->joint_path_list.pop_front();

                if(node->joint_path_list.empty()){
                    continue;
                }

                node->rep_id_list = node->joint_path_list.front().second;
                node->rep_apex_cost = node->joint_path_list.front().first;

                if(solve_mode == SolveMode::SMALLEST_EPS || solve_mode == SolveMode::DIVERSITY){
                    double _eps = calculate_eps(node->rep_apex_cost, real_cost);
                    heps_apex = heps_apex > _eps ? heps_apex : _eps;
                }

                open_list.insert(node);
                continue;
            }
        }
        
        // print constraint info
        auto _t1 = std::chrono::high_resolution_clock::now();
        constraint_num ++;
        if (constraint_num % (500/agent_num) == 0) {
            auto tnow = std::chrono::high_resolution_clock::now(); // for timing.
            auto duration1 = std::chrono::duration_cast<std::chrono::microseconds>(tnow - t0);
            std::cout << "[INFO] * Solver::Search, after " << constraint_num << " conflict splits " 
            << "       time = " << ((double)duration1.count())/1000000.0 << std::endl;
            std::cout << map_name << std::endl;
        }


        if(std::get<2>(cft).size() == 1){   // vertex confliction, split 1 or 2 constraints
            for(int i = 0; i < 2; i ++){
                int agent_id = i == 0 ? std::get<0>(cft) : std::get<1>(cft);
                if(agent_id < 0){
                    continue;
                }
                auto new_node = std::make_shared<HighLevelNode>(*node);
                new_node->indiv_paths_list.at(agent_id).clear();
                new_node->indiv_apex_costs.at(agent_id).clear();
                new_node->indiv_real_costs.at(agent_id).clear();
                new_node->conflict_num.clear();

                add_constraint(new_node->vertex_constraints, agent_id, std::get<2>(cft).front(), std::get<3>(cft));

            //  Low Level Search
                CAT _cat(graph_size);
                if(algorithm == "Apex" && if_CAT){
                    calculateCAT(node, _cat, agent_id);
                }
               

                auto _t_0_ = std::chrono::high_resolution_clock::now(); // for timing.
                single_run_map(graph_size, graph, heuristics.at(agent_id), start_end.at(agent_id).first, start_end.at(agent_id).second, 
                    algorithm, l_ms, logger, Leps_merge, Leps_prune, time_limit, new_node->indiv_paths_list.at(agent_id), 
                    new_node->indiv_apex_costs.at(agent_id), new_node->indiv_real_costs.at(agent_id), 
                    new_node->vertex_constraints[agent_id], new_node->edge_constraints[agent_id], _cat, new_node->conflict_num, turn_mode, turn_cost); 
                auto _t_1_ = std::chrono::high_resolution_clock::now();
                auto duration0__ = std::chrono::duration_cast<std::chrono::microseconds>(_t_1_ - _t_0_);
                std::cout << "LowLevelTime = " << ((double)duration0__.count())/1000000.0 << std::endl;
                LowLevelTime += ((double)duration0__.count())/1000000.0;

                if(new_node->indiv_paths_list.at(agent_id).empty()){
                    continue;
                }

            //  Calculate NonDomSet
                auto __t1 = std::chrono::high_resolution_clock::now();
                if(algorithm == "Apex"){
                    if(solve_mode == SolveMode::SMALLEST_EPS || solve_mode == SolveMode::DIVERSITY){
                        NonDomJointPath(new_node, solution_num);
                    }else{
                        if(if_CAT){
                            NonDomJointPath(new_node, ms, Heps_merge, agent_id);
                        }else{
                            NonDomJointPath(new_node, ms, Heps_merge);
                        }
                    }
                }else{
                    NonDomJointPath(new_node);
                }
                if((std::clock() - start_time)/CLOCKS_PER_SEC > time_limit){
                    is_success = false;
                    continue;
                }
                auto __t2 = std::chrono::high_resolution_clock::now(); // for timing.
                auto duration2_ = std::chrono::duration_cast<std::chrono::microseconds>(__t2 - __t1);
                NonDomTime += ((double)duration2_.count()) / 1000000.0;
                
                new_node->rep_id_list = new_node->joint_path_list.front().second;
                new_node->rep_apex_cost = new_node->joint_path_list.front().first;
                open_list.insert(new_node);
            }

        }else{  //  edge confliction
            for(int i = 0; i < 2; i++){
                int agent_id = i == 0 ? std::get<0>(cft) : std::get<1>(cft);
                auto new_node = std::make_shared<HighLevelNode>(*node);
                new_node->indiv_paths_list.at(agent_id).clear();
                new_node->indiv_apex_costs.at(agent_id).clear();
                new_node->indiv_real_costs.at(agent_id).clear();
                new_node->conflict_num.clear();

                add_constraint(new_node->edge_constraints, agent_id, std::get<2>(cft).at(i), std::get<2>(cft).at(1-i), std::get<3>(cft));

                CAT _cat(graph_size);
                if(algorithm == "Apex" && if_CAT){
                    calculateCAT(node, _cat, agent_id);
                }


                auto _t_0_ = std::chrono::high_resolution_clock::now(); // for timing.
                single_run_map(graph_size, graph, heuristics.at(agent_id), start_end.at(agent_id).first, start_end.at(agent_id).second, 
                    algorithm, l_ms, logger, Leps_merge, Leps_prune, time_limit, new_node->indiv_paths_list.at(agent_id), 
                    new_node->indiv_apex_costs.at(agent_id), new_node->indiv_real_costs.at(agent_id), 
                    new_node->vertex_constraints[agent_id], new_node->edge_constraints[agent_id], _cat, new_node->conflict_num, turn_mode, turn_cost); 
                auto _t_1_ = std::chrono::high_resolution_clock::now();
                auto duration0__ = std::chrono::duration_cast<std::chrono::microseconds>(_t_1_ - _t_0_);
                std::cout << "LowLevelTime = " << ((double)duration0__.count())/1000000.0 << std::endl;
                LowLevelTime += ((double)duration0__.count())/1000000.0;

                if(new_node->indiv_paths_list.at(agent_id).empty()){
                    continue;
                }

            //  Calculate NonDomSet
                auto __t1 = std::chrono::high_resolution_clock::now();
                if(algorithm == "Apex"){
                    if(solve_mode == SolveMode::SMALLEST_EPS || solve_mode == SolveMode::DIVERSITY){
                        NonDomJointPath(new_node, solution_num);
                    }else{
                        if(if_CAT){
                            NonDomJointPath(new_node, ms, Heps_merge, agent_id);
                        }else{
                            NonDomJointPath(new_node, ms, Heps_merge);
                        }
                    }
                }else{
                    NonDomJointPath(new_node);
                }

                if((std::clock() - start_time)/CLOCKS_PER_SEC > time_limit){
                    is_success = false;
                    continue;
                }

                if(new_node->joint_path_list.empty()){
                    continue;
                }

                auto __t2 = std::chrono::high_resolution_clock::now(); // for timing.
                auto duration2_ = std::chrono::duration_cast<std::chrono::microseconds>(__t2 - __t1);
                NonDomTime += ((double)duration2_.count()) / 1000000.0;

                new_node->rep_id_list = new_node->joint_path_list.front().second;
                new_node->rep_apex_cost = new_node->joint_path_list.front().first;
                open_list.insert(new_node);
            }
        }
    }
    auto t1 = std::chrono::high_resolution_clock::now(); // for timing.
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(t1 - t0);
    double TotalTime = ((double)duration.count())/1000000.0;

//  post process
    if(solve_mode != SolveMode::SMALLEST_EPS && solve_mode != SolveMode::DIVERSITY){
        auto _hsolution_costs = hsolution_costs;
        auto _hsolution_apex_costs = hsolution_apex_costs;
        hsolution_costs.clear(); hsolution_apex_costs.clear();
        hsolution_costs.shrink_to_fit(); hsolution_apex_costs.shrink_to_fit();
        for(int i = 0; i < _hsolution_apex_costs.size(); i++){
            bool if_merged = false;
            for(int j = 0; j < hsolution_apex_costs.size(); j++){
                if(calculate_eps(_hsolution_apex_costs.at(i), hsolution_costs.at(j)) < heps_apex){
                    hsolution_apex_costs.at(j) = vector_min(hsolution_apex_costs.at(j), _hsolution_apex_costs.at(i));
                    if_merged = true;
                    break;
                }
                if(calculate_eps(hsolution_apex_costs.at(j), _hsolution_costs.at(i)) < heps_apex){
                    hsolution_apex_costs.at(j) = vector_min(hsolution_apex_costs.at(j), _hsolution_apex_costs.at(i));
                    hsolution_costs.at(j) = _hsolution_costs.at(i);
                    if_merged = true;
                    break;
                }
            }
            if(!if_merged){
                hsolution_costs.push_back(_hsolution_costs.at(i));
                hsolution_apex_costs.push_back(_hsolution_apex_costs.at(i));
            }
        }
    }else if(solve_mode == SolveMode::SMALLEST_EPS){
        // MergeBySmallestEps(hsolution_apex_costs, hsolution_costs, solution_num);
    }if(solve_mode == SolveMode::DIVERSITY){
        // MergeBySmallestEps
    }
    
    std::vector<std::pair<CostVector, int>> apex_id_list(hsolution_costs.size());
    auto _hsolution_costs = hsolution_costs;
    for(int i = 0; i < hsolution_costs.size(); i++){
        apex_id_list.at(i) = std::make_pair(hsolution_apex_costs.at(i), i);
    }
    hsolution_costs.clear(); hsolution_costs.shrink_to_fit();
    hsolution_apex_costs.clear(); hsolution_apex_costs.shrink_to_fit();    
    std::sort(apex_id_list.begin(), apex_id_list.end(), [](const std::pair<CostVector, int>& a, const std::pair<CostVector, int>& b){
        for(int i = 0; i < a.first.size(); i++){
            if(a.first.at(i) != b.first.at(i)){
                return a.first.at(i) < b.first.at(i);
            }
        }
        return true;
    });
    for(auto ele: apex_id_list){
        hsolution_apex_costs.push_back(ele.first);
        hsolution_costs.push_back(_hsolution_costs.at(ele.second));
    }
    
    if(solve_mode == SolveMode::SMALLEST_EPS || solve_mode == SolveMode::DIVERSITY){
        for(int i = 0; i < hsolution_apex_costs.size(); i ++){
            double _eps = calculate_eps(hsolution_apex_costs.at(i), hsolution_costs.at(i));
            heps_apex = heps_apex > _eps ? heps_apex : _eps;
        }
    }  

    if(is_success){
        output << "SUCCESS" << std::endl;
    }else{
        output << "FAIL" << std::endl;
    }
    output << "apex cost: " << std::endl;
    int i = 0;
    for(size_t num = 0; num < hsolution_apex_costs.size(); num ++){
        if(i++ == 7){
            output << std::endl;
            i = 1;
        }
        if(vm["dim"].as<int>() == 2){
            output << "{" << hsolution_apex_costs.at(num).at(0) << ", " << hsolution_apex_costs.at(num).at(1);
        }else{
            output << "{" << hsolution_apex_costs.at(num).at(0) << ", " << hsolution_apex_costs.at(num).at(1) << ", " << hsolution_apex_costs.at(num).at(2);
        }
        output << "}, ";
    }
    output << std::endl;
    output << "real cost: " << std::endl;
    int j = 0;
    for(size_t num = 0; num < hsolution_costs.size(); num ++){
        if(j++ == 7){
            output << std::endl;
            j = 1;
        }
        if(vm["dim"].as<int>() == 2){
            output << "{" << hsolution_costs.at(num).at(0) << ", " << hsolution_costs.at(num).at(1);
        }else{
            output << "{" << hsolution_costs.at(num).at(0) << ", " << hsolution_costs.at(num).at(1) << ", " << hsolution_costs.at(num).at(2);
        }
        output << "}, ";
    }
    output << std::endl;
    if(solve_mode == SolveMode::SMALLEST_EPS || solve_mode == SolveMode::DIVERSITY){
        output << "epsilon = " << heps_apex << std::endl;
    }
    output << "SolutionNumber = " << solution_number << "/" << hsolution_costs.size() << std::endl << std::endl;
    

    return std::tuple<double, double, double, double, int, int, int, int>(NonDomTime, LowLevelTime, TotalTime, EagerTime, DomPruneNum, constraint_num, 
        solution_number, hsolution_costs.size());
}