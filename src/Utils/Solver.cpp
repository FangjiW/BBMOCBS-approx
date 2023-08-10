#include "Utils/Solver.h"
#include <cstdlib>
#include <list>
#include <algorithm>
#include <set>
#include <random>

template <typename T>
bool less_than(const T &a, const T &b, bool lexico=true)
{
    if(lexico == true){  // lexicographic
        auto vec1 = std::get<0>(a), vec2 = std::get<0>(b);
        for(int i = 0; i < vec1.size(); i ++){
            if(vec1.at(i) != vec2.at(i)){
                return vec1.at(i) < vec2.at(i);
            }
        }
        return true;
    }else{  // only compare the first component
        return std::get<0>(a).at(0) <= std::get<0>(b).at(0);
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
} less_than_;

void Solver::add_constraint(std::vector<VertexConstraint>& vertex_constraints, size_t agent_id, size_t node_id, size_t time)
{
    vertex_constraints[agent_id][time].push_back(node_id);
}

void Solver::add_constraint(std::vector<EdgeConstraint>& edge_constraints, size_t agent_id, size_t source, size_t target, size_t time)
{
    edge_constraints.at(agent_id)[source][time].push_back(target);
}

bool Solver::DomPrune(std::vector<CostVector>& solution_costs, std::list<JointPathPair>& joint_path_list, std::vector<CostSet>& indiv_real_costs, double eps, int& DomPruneNum)
{
    int agent_num = joint_path_list.front().second.size();
    bool is_prune = false;
    for (auto iter = joint_path_list.begin(); iter != joint_path_list.end(); ) {
        CostVector real_cost(joint_path_list.front().first.size(), 0);
        for(int i = 0; i < agent_num; i ++){
            add_cost(real_cost, indiv_real_costs.at(i)[iter->second.at(i)]);
        }
        
        bool is_dominated = false;
        for (auto solution_cost : solution_costs) {
            // std::cout << solution_cost.at(0) << ", " << solution_cost.at(1) << ", " << solution_cost.at(2) << std::endl;
            // std::cout << iter->first.at(0) << ", " << iter->first.at(1) << ", " << iter->first.at(2) << std::endl;
            // getchar();
            bool is_dominated_by_this_one = true;
            for (int i = 0; i < solution_cost.size(); i++) {
                if (real_cost.at(i) * (1 + eps) < solution_cost.at(i)) {
                    is_dominated_by_this_one = false;
                    break;
                }
            }
            if (is_dominated_by_this_one) {
                DomPruneNum ++;
                // getchar();
                is_dominated = true;
                iter = joint_path_list.erase(iter);
                is_prune = true;
                break;
            }
        }
        if (!is_dominated) {
            return is_prune;
        }
    }
    return is_prune;
}

bool Solver::DomPrune(std::vector<CostVector>& solution_costs, JointPathPair& joint_path, double eps)
{
    for (auto& solution_cost : solution_costs) {
        bool is_dominated = true;
        for (int i = 0; i < solution_cost.size(); i++) {
            if (joint_path.first.at(i) * (1 + eps) < solution_cost.at(i)) {
                is_dominated = false;
                break;
            }
        }
        if(is_dominated){
            return true;
        }
    }
    return false;
}


// A*pex version
void Solver::NonDomJointPath(HighLevelNodePtr node, MergeStrategy ms, double eps)
{
    for(auto e : node->indiv_paths_list){
        std::cout << e.size() << " ";
    }
    auto _t1 = std::chrono::high_resolution_clock::now();
    JointPathTuple temp;
    node->joint_path_list.clear();
    
    std::list<JointPathTuple> joint_path_vector;

    for(size_t id = 0; id < node->indiv_paths_list.at(0).size(); id++){
        auto element = node->indiv_paths_list.at(0)[id];
        joint_path_vector.push_back(std::make_tuple(node->indiv_apex_costs.at(0)[id], node->indiv_real_costs.at(0)[id], std::vector<size_t>({id})));
    }

    for(int i = 1; i < node->indiv_paths_list.size(); i++){
        std::list<JointPathTuple> _joint_path_vector(joint_path_vector);
        joint_path_vector.clear();
        for(const auto& joint_path : _joint_path_vector){
            for(const auto& indiv_path : node->indiv_paths_list.at(i)){
                temp = joint_path;
                add_cost(std::get<0>(temp), node->indiv_apex_costs.at(i)[indiv_path.first]);
                add_cost(std::get<1>(temp), node->indiv_real_costs.at(i)[indiv_path.first]);
                std::get<2>(temp).push_back(indiv_path.first);
                joint_path_vector.push_back(std::make_tuple(std::get<0>(temp), std::get<1>(temp), std::get<2>(temp)));
                // std::push_heap(_joint_path_vector.begin(), _joint_path_vector.end(), less_than);
            }
        }
        joint_path_vector.sort([](const JointPathTuple& a, JointPathTuple& b){
            return less_than<JointPathTuple>(a, b);});        
        NonDomVec(joint_path_vector, ms, eps);
    }

    for(auto& element : joint_path_vector){
        node->joint_path_list.push_back(std::make_pair(std::get<0>(element), std::get<2>(element)));
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
    // getchar();
}


// A*pex version
void Solver::NonDomVec(std::list<JointPathTuple>& joint_path_vector, MergeStrategy ms, double eps)
{
    if (std::get<0>(joint_path_vector.front()).size() == 2) {
        size_t g_min = UINT64_MAX;
        for(auto iter = joint_path_vector.begin(); iter != joint_path_vector.end(); ) {
            if (std::get<0>(*iter).at(1) < g_min) {
                g_min = std::get<0>(*iter).at(1);
                bool is_merge = false;
                for (auto iter1 = joint_path_vector.begin(); iter1 != iter; iter1++) {
                    if (HighLevelMerge(*iter1, *iter, ms, eps)) {
                        // std::cout << "you are there" << std::get<0>(*iter).size();
                        // getchar();
                        iter = joint_path_vector.erase(std::next(iter1), std::next(iter));
                        is_merge = true;
                        break;
                    }
                }
                if (!is_merge) {
                    iter ++;
                }
            }else{
                iter = joint_path_vector.erase(iter);
            }
        }
    }else if(std::get<0>(joint_path_vector.front()).size() == 3){
	// dominance check
		std::set<CostVector, Less_Than_>  domination_set;
		for(auto iter = joint_path_vector.begin(); iter != joint_path_vector.end(); ){
        //  dominance check: if apex be strictly dominated(not eps-), then prune; 
        //  else erase the behind strictly dominated ones in RB-tree
			std::vector<size_t>  trun_vec({std::get<0>(*iter).at(1), std::get<0>(*iter).at(2)});
            if(domination_set.empty()){
                domination_set.insert(trun_vec);
                iter ++;
                continue;
            }
			auto it = domination_set.upper_bound(trun_vec);
            if(it != domination_set.begin() && (*std::prev(it)).at(1) <= trun_vec.at(1)){
                iter = joint_path_vector.erase(iter);
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

        //  merge
                bool is_merge = false;
                for (auto iter1 = joint_path_vector.begin(); iter1 != iter; iter1++) {
                    // comment: even if merging, the RB-tree don't need changing
                    if(HighLevelMerge(*iter1, *iter, ms, eps)){  
                        iter = joint_path_vector.erase(iter);
                        is_merge = true;
                        break;
                    }
                }
                if (!is_merge) {
                    iter ++;
                }
            }
		}
	}else{
        std::cout << std::endl << "(A*pex) ERROR: only g size = 2 or 3" << std::endl;
    }
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
            // std::pop_heap(_joint_path_vector.begin(), _joint_path_vector.end(), more_than_);
            // _joint_path_vector.pop_back();
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
        std::cout << std::endl << "BOA*/NAMOA* ERROR: only g size = 2 or 3" << std::endl;
        exit(1);
    }
}


bool Solver::HighLevelMerge(JointPathTuple& existing_path, JointPathTuple& new_path, MergeStrategy ms, double eps)
{
    // return false;
    CostVector apex(std::get<0>(existing_path).size());
    for(int i = 0; i < apex.size(); i ++){
        apex.at(i) = std::min(std::get<0>(existing_path).at(i), std::get<0>(new_path).at(i));
    }
    if(ms == SMALLER_G2){
        if(apex.size() != 2){
            std::cerr << "SMALLER_G2 can only used for bi-objectives";
            exit(-1);
        }else{
            if(std::get<0>(existing_path).at(1) < std::get<0>(new_path).at(1)){
                if(std::get<1>(existing_path).at(1) < ((double)apex.at(1))*(1+eps)){
                    existing_path = std::make_tuple(apex, std::get<1>(existing_path), std::get<2>(existing_path));
                    return true;
                }else{
                    if(std::get<1>(new_path).at(0) < ((double)apex.at(0))*(1+eps)){
                        existing_path = std::make_tuple(apex, std::get<1>(new_path), std::get<2>(new_path));
                        return true;
                    }
                    return false;
                }
            }else{
                if(std::get<1>(new_path).at(0) < ((double)apex.at(0))*(1+eps)){
                    existing_path = std::make_tuple(apex, std::get<1>(new_path), std::get<2>(new_path));
                    return true;
                }else{
                    if(std::get<1>(existing_path).at(1) < ((double)apex.at(1))*(1+eps)){
                        existing_path = std::make_tuple(apex, std::get<1>(existing_path), std::get<2>(existing_path));
                        return true;
                    }
                    return false;
                }
            }
        }
    }else if(ms == RANDOM){
        auto seed = std::chrono::high_resolution_clock::now().time_since_epoch().count();
        std::mt19937 rng(seed);
        std::uniform_real_distribution<double> real_dist(0.0, 1.0);
        if(real_dist(rng) < 0.5){
        // if(0 < 0.5){
            bool can_merge1 = true, can_merge2 = true;
            for(int i = 0; i < apex.size(); i++){
                if(std::get<1>(existing_path).at(i) >= ((double)apex.at(i))*(1+eps)){
                    can_merge1 = false;
                }
            }
            if(can_merge1){
                existing_path = std::make_tuple(apex, std::get<1>(existing_path), std::get<2>(existing_path));
                return true;
            }
            for(int i = 0; i < apex.size(); i++){
                if(std::get<1>(new_path).at(i) >= ((double)apex.at(i))*(1+eps)){
                    can_merge2 = false;
                }
            }
            if(can_merge2){
                existing_path = std::make_tuple(apex, std::get<1>(new_path), std::get<2>(new_path));
                return true;
            }
            return false;
        }else{
            bool can_merge1 = true, can_merge2 = true;
            for(int i = 0; i < apex.size(); i++){
                if(std::get<1>(new_path).at(i) >= ((double)apex.at(i))*(1+eps)){
                    can_merge1 = false;
                }
            }
            if(can_merge1){
                existing_path = std::make_tuple(apex, std::get<1>(new_path), std::get<2>(new_path));
                return true;
            }
            for(int i = 0; i < apex.size(); i++){
                if(std::get<1>(existing_path).at(i) >= ((double)apex.at(i))*(1+eps)){
                    can_merge2 = false;
                }
            }
            if(can_merge2){
                existing_path = std::make_tuple(apex, std::get<1>(existing_path), std::get<2>(existing_path));
                return true;
            }
            return false;
        }
    }else{
        std::cout << std::endl << "merge strategy only SMALLER_G2 and RANDOM";
        exit(1);
    }
}


size_t Solver::search(size_t graph_size, std::vector<Edge>& edges, boost::program_options::variables_map& vm, 
        std::vector<std::pair<size_t, size_t>> start_end, MergeStrategy& ms, LoggerPtr& logger, 
        HSolutionID& hsolution_ids, std::vector<CostVector>& hsolution_costs, std::ofstream& Output)
{
    bool is_success = true;
    auto start_time = std::clock();

    int agent_num = vm["agent_num"].as<int>();
    double Heps_merge_max = vm["hem"].as<double>();
    double Heps_conflict_prune = vm["hep"].as<double>();
    double Heps_nonconflict_prune = 0.001;
    // double Heps_conflict_prune = 0;
    double Leps_merge = vm["lem"].as<double>();
    double Leps_prune = vm["lep"].as<double>();
    double time_limit = vm["cutoffTime"].as<int>();
    std::string algorithm = vm["algorithm"].as<std::string>();
    std::string output = vm["output"].as<std::string>();
    MergeStrategy l_ms = MORE_SLACK;
    if(vm["dim"].as<int>() == 2){
        l_ms = SMALLER_G2;
    }
    
    double Heps_merge = Heps_merge_max;

    double NonDomTime = 0;
    double LowLevelTime = 0;
    double DomPruneTime = 0;
    double ConflictionTime = 0;

    int DomPruneNum = 0;
    auto t0 = std::chrono::high_resolution_clock::now();    // record time
    HLQueue open_list;
    int extra_solution = 0;

    ConflictChecker conflict_checker;

//  initialize open_list
    HighLevelNodePtr root_node = std::make_shared<HighLevelNode>(agent_num);
    for(size_t i = 0; i < agent_num; i ++){
        single_run_map(graph_size, edges, start_end.at(i).first, start_end.at(i).second, output, 
            algorithm, l_ms, logger, Leps_merge, Leps_prune, time_limit, root_node->indiv_paths_list.at(i), 
            root_node->indiv_apex_costs.at(i), root_node->indiv_real_costs.at(i), 
            root_node->vertex_constraints.at(i), root_node->edge_constraints.at(i));
    }
    if(algorithm == "Apex"){
        NonDomJointPath(root_node, ms, Heps_merge);
    }else{
        NonDomJointPath(root_node);
    }
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
    

        // Dom Prune 
        auto _t3 = std::chrono::high_resolution_clock::now();
        if(DomPrune(hsolution_costs, node->joint_path_list, node->indiv_real_costs, Heps_nonconflict_prune, DomPruneNum)){
            if(node->joint_path_list.empty()){
                continue;
            }
            node->rep_id_list = node->joint_path_list.front().second;
            node->rep_apex_cost = node->joint_path_list.front().first;
            open_list.insert(node);
            continue;
        }
        cft = conflict_checker.is_conflict(node);

        //  Conflict check
        if(!std::get<2>(cft).empty()){
            // non-conflict, if eps-dominated, then prune
            if(DomPrune(hsolution_costs, node->joint_path_list.front(), Heps_conflict_prune)){
                DomPruneNum ++;
                node->joint_path_list.pop_front();
                if(node->joint_path_list.empty()){
                    continue;
                }
                DomPrune(hsolution_costs, node->joint_path_list, node->indiv_real_costs, Heps_nonconflict_prune, DomPruneNum);
                if(node->joint_path_list.empty()){
                    continue;
                }
                node->rep_id_list = node->joint_path_list.front().second;
                node->rep_apex_cost = node->joint_path_list.front().first;
                open_list.insert(node);
                continue;
            }
        }
        auto t4 = std::chrono::high_resolution_clock::now(); // for timing.
        auto duration = std::chrono::duration_cast<std::chrono::microseconds>(t4 - _t3);
        DomPruneTime += ((double)duration.count())/1000000.0;

        
        if(std::get<2>(cft).empty()){   // generate solution
            std::vector<std::vector<size_t>> new_hsolution;
            CostVector  new_cost(node->rep_apex_cost.size(), 0);
            for(int i = 0; i < agent_num; i ++){
                new_hsolution.push_back(node->indiv_paths_list.at(i)[node->rep_id_list.at(i)]);
                add_cost(new_cost, node->indiv_real_costs.at(i)[node->rep_id_list.at(i)]);
            }
            hsolution_ids.push_back(new_hsolution);
            hsolution_costs.push_back(new_cost);

            node->joint_path_list.pop_front();
            if(!node->joint_path_list.empty()){
                node->rep_id_list = node->joint_path_list.front().second;
                node->rep_apex_cost = node->joint_path_list.front().first;
                open_list.insert(node);
            }
            continue;
        }

    // //  print confliction information
    //     if(std::get<2>(cft).size() == 2){

    //     int _id1 = std::get<0>(cft);
    //     int _id2 = std::get<1>(cft);
    //     int id1 = _id1 >= 0 ? _id1 : -_id1-1;
    //     int id2 = _id2 >= 0 ? _id2 : -_id2-1;
    //     std::cout << std::endl << std::endl << "agent1: " << id1 << "  agent2: " << id2 << "  state: {" 
    //         << std::get<2>(cft).front()/32 << ", " << std::get<2>(cft).front()%32
    //          << "}    t:" << std::get<3>(cft) << std::endl << "path1: ";
    //     for(size_t ii : node->indiv_paths_list.at(id1)[node->rep_id_list.at(id1)]){
    //         std::cout << "{" << ii/32 << ", " << ii%32 << "}, ";
    //     }
    //     std::cout << std::endl << "path2: ";
    //     for(size_t ii : node->indiv_paths_list.at(id2)[node->rep_id_list.at(id2)]){
    //         std::cout << "{" << ii/32 << ", " << ii%32 << "}, ";
    //     }
    //     std::cout << std::endl << std::endl;
    //     std::cout << "constraint: ";
    //     int nn = 0;
    //     for(auto ele : node->vertex_constraints){
    //         bool ifprint = false;
    //         for(auto ele1 : ele){
    //             if(ele1.second.empty()){
    //                 continue;
    //             }
    //             if(!ifprint){
    //                 std::cout << std::endl  << "agent: " << nn << "  ";
    //         std::cout << std::endl;
    //                 ifprint = true;
    //             }
    //             std::cout << "t: " << ele1.first << "  [";
    //             for(auto ele2 : ele1.second){
    //                 std::cout << "{" << ele2/32 << ", " << ele2%32 << "}, ";
    //             }
    //             std::cout << "]" << std::endl;
    //         }
    //         nn ++;
    //     }
    //     getchar();

    //     }


    // if(Heps_merge > 0.001 || Heps_conflict_prune > 0.001 || Leps_merge > 0.001 || Leps_prune > 0.001){
    // //  exhaust remaing joint path in high-level node, if no confliction, then add to solution set
    // //  remark: this cannot ensure non-dominated set
    //     for(auto iter = std::next(node->joint_path_list.begin()); iter != node->joint_path_list.end(); iter++){
    //         if(DomPrune(hsolution_costs, *iter, Heps_nonconflict_prune) 
    //         || conflict_checker.is_conflict(*iter, node->indiv_paths_list, agent_num)){
    //             continue;
    //         }
    //         std::vector<std::vector<size_t>> new_hsolution;
    //         CostVector  new_cost(node->rep_apex_cost.size(), 0);
    //         for(int i = 0; i < agent_num; i ++){
    //             new_hsolution.push_back(node->indiv_paths_list.at(i)[iter->second.at(i)]);
    //             add_cost(new_cost, node->indiv_real_costs.at(i)[iter->second.at(i)]);
    //         }
    //         hsolution_ids.push_back(new_hsolution);
    //         hsolution_costs.push_back(new_cost);
    //         std::cout << "there is a solution" << std::endl;
    //         extra_solution ++;
    //     }
    // }
        

        auto _t1 = std::chrono::high_resolution_clock::now();
        constraint_num ++;
        if (constraint_num % (500/agent_num) == 0) {
            auto tnow = std::chrono::high_resolution_clock::now(); // for timing.
            auto duration1 = std::chrono::duration_cast<std::chrono::microseconds>(tnow - t0);
            std::cout << "[INFO] * Solver::Search, after " << constraint_num << " conflict splits " 
            << "       time = " << ((double)duration1.count())/1000000.0 << std::endl;
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

                add_constraint(new_node->vertex_constraints, agent_id, std::get<2>(cft).front(), std::get<3>(cft));
                

            //  Low Level Search
                auto _t_0_ = std::chrono::high_resolution_clock::now(); // for timing.
                single_run_map(graph_size, edges, start_end.at(agent_id).first, start_end.at(agent_id).second, 
                    output, algorithm, l_ms, logger, Leps_merge, Leps_prune, time_limit, new_node->indiv_paths_list.at(agent_id), 
                    new_node->indiv_apex_costs.at(agent_id), new_node->indiv_real_costs.at(agent_id), 
                    new_node->vertex_constraints[agent_id], new_node->edge_constraints[agent_id]); 
                auto _t_1_ = std::chrono::high_resolution_clock::now();
                auto duration0__ = std::chrono::duration_cast<std::chrono::microseconds>(_t_1_ - _t_0_);
                LowLevelTime += ((double)duration0__.count())/1000000.0;

                if(new_node->indiv_paths_list.at(agent_id).empty()){
                    continue;
                }


            //  Calculate NonDomSet
                auto __t1 = std::chrono::high_resolution_clock::now();
                if(algorithm == "Apex"){
                    NonDomJointPath(new_node, ms, Heps_merge);
                }else{
                    NonDomJointPath(new_node);
                }
                auto __t2 = std::chrono::high_resolution_clock::now(); // for timing.
                auto duration2_ = std::chrono::duration_cast<std::chrono::microseconds>(__t2 - __t1);
                NonDomTime += ((double)duration2_.count()) / 1000000.0;
                
                new_node->rep_id_list = new_node->joint_path_list.front().second;
                new_node->rep_apex_cost = new_node->joint_path_list.front().first;
                open_list.insert(new_node);
            }

        }else{  //  edge confliction
            for(int i = 0; i < 2; i++){     // 2 agents
                int agent_id = i == 0 ? std::get<0>(cft) : std::get<1>(cft);
                auto new_node = std::make_shared<HighLevelNode>(*node);
                new_node->indiv_paths_list.at(agent_id).clear();
                new_node->indiv_apex_costs.at(agent_id).clear();
                new_node->indiv_real_costs.at(agent_id).clear();

                add_constraint(new_node->edge_constraints, agent_id, std::get<2>(cft).at(i), 
                    std::get<2>(cft).at(1-i), std::get<3>(cft));

                single_run_map(graph_size, edges, start_end.at(agent_id).first, start_end.at(agent_id).second, 
                    output, algorithm, l_ms, logger, Leps_merge, Leps_prune, time_limit, new_node->indiv_paths_list.at(agent_id), 
                    new_node->indiv_apex_costs.at(agent_id), new_node->indiv_real_costs.at(agent_id), 
                    new_node->vertex_constraints[agent_id], new_node->edge_constraints[agent_id]); 

                if(new_node->indiv_paths_list.at(agent_id).empty()){
                    continue;
                }

            //  Calculate NonDomSet
                auto __t1 = std::chrono::high_resolution_clock::now();
                if(algorithm == "Apex"){
                    NonDomJointPath(new_node, ms, Heps_merge);
                }else{
                    NonDomJointPath(new_node);
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
    if(is_success){
        Output << "SUCCESS" << std::endl << std::endl;
    }else{
        Output << "FAIL" << std::endl << std::endl;
    }
    for(size_t num = 0; num < hsolution_ids.size(); num ++){
        Output << "SOLUTION " << num+1;
        Output << " COST   " << "{";
        if(vm["dim"].as<int>() == 2){
            Output << hsolution_costs.at(num).at(0) << ", " << hsolution_costs.at(num).at(1);
        }else{
            Output << hsolution_costs.at(num).at(0) << ", " << hsolution_costs.at(num).at(1) << ", " << hsolution_costs.at(num).at(2);
        }
        Output << "}" << std::endl;
    }
    Output << std::endl;
    Output << "Extra Solution Number = " << extra_solution << std::endl;
    Output << "NonDomTime = " << NonDomTime << std::endl;
    Output << "LowLevelTime = " << LowLevelTime << std::endl;
    Output << "DomPruneTime = " << DomPruneTime << std::endl;
    Output << "ConflictionTime = " << ConflictionTime << std::endl; 
    auto t1 = std::chrono::high_resolution_clock::now(); // for timing.
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(t1 - t0);
    Output << "Total Time = " << ((double)duration.count())/1000000.0 << std::endl;
    Output << "DomPruneNum/NodeExpandNum = " << DomPruneNum << "/" << constraint_num << std::endl;
    Output << std::endl << std::endl;
    return constraint_num;
}





//  BOA* and NAMOA* version
// void Solver::NonDomJointPath(HighLevelNodePtr node, double eps)
// {
//     JointPathPair temp;
//     auto _t1 = std::chrono::high_resolution_clock::now();
//     std::list<JointPathPair>& joint_path_list = node->joint_path_list;
//     joint_path_list.clear();

//     for(size_t id = 0; id < node->indiv_paths_list.at(0).size(); id++){
//         auto element = node->indiv_paths_list.at(0)[id];
//         joint_path_list.push_back(std::make_pair(node->indiv_real_costs.at(0)[id], std::vector<size_t>({id})));
//     }

//     for(int i = 1; i < node->indiv_paths_list.size(); i++){
//         std::vector<JointPathPair> _joint_path_vector;
//         for(auto& joint_path : joint_path_list){
//             for(auto iter = node->indiv_paths_list.at(i).begin(); iter != node->indiv_paths_list.at(i).end(); iter++){
//                 temp = joint_path;
//                 temp.second.push_back(iter->first);
//                 // joint_path.second.push_back(indiv_path.first);
//                 _joint_path_vector.push_back(std::make_pair(add_cost(std::move(temp.first), node->indiv_real_costs.at(i)[iter->first]), std::move(temp.second)));
//                 // std::push_heap(_joint_path_vector.begin(), _joint_path_vector.end(), more_than_);
//             }
//         }
//         std::sort(_joint_path_vector.begin(), _joint_path_vector.end(), less_than_);
//         NonDomVec(joint_path_list, _joint_path_vector, eps);
//     }
//     // for(auto ele : joint_path_list){
//     //     std::cout << ele.first.at(0) << ", " << ele.first.at(1) << std::endl;
//     //     // std::cout << ele.second.at(0) << ele.second.at(1) << ele.second.at(2) << std::endl;
//     // }
//     auto t2 = std::chrono::high_resolution_clock::now(); // for timing.
//     auto duration = std::chrono::duration_cast<std::chrono::microseconds>(t2 - _t1);
//     std::cout << " Non Dom time = " << ((double)duration.count()) / 1000000.0 << std::endl;
// }



// void Solver::NonDomJointPath(HighLevelNodePtr node, double eps)
// {
//     // auto _t1 = std::chrono::high_resolution_clock::now();
//     node->joint_path_list.clear();
    
//     std::vector<JointPathTuple> joint_path_vector;

//     for(size_t id = 0; id < node->indiv_paths_list.at(0).size(); id++){
//         auto element = node->indiv_paths_list.at(0)[id];
//         joint_path_vector.push_back(std::make_tuple(node->indiv_apex_costs.at(0)[id], node->indiv_real_costs.at(0)[id], std::vector<size_t>({id})));
//     }

//     for(int i = 1; i < node->indiv_paths_list.size(); i++){
//         std::vector<JointPathTuple> _joint_path_vector;
//         for(const auto& joint_path : joint_path_vector){
//             for(const auto& indiv_path : node->indiv_paths_list.at(i)){
//                 std::vector<size_t> temp(std::get<2>(joint_path));
//                 temp.push_back(indiv_path.first);
//                 _joint_path_vector.push_back(std::make_tuple(add_cost(std::get<0>(joint_path), node->indiv_apex_costs.at(i)[indiv_path.first]), add_cost(std::get<1>(joint_path), node->indiv_real_costs.at(i)[indiv_path.first]), temp));
                
//             }
//         }
//         std::sort(_joint_path_vector.begin(), _joint_path_vector.end(), less_than_);
//         joint_path_vector = NonDomVec(std::move(_joint_path_vector), ms, eps);
//     }

//     while(!joint_path_vector.empty()){
//         node->joint_path_list.push_back(std::make_pair(std::get<0>(joint_path_vector.front()), std::get<2>(joint_path_vector.front())));
//         std::pop_heap(joint_path_vector.begin(), joint_path_vector.end(), more_than);
//         joint_path_vector.pop_back();
//     }
//     // auto t2 = std::chrono::high_resolution_clock::now(); // for timing.
//     // auto duration = std::chrono::duration_cast<std::chrono::microseconds>(t2 - _t1);
//     // std::cout << " Non Dom time = " << ((double)duration.count()) / 1000000.0 << std::endl;
// }

// // no merge version
// std::vector<JointPathTuple> Solver::NonDomVec(std::vector<JointPathTuple>  joint_path_vector)
// {
//     for(auto iter : joint_path_vector){
//         std::cout << std::get<0>(iter).at(0) << ", " << std::get<0>(iter).at(1) << std::endl;
//     }
//     getchar();
//     std::vector<JointPathTuple>     NonDom_joint_path;
//     if(std::get<0>(joint_path_vector.front()).size() == 2){
//         size_t g_min = UINT64_MAX;
//         while(!joint_path_vector.empty()){
//             if(std::get<0>(joint_path_vector.front()).at(1) < g_min){
//                 g_min = std::get<0>(joint_path_vector.front()).at(1);
//                 NonDom_joint_path.push_back(joint_path_vector.front());
//             }
//             std::pop_heap(joint_path_vector.begin(), joint_path_vector.end(), more_than);
//             joint_path_vector.pop_back();
//         }
//     }
//     for(auto iter : NonDom_joint_path){
//         std::cout << std::get<0>(iter).at(0) << ", " << std::get<0>(iter).at(1) << std::endl;
//     }
//     getchar();
//     return NonDom_joint_path;
// }

// merge version


// void Solver::search(size_t graph_size, std::vector<Edge>& edges, boost::program_options::variables_map& vm, size_t agent_num, double eps, MergeStrategy& ms, LoggerPtr& logger, HSolutionID& hsolution_ids, std::vector<CostVector>& hsolution_costs)
// {
//     HLQueue open_list;
//     // size_t agent_num;

//     ConflictChecker conflict_checker;

// //  initialize open_list
//     HighLevelNodePtr root_node = std::make_shared<HighLevelNode>(agent_num);
//     for(size_t i = 0; i < agent_num; i ++){
//         single_run_map(graph_size, edges, vm["start"].as<int>(), vm["goal"].as<int>(), vm["output"].as<std::string>(), 
//             vm["algorithm"].as<std::string>(), ms, logger, vm["eps"].as<double>(), vm["cutoffTime"].as<int>(), 
//             root_node->indiv_paths_list.at(i), root_node->indiv_apex_costs.at(i), root_node->indiv_real_costs.at(i), 
//             root_node->constraints.at(i));
//     }
//     NonDomJointPath(root_node);
//     std::cout << std::endl << std::endl << "NonDomJointPath Finished" << std::endl << std::endl;
//     root_node->rep_id_list = root_node->joint_path_list.front().second;
//     root_node->rep_apex_cost = root_node->joint_path_list.front().first;
//     open_list.insert(root_node);

// //  main loop    
//     while(!open_list.empty())
//     {
//         auto node = open_list.pop();
//         bool is_filtered = DomPrune(hsolution_costs, node->joint_path_list, eps);
//         if(node->joint_path_list.empty()){
//             continue;
//         }
//         if(is_filtered){
//             node->rep_id_list = node->joint_path_list.front().second;
//             node->rep_apex_cost = node->joint_path_list.front().first;
//             open_list.insert(node);
//             continue;
//         }
//         auto cft = conflict_checker.is_conflict(node);
//         if(cft.empty()){
//             std::vector<std::vector<size_t>> new_hsolution;
//             CostVector  new_cost(node->rep_apex_cost.size(), 0);
//             for(int i = 0; i < agent_num; i ++){
//                 new_hsolution.push_back(node->indiv_paths_list.at(i)[node->rep_id_list.at(i)]);
//                 new_cost = add_cost(new_cost, node->indiv_real_costs.at(i)[node->rep_id_list.at(i)]);
//             }
//             hsolution_ids.push_back(new_hsolution);
//             hsolution_costs.push_back(new_cost);

//             node->joint_path_list.pop_front();
//             if(!node->joint_path_list.empty()){
//                 node->rep_id_list = node->joint_path_list.front().second;
//                 node->rep_apex_cost = node->joint_path_list.front().first;
//                 open_list.insert(node);
//             }
//             continue;
//         }
//     //  generate branch
//         for(int i = 0; i < 2; i ++){
//             auto new_node = std::make_shared<HighLevelNode>(*node);
//             add_constraint(new_node->constraints, cft[i], cft[2], cft[3]);
//             single_run_map(graph_size, edges, vm["start"].as<int>(), vm["goal"].as<int>(), vm["output"].as<std::string>(), 
//                     vm["algorithm"].as<std::string>(), ms, logger, vm["eps"].as<double>(), vm["cutoffTime"].as<int>(), 
//                     new_node->indiv_paths_list.at(cft[i]), new_node->indiv_apex_costs.at(cft[i]), new_node->indiv_real_costs.at(cft[i]), 
//                     new_node->constraints[cft[i]]);
//             NonDomJointPath(new_node);
//             new_node->rep_id_list = root_node->joint_path_list.front().second;
//             new_node->rep_apex_cost = root_node->joint_path_list.front().first;
//             open_list.insert(new_node);
//         }
//     }
// }




