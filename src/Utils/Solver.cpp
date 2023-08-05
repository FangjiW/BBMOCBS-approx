#include "Utils/Solver.h"
#include <cstdlib>
#include <list>
#include <algorithm>

bool more_than(JointPathTuple a, JointPathTuple b);
bool more_than_(JointPathPair a, JointPathPair b);

void Solver::add_constraint(ConstraintSet& constraints, size_t agent_id, size_t node_id, size_t time)
{
    constraints[agent_id][time].push_back(node_id);
}

bool Solver::DomPrune(std::vector<CostVector> solution_costs, std::list<JointPathPair>& joint_path_list, double eps)
{
    // std::cout << "joint_path_list size = " << joint_path_list.front().first.at(0) << ", " << joint_path_list.front().first.at(1) << std::endl;
    // getchar();
    bool is_prune = false;
    for (auto iter = joint_path_list.begin(); iter != joint_path_list.end(); ) {
        bool is_dominated = false;
        for (auto& solution_cost : solution_costs) {
            bool is_dominated_by_this_one = true;
            for (int i = 1; i < solution_cost.size(); i++) {
                if (iter->first.at(i) * (1 + eps) < solution_cost.at(i)) {
                    is_dominated_by_this_one = false;
                    break;
                }
            }
            if (is_dominated_by_this_one) {
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

// A*pex version
void Solver::NonDomJointPath(HighLevelNodePtr node, MergeStrategy ms, double eps)
{
    JointPathTuple temp;
    // auto _t1 = std::chrono::high_resolution_clock::now();
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
                joint_path_vector.push_back(std::make_tuple(std::move(std::get<0>(temp)), std::move(std::get<1>(temp)), std::move(std::get<2>(temp))));
                // std::push_heap(_joint_path_vector.begin(), _joint_path_vector.end(), more_than);
            }
        }
        joint_path_vector.sort(more_than);
        // std::cout << "im here";
        // std::cout << std::get<0>(joint_path_vector.front()).size();
        
        // getchar();
        NonDomVec(joint_path_vector, ms, eps);
    }

    for(auto& element : joint_path_vector){
        node->joint_path_list.push_back(std::make_pair(std::get<0>(element), std::get<2>(element)));
    }
    // auto t2 = std::chrono::high_resolution_clock::now(); // for timing.
    // auto duration = std::chrono::duration_cast<std::chrono::microseconds>(t2 - _t1);
    // std::cout << " Non Dom time = " << ((double)duration.count()) / 1000000.0 << std::endl;
}

//  BOA* and NAMOA*dr version
void Solver::NonDomJointPath(HighLevelNodePtr node, double eps)
{
    JointPathPair temp;
    // auto _t1 = std::chrono::high_resolution_clock::now();
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
                // std::push_heap(_joint_path_vector.begin(), _joint_path_vector.end(), more_than_);
            }
        }
        joint_path_list.sort(more_than_);
        NonDomVec(joint_path_list, eps);
    }
    // auto t2 = std::chrono::high_resolution_clock::now(); // for timing.
    // auto duration = std::chrono::duration_cast<std::chrono::microseconds>(t2 - _t1);
    // std::cout << " Non Dom time = " << ((double)duration.count()) / 1000000.0 << std::endl;
}


// A*pex version
void Solver::NonDomVec(std::list<JointPathTuple>& joint_path_vector, MergeStrategy ms, double eps)
{
    // std::cout << "aaa";
    // getchar();

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
    } else {
        std::cout << std::endl << "Only g size = 2" << std::endl;
        exit(1);
    }
    // std::cout << "bbb";
    // getchar();
}


//  BOA* and NAMOA* version
void Solver::NonDomVec(std::list<JointPathPair>& joint_path_list, double eps)
{
    if (joint_path_list.front().first.size() == 2) {
        size_t g_min = UINT64_MAX;
        // while (!_joint_path_vector.empty()) {
        //     if ((1+eps)*_joint_path_vector.front().first.at(1) < g_min) {
        //         g_min = (1+eps)*_joint_path_vector.front().first.at(1);
        //         joint_path_vector.push_back(_joint_path_vector.front());
        //     }
        //     std::pop_heap(_joint_path_vector.begin(), _joint_path_vector.end(), more_than_);
        //     _joint_path_vector.pop_back();
        // }
        for(auto iter = joint_path_list.begin(); iter != joint_path_list.end(); ) {
            if ((1+eps)*iter->first.at(1) < g_min) {
                g_min = (1+eps)*iter->first.at(1);
                iter ++;
            }else{
                iter = joint_path_list.erase(iter);
            }
            // std::pop_heap(_joint_path_vector.begin(), _joint_path_vector.end(), more_than_);
            // _joint_path_vector.pop_back();
        }
    } else {
        std::cout << std::endl << "Only g size = 2" << std::endl;
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
    }else{
        return false;
    }
}


size_t Solver::search(size_t graph_size, std::vector<Edge>& edges, boost::program_options::variables_map& vm, 
        std::vector<std::pair<size_t, size_t>> start_end, int agent_num, double eps, MergeStrategy& ms, 
        LoggerPtr& logger, HSolutionID& hsolution_ids, std::vector<CostVector>& hsolution_costs)
{
    auto _t0 = std::chrono::high_resolution_clock::now();    // record time
    HLQueue open_list;
    // size_t agent_num;

    ConflictChecker conflict_checker;

//  initialize open_list
    HighLevelNodePtr root_node = std::make_shared<HighLevelNode>(agent_num);
    for(size_t i = 0; i < agent_num; i ++){
        single_run_map(graph_size, edges, start_end.at(i).first, start_end.at(i).second, vm["output"].as<std::string>(), 
            vm["algorithm"].as<std::string>(), ms, logger, vm["eps"].as<double>(), vm["cutoffTime"].as<int>(), 
            root_node->indiv_paths_list.at(i), root_node->indiv_apex_costs.at(i), root_node->indiv_real_costs.at(i), 
            root_node->constraints.at(i));
    }
    if(vm["algorithm"].as<std::string>() == "Apex"){
        NonDomJointPath(root_node, ms, eps);
    }else{
        NonDomJointPath(root_node, eps);
    }
    // std::cout << "here" << root_node->joint_path_list.size();
    // getchar();
    root_node->rep_id_list = root_node->joint_path_list.front().second;
    root_node->rep_apex_cost = root_node->joint_path_list.front().first;
    // for(auto i : root_node->rep_id_list){
    //     std::cout << i << " ";
    // }
    // getchar();
    open_list.insert(root_node);

//  main loop    
    size_t constraint_num = 0;
    while(!open_list.empty())
    {
        auto node = open_list.pop();
        // std::cout << node->indiv_paths_list.at(0)[0].at(0) << node->indiv_paths_list.at(0)[0].at(1) << std::endl;
        // std::cout << node->indiv_paths_list.at(1)[0].at(0) << node->indiv_paths_list.at(1)[0].at(1) << std::endl;
        
        // getchar();
        // std::cout << hsolution_costs.size();
        // std::cout << open_list.size() << std::endl;
        // getchar();
        bool is_filtered = DomPrune(hsolution_costs, node->joint_path_list, eps);
        // std::cout << "DomPrune finish";
        // getchar();
        if(node->joint_path_list.empty()){
            continue;
        }
        if(is_filtered){
            node->rep_id_list = node->joint_path_list.front().second;
            node->rep_apex_cost = node->joint_path_list.front().first;
            open_list.insert(node);
            continue;
        }
        auto cft = conflict_checker.is_conflict(node);
        // if(cft.empty()){
        //     std::cout << "NO CONFLICTION!" << std::endl;
        // }
        // getchar();
        // else{
        //     std::cout << cft[0] <<"    " << cft[1] << "    " << cft[2] << "    " << cft[3] << std::endl;
        //     getchar();
        // }
        if(cft.empty()){
            std::vector<std::vector<size_t>> new_hsolution;
            CostVector  new_cost(node->rep_apex_cost.size(), 0);
            for(int i = 0; i < agent_num; i ++){
                new_hsolution.push_back(node->indiv_paths_list.at(i)[node->rep_id_list.at(i)]);
                add_cost(new_cost, node->indiv_real_costs.at(i)[node->rep_id_list.at(i)]);
            }
            hsolution_ids.push_back(new_hsolution);
            hsolution_costs.push_back(new_cost);

            node->joint_path_list.pop_front();
            // for(auto ele : node->joint_path_list){
            //     std::cout << ele.first.at(0) << ", " << ele.first.at(1) << std::endl;
            // }
            // getchar();
            if(!node->joint_path_list.empty()){
                node->rep_id_list = node->joint_path_list.front().second;
                node->rep_apex_cost = node->joint_path_list.front().first;
                open_list.insert(node);
            }
            // std::cout << "Add to solution";
            // std::cout << open_list.size();
            // getchar();
            continue;
        }
    // //  print confliction information
    //     int id1 = cft[0] >= 0 ? cft[0] : -cft[0]-1;
    //     int id2 = cft[1] >= 0 ? cft[1] : -cft[1]-1;
    //     std::cout << std::endl << std::endl << "agent1: " << id1 << "  agent2: " << id2 << "  state: {" 
    //         << cft[2]/32 << ", " << cft[2]%32 << "}    t:" << cft[3] << std::endl << "path1: ";
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
    //     for(auto ele : node->constraints){
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

        constraint_num ++;
        if (constraint_num % (500/agent_num) == 0) {
            auto tnow = std::chrono::high_resolution_clock::now(); // for timing.
            auto duration = std::chrono::duration_cast<std::chrono::microseconds>(tnow - _t0);
            std::cout << "[INFO] * Solver::Search, after " << constraint_num << " conflict splits " 
            << "       time = " << ((double)duration.count())/1000000.0 << std::endl;
        }
        for(int i = 0; i < 2; i ++){
            if(cft[i] < 0){
                continue;
            }
            auto new_node = std::make_shared<HighLevelNode>(*node);
            new_node->indiv_paths_list.at(cft[i]).clear();
            new_node->indiv_apex_costs.at(cft[i]).clear();
            new_node->indiv_real_costs.at(cft[i]).clear();
            add_constraint(new_node->constraints, cft[i], cft[2], cft[3]);
            // std::cout << "RUN AGENT " << cft[i] << std::endl;
            // getchar();
            single_run_map(graph_size, edges, start_end.at(cft[i]).first, start_end.at(cft[i]).second, vm["output"].as<std::string>(), 
                    vm["algorithm"].as<std::string>(), ms, logger, vm["eps"].as<double>(), vm["cutoffTime"].as<int>(), 
                    new_node->indiv_paths_list.at(cft[i]), new_node->indiv_apex_costs.at(cft[i]), new_node->indiv_real_costs.at(cft[i]), 
                    new_node->constraints[cft[i]]); 

            // std::cout << new_node->indiv_real_costs.at(cft[i]).size();
            // getchar();

            if(new_node->indiv_paths_list.at(cft[i]).empty()){
                continue;
            }
            if(vm["algorithm"].as<std::string>() == "Apex"){
                NonDomJointPath(new_node, ms, eps);
            }else{
                NonDomJointPath(new_node, eps);
            }

            // std::cout << "here" << new_node->joint_path_list.size();
            // getchar();
            
            new_node->rep_id_list = new_node->joint_path_list.front().second;
            new_node->rep_apex_cost = new_node->joint_path_list.front().first;
            open_list.insert(new_node);
            // std::cout << new_node->indiv_paths_list.at(0)[0].at(0) << new_node->indiv_paths_list.at(0)[0].at(1) << std::endl;
            // std::cout << new_node->indiv_paths_list.at(1)[0].at(0) << new_node->indiv_paths_list.at(1)[0].at(1) << std::endl;
            // getchar();
        }
        // std::cout << "Open List Size:    " << open_list.size() << std::endl;
        // std::cout << "END ONE LOOP" << std::endl;
    }
    return constraint_num;
}


bool more_than(JointPathTuple a, JointPathTuple b)
{
    for(int i = 0; i < std::get<0>(a).size(); i ++){
        if(std::get<0>(a).at(i) != std::get<0>(b).at(i)){
            return std::get<0>(a).at(i) <= std::get<0>(b).at(i);
        }
    }
    return false;
    // for(int i = 0; i < a->apex_cost.size(); i ++){
    //     if(a->apex_cost.at(i) != b->apex_cost.at(i)){
    //         return a->apex_cost.at(i) > b->apex_cost.at(i);
    //     }
    // }
    // return false;
}


bool more_than_(JointPathPair a, JointPathPair b)
{
    for(int i = 0; i < a.first.size(); i ++){
        if(a.first.at(i) != b.first.at(i)){
            return a.first.at(i) <= b.first.at(i);
        }
    }
    return false;
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
//         std::sort(_joint_path_vector.begin(), _joint_path_vector.end(), more_than_);
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
//         std::sort(_joint_path_vector.begin(), _joint_path_vector.end(), more_than_);
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




