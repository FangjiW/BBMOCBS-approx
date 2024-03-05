#include "Utils/Definitions.h"
#include <random>

extern std::unordered_map<size_t, std::vector<int>> id2coord;

// return true if node dom ape x
bool is_dominated_dr(NodePtr apex, NodePtr node){
  for (int i = 1; i < apex->f.size(); i ++ ){
    if (node->f[i] > apex->f[i]){
      return false;
    }
  }
  return true;
}

// (dominatee ,dominator)
bool is_dominated_dr(NodePtr apex, NodePtr node, const EPS eps){
  for (int i = 1; i < apex->f.size(); i ++ ){
    if (node->f[i] > (1 + eps[i]) * apex->f[i]){
      return false;
    }
  }
  return true;
}


bool is_bounded(NodePtr apex, NodePtr node,  const EPS eps){
  for (int i = 0; i < apex->f.size(); i ++ ){
    if (node->f[i] > (1 + eps[i]) * apex->f[i]){
      return false;
    }
  }
  return true;
}

// bool is_bounded_g(NodePtr apex, NodePtr node,  const EPS eps){
//   for (int i = 0; i < apex->f.size(); i ++ ){
//     if (node->g[i] > (1 + eps[i]) * apex->g[i]){
//       return false;
//     }
//   }
//   return true;
// }

double compute_slack(NodePtr apex, NodePtr node,  const EPS eps){
  double min_slack = ( (1 + eps[0]) - (double)node->g[0] / (double) apex->g[0] ) / eps[0];
  for (int i = 1; i < apex->g.size(); i ++ ){

    double slack = ( (1 + eps[i]) - (double)node->g[i] / (double) apex->g[i] ) / eps[i];
    if (slack < min_slack){
      min_slack = slack;
    }
  }
  return min_slack;
}

ApexPathPair::ApexPathPair(const ApexPathPairPtr parent, const Edge&  edge, int turn_mode, int turn_cost): h(parent->h){
  size_t next_id = edge.target;

  std::vector<size_t> new_apex_g(parent->apex->g);
  std::vector<size_t> new_g(parent->path_node->g);

  for (int i = 0; i < new_apex_g.size(); i ++){
    new_apex_g[i] += edge.cost[i];
    new_g[i] += edge.cost[i];
  }

  bool if_turn = turn_mode == -1 ? false : true;

  if(if_turn){
    if(parent->path_node->parent == nullptr){
      if(next_id != parent->id){
        new_g.at(turn_mode) += turn_cost;
        new_apex_g.at(turn_mode) += turn_cost;
      }
    }else{
      int x0 = id2coord[parent->parent->id].at(0);
      int y0 = id2coord[parent->parent->id].at(1);
      int x1 = id2coord[parent->id].at(0);
      int y1 = id2coord[parent->id].at(1);
      int x2 = id2coord[next_id].at(0);
      int y2 = id2coord[next_id].at(1);
      if(x1-x0 != x2-x1 || y1-y0 != y2-y1){
        new_g.at(turn_mode) += turn_cost;
        new_apex_g.at(turn_mode) += turn_cost;
      }
    }
  }

  this->parent = parent->path_node;
  this->id = next_id;
  auto new_h = h(next_id, parent->id, if_turn);
  this->apex = std::make_shared<Node>(next_id, new_apex_g, new_h);
  this->path_node = std::make_shared<Node>(next_id, new_g, new_h, parent->t+1, parent->path_node->conflict_num, parent->path_node);
  this->t = parent->t + 1;
}


bool ApexPathPair::update_nodes_by_merge_if_bounded(const ApexPathPairPtr &other, const std::vector<double> eps, MergeStrategy s){
  // Returns true on sucessful merge and false if it failure
  if (this->id != other->id) {
    return false;
  }

  NodePtr new_apex = std::make_shared<Node>(this->apex->id, this->apex->g, this->apex->h);
  NodePtr new_path_node = nullptr;

  // Merge apex
  for (int i = 0; i < other->apex->g.size(); i ++){
    if (other->apex->g[i] < new_apex->g[i]){
      new_apex->g[i] = other->apex->g[i];
      new_apex->f[i] = other->apex->f[i];
    }
  }

  // choose a path node
  if (s == MergeStrategy::SMALLER_G2 || s == MergeStrategy::SMALLER_G2_FIRST){
    if (other->path_node->g.size() != 2){
      std::cerr << "SMALLER_G2 can only used for bi-objectives";
      exit(-1);
    }
    NodePtr other_path_node;
    if (other->path_node->g[1] == this->path_node->g[1]){
      new_path_node = other->path_node->g[0] < this->path_node->g[0] ? other->path_node: this->path_node;
      other_path_node = other->path_node->g[0] < this->path_node->g[0] ? this->path_node : other->path_node;
    }else{
      new_path_node = other->path_node->g[1] < this->path_node->g[1] ? other->path_node: this->path_node;
      other_path_node = other->path_node->g[1] < this->path_node->g[1] ? this->path_node : other->path_node;
    }
    if (! is_bounded(new_apex, new_path_node, eps)){
      if (s == MergeStrategy::SMALLER_G2_FIRST && is_bounded(new_apex, other_path_node, eps)){
        new_path_node = other_path_node;
      }else{
        return false;
      }
    }
  }else if (s == MergeStrategy::RANDOM){
    if (is_bounded(new_apex, this->path_node, eps)){
      new_path_node = this->path_node;
    }
    if (is_bounded(new_apex, other->path_node, eps)){
      if (new_path_node == nullptr){
        new_path_node = other->path_node;
      }else{
        if (rand() % 2 == 1){
          new_path_node = other->path_node;
        }
      }
    }
    if (new_path_node == nullptr){
      return false;
    }
  }else if (s == MergeStrategy::MORE_SLACK){
    if (is_bounded(new_apex, this->path_node, eps)){
      new_path_node = this->path_node;
    }
    if (is_bounded(new_apex, other->path_node, eps)){
      if (new_path_node == nullptr){
        new_path_node = other->path_node;
      }else if ( compute_slack(new_apex, other->path_node, eps) > compute_slack(new_apex, new_path_node, eps)){
          new_path_node = other->path_node;
      }
    }
    if (new_path_node == nullptr){
      return false;
    }
  }else if (s == MergeStrategy::REVERSE_LEX){
    new_path_node = this->path_node;
    for (int i = 0; i < new_apex->g.size(); i++){
      int i_r = new_apex->g.size() - 1 - i;
      if (this->path_node->g[i_r] != other->path_node->g[i_r]){
        new_path_node = this->path_node->g[i_r] < other->path_node->g[i_r] ?this->path_node: other->path_node;
        break;
      }
    }
    if (! is_bounded(new_apex, new_path_node, eps)){
      return false;
    }
  }else if(s == MergeStrategy::LESS_CONFLICT){
    if(this->path_node->conflict_num < other->path_node->conflict_num){
      if (is_bounded(new_apex, this->path_node, eps)){
        new_path_node = this->path_node;
      }else{
        return false;
      }
    }else if(this->path_node->conflict_num > other->path_node->conflict_num){
      if (is_bounded(new_apex, other->path_node, eps)){
        new_path_node = other->path_node;
      }else{
        return false;
      }
    }else{
      return this->update_nodes_by_merge_if_bounded(other, eps, MergeStrategy::MORE_SLACK);
    }
  }
  else{
    std::cerr << "merge strategy not known" << std::endl;
    exit(-1);
  }

  this->apex = new_apex;
  this->path_node = new_path_node;
  this->parent = new_path_node->parent;
  this->t = new_path_node->t;

  return true;
}

bool ApexPathPair::update_apex_by_merge_if_bounded(const NodePtr &other_apex, const std::vector<double> eps){
  // if (!is_bounded(other_apex, path_node, eps)){
  //   return false;
  // }
  NodePtr new_apex = std::make_shared<Node>(this->apex->id, this->apex->g, this->apex->h);
  bool update_flag = false;
  // Merge apex
  for (int i = 0; i < other_apex->g.size(); i ++){
    if (other_apex->f[i] < new_apex->f[i]){
      new_apex->g[i] = other_apex->f[i];
      new_apex->f[i] = other_apex->f[i];
      if ( path_node->f[i] > (1 + eps[i]) * new_apex->f[i] ){
        return false;
      }

      update_flag = true;
    }
  }
  if (update_flag){
    apex = new_apex;
  }
  return true;
}



bool ApexPathPair::more_than_full_cost::operator()(const ApexPathPairPtr &a, const ApexPathPairPtr &b) const {
  return Node::more_than_full_cost()(a->apex, b->apex);
}


std::ostream& operator<<(std::ostream &stream, const ApexPathPair &ap) {
  // Printed in JSON format
  stream << "{" << *(ap.apex) << ", " << *(ap.path_node) << "}";
  return stream;
}
