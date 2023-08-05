# A* pex 算法学习

## 类定义

### Definitions.h

- Edge类
- AdjacencyMatrix类
  std::vector<std::vector<Edge>> matrix;    // 以每个source为一组存储边
  size_t graph_size;    // state个数

## 语法

- using = typedef
- std::function 在functional库中
  eg:
  int a(double x){return 1;}
  function<int(double)> f = a;
  cout << f(2.2)     // 输出1
  或using F = function<int(double)>;
  F f = a;

- struct在栈上进行操作，class在堆上，struct比class更加省时
- assert(表达式); 在表达式为假时报错aborted
- 哈希表：查找更快的字典。std::unordered_map

## Questions

- 没懂Kung的On finding the maxima of a set of vectors中d=3如何找NonDomVec
- 空间复杂度是否考虑？论文上只记录cost，如何找到和cost相对应的路径？
- truncate的不比较第一个，只比较后边吗？

## 操作

- g2_min可以直接设置成最后一个添加的g2/(1+eps)
  答：不可以，因为如果待添加的node被上一个添加的node的representative path dominated了，而该node又被另一node dominated，那么第一个被过滤的path不一定被最后一个node dominated。
  
  因此同样的，需要把Apex的solution dominance check给改成apex之间的dominate，否则会与high level search构成连环dominate
