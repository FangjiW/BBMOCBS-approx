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
- lambda表达式：直接在该处定义函数
  [捕获列表] (参数列表) -> 返回类型 {
    // 函数体
  }
  eg: int x = 1;
      int result = [&x](int y) { return x * y; }(3);

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

## BB-MO-CBS vs CBS

以下分析有问题：MO-CBS并不能直接计算NonDomJointPath
<!-- 优势：

1. BB-MO-CBS不需要进行更多的low-level search，每次发现conflict之后直接把所有这种conflict直接解决，而CBS每次碰到这种conflict都需要重复进行low-level，解决方法是记录所有的low-level，遇到同样的confict直接替换即可，但很耗费空间
2. BB-MO-CBS每次发现conflict会重新计算NonDomJointPath，因此同一个conflict只会算non-dominated set，而CBS会包含一些dominated joint path

劣势：BB-MO-CBS每次发现conflict都需要重新计算NonDomJointPath，这会消耗不少时间，而CBS只用在root node计算一次，后面替换一个agent的Pateto set即可。(ps: 这真的是BB-MO-CBS一个很大的问题，因为MO-CBS第一次算完之后只留下了精选的一部分，而不需要在之后考虑没有必要的path组合)

但由此受到启发：虽然BB-MO-CBS无法直接留下精选的组合，因为根据BB-MO-CBS的原理NonDomJointPath是一定要计算的 (而且这并不是坏事，理论上要好于MO-CBS在优势1中的解决方案，因为其有优势2，少产生node总是要比少计算NonDomJointPath要重要: 一个是指数，一个是线性)，但是我们可以留下 -->


output.open("../1.out/random-32-32-20/n=" + std::to_string(vm["agent_num"].as<int>()) + "/" + std::to_string(vm["hem"].as<double>()).substr(0, 4)
        + ", " + std::to_string(vm["hep"].as<double>()).substr(0, 4) + ", " + std::to_string(vm["lem"].as<double>()).substr(0, 4) 
        + ", " + std::to_string(vm["lep"].as<double>()).substr(0, 4));