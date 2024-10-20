# BB-MO-CBS-approx

To compile:
We C++ implemented BB-MO-CBS-pex, BB-MO-CBS-k, BB-MO-CBS-approx , as well as re-implemented BB-MO-CBS. We implemented eager-solution update for BB-MO-CBS-approx, BB-MO-CBS-pex, and BB-MO-CBS-k, and conflict-based merging for BB-MO-CBS-pex and BB-MO-CBS-k.

The low level of BB-MO-CBS and BB-MO-CBS-approx are implemented with BOA* for bi-objective domains and NAMOA*dr for $\geq3$-obejctive domains.

You can compile the project by 

```
mkdir build
cd build
cmake ..
make
```

You can type `bbmocbs_approx --help` to see the expected input arguments.

Example usage:
```
./bin/bbmocbs_approx -m ../example/random-32-32-20.map -n 12 -d 2 -s ../example/random-32-32-20-random.scen -e 0.05 -a BBMOCBS-pex --c1 ../example/random-1.cost --c2 ../example/random-2.cost --CB true --eager true -t 120 -o ../output.txt
```
