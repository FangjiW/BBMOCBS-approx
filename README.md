# BB-MO-CBS-approx

To compile:

```
mkdir build
cd build
cmake ..
make
```
To run
```
cd run
python3 run.py
```
To generate figures on page 8 and 9
```
cd post_process
python3 plot.py
```
To generate the figure on page 1, select the result of any instance and use the following commands:
```
cd post_process
python3 plot_pareto_frontier.py
```