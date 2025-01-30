# MAPF Project
This project tests the performance of standard Multi-Agent Pathfinding algorithms. Read [MAPF Report.pdf](https://github.com/ColinMcD6/MAPF_project/blob/master/MAPF%20Project.pdf) for further details and results.
### Running algorithms on test instances
To run prioritized planning algorithm
```python run_experiments.py --instance <instance path> --solver Prioritized```

To run CBS non-disjoint
```python run_experiments.py --instance <instance path> --solver CBS```

To run CBS disjoint
```python run_experiments.py --instance <instance path> --solver CBS --disjoint```

### To Benchmark All Algorithms
```python run_benchmarks --instance "benchmarks/" --solver All --time 60```
