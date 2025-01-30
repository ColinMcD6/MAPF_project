# MAPF Project
### Running algorithms on test instances
To run prioritized planning algorithm
```python run_experiments.py --instance <instance path> --solver Prioritized```

To run CBS non-disjoint
```python run_experiments.py --instance <instance path> --solver CBS```

To run CBS disjoint
```python run_experiments.py --instance <instance path> --solver CBS --disjoint```

### To Benchmark All Algorithms
```python run_benchmarks --instance "benchmarks/" --solver All --time 60```