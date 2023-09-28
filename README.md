# MILP Introduction

This is a repo to get familiar with MILP formulation using Gurobi's APIs.

## Getting Started

First, you need to have docker and docker compose installed in your system.
Installing Docker Desktop installs all these things.

Then, you need to get a license file from Gurobi. Save this to `docker/jupyter-gurobi/gurobi.lic`

Finally, in the root of the project, run this command to access all notebooks, codes:

```
docker compose up --build jupyter
```

Click on the link with the token in the output to open jupyter.

In `jupyter/` folder, I included examples from Gurobi.

