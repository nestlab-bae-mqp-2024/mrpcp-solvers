#!/bin/bash
#SBATCH -N 1
#SBATCH -n 48
#SBATCH --mem=16g
#SBATCH -J "MQP_MILP"
#SBATCH -p short
#SBATCH -t 24:00:00
#SBATCH --gres=gpu:0

pip3 install -r requirements.txt
GRB_LICENSE_FILE="$(pwd)/gurobi.lic" PYTHONPATH="$PYTHONPATH:$(pwd)" python3 "$(pwd)/src/mrpcp_2015/modified_mrpcp.py"