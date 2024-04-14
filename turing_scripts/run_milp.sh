pip3 install -r requirements.txt
GRB_LICENSE_FILE="$(pwd)/gurobi.lic" PYTHONPATH="$PYTHONPATH:$(pwd)" python3 "$(pwd)/src/mrpcp_2015/modified_mrpcp.py"