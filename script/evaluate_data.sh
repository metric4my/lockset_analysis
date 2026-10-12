#!/bin/sh

echo "Running the tool on Post dataset..."

python3 run_post_tests.py > ignoreout.txt

echo "Running the tool on Proj dataset..."

python3 run_proj_tests.py >> ignoreout.txt
python3 run_python_files.py >> ignoreout.txt

echo "Computing count of inconsistencies...\n"
python3 compute_count.py

