#!/bin/bash

for version in {1..5}; do
    source_dir="$HOME/code/rostm/generated_repos/leetcode/v${version}/3002-maximum-size-of-a-set-after-removals"
    target_dir="$HOME/code/rostm/leetcode/dsa-leetcode/src"
    test_script="$HOME/code/rostm/leetcode/dsa-leetcode/run_tests.sh"
    output_file="$source_dir/test_results_v${version}.txt"
    
    echo "copying v${version}..."
    cp -r "$source_dir" "$target_dir"

    if [ -f "$test_script" ] && [ -x "$test_script" ]; then
        cd "$(dirname "$test_script")" && ./run_tests.sh > "$output_file" 2>&1
    else
        echo "Can not run_test: $test_script"
    fi
    rm -rf "$target_dir"/*3002*
done