#!/bin/bash

for i in {231..231}; do
    yaml_file="/root/rostm/rosdeveval/autoware/${i}.yaml"
    # 在这里添加处理 $yaml_file 的命令
    echo "Processing $yaml_file"

    ros_package_file=$(awk '/ros_package_file:/ {print $2}' "$yaml_file")
    ros_package_name=$(awk '/ros_package:/ {print $2}' "$yaml_file")

    file_path=$(awk '/file_path:/ {print $2}' "$yaml_file")
    sub_name=$(echo "$file_path" | awk -F'/' '{print $(NF-1)}')


    method_name=$(awk '/method_name:/ {print $2}' "$yaml_file")
    test=$(awk '/test:/ {print $2}' "$yaml_file")

    readonly COMPLETE_PATH="/root/repos/diagnostics_ws"
    readonly COMPLETE_REPO_PATH="/root/repos/diagnostics_ws"
    readonly GENERATE_REPO_PATH="/root/generated_repos/diagnostics_ws"

    complete_file="$COMPLETE_REPO_PATH/$ros_package_file/$file_path"
    for param4 in deepseek gpt-4o llama; do
    # for param4 in deepseek llama gpt-4o; do
        # for param5 in advanced_prompt ; do
        for param5 in basic; do
            complete_path="$COMPLETE_REPO_PATH/$ros_package_file/$(dirname "$file_path")"
            generate_path="$GENERATE_REPO_PATH/$ros_package_file/$param4/$param5/$sub_name/$(basename "${2%%.*}")"

            # Skip if generate_path doesn't exist
            if [ ! -d "$generate_path" ]; then
                echo "Skipping: $generate_path does not exist"
                continue
            fi

            # generate_path="$GENERATE_REPO_PATH/$package/$model/$mode/$(basename "${name%%.*}")"
            if sudo cp "$complete_file" "$generate_path"; then
                echo "Success"
            else
                # echo "Failed to copy file to $generate_path" 
                exit 1
            fi

            if find "$generate_path" -type f | grep -q "/$(basename "$file_path")$"; then
            # generate_file=$(find "$generate_path" -type f | grep "/$method_name/.*/$(basename "$file_path")$" | grep -E "v[1-5]")
            generate_file=$(find "$generate_path" -type f | grep "/$method_name/.*/$(basename "$file_path")$" | grep -E "v[1-5]")
                while IFS= read -r file; do
                    # # if [[ "$file" =~ "DefaultPlanner::map_callback" ]]; then
                    cd "$COMPLETE_PATH"
                    cp "$file" "$complete_path/"
                    echo $file
                    echo $complete_path/

                    sudo touch compilation.log
                    sudo chmod -R 777 compilation.log

                    sudo touch test_detail.xml
                    sudo chmod -R 777 test_detail.xml

                    # rm -rf build/ log/ install/
                    # sudo colcon build --packages-select "$ros_package_name"
                    # sudo chmod -R 777 log/
                    # colcon build --packages-select "$ros_package_name"
                    # sudo chmod -R 777 build/
                    # colcon build --packages-select "$ros_package_name"
                    # sudo chmod -R 777 install/
                    colcon build --packages-select "$ros_package_name" > compilation.log 2>&1 
                    colcon build --packages-select "$ros_package_name" > compilation.log 2>&1 

                    echo $ros_package_name $test
                    colcon test --packages-select "$ros_package_name" --ctest-args -R "$test" --output-on-failure > test_detail.xml 2>&1
                    
                    
                    sudo cp compilation.log "$(dirname "$file")"
                    sudo cp test_detail.xml "$(dirname "$file")"
                    sudo cp -r build/"$ros_package_name"/test_results/"$ros_package_name"/*"$test"* "$(dirname "$file")"
                    # sudo cp build/boost_udp_driver/test_results/boost_udp_driver/* "$(dirname "$file")"

                    # sudo rm -rf build/ log/ install/
                    sudo rm -rf compilation.log test_detail.xml
                    # fi
                    sleep 10 

                done <<< "$generate_file"

            fi
        done
    done

    if echo $ros_package_file | grep -q 'dia'; then
        sudo rm -rf /root/repos/diagnostics_ws/
        sudo cp -r /root/diagnostics-ros2-humble/ /root/repos/diagnostics_ws/src/
        echo "copy diagnostics successfully"
    else
        echo $ros_package_file $file_path $param3
    fi
done
