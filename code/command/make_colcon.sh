#!/bin/bash
# 94,95,98,135
for i in {223..223}; do
    yaml_file="/workspace/rostm/rosdeveval/autoware/${i}.yaml"
    # 在这里添加处理 $yaml_file 的命令
    echo "Processing $yaml_file"

    ros_package_file=$(awk '/ros_package_file:/ {print $2}' "$yaml_file")
    ros_package_name=$(basename "$ros_package_file")

    file_path=$(awk '/file_path:/ {print $2}' "$yaml_file")
    sub_name=$(echo "$file_path" | awk -F'/' '{print $(NF-1)}')


    method_name=$(awk '/method_name:/ {print $2}' "$yaml_file")
    test=$(awk '/test:/ {print $2}' "$yaml_file")

    readonly COMPLETE_REPO_PATH="/workspace/repos/autoware"
    chmod -R 777 $COMPLETE_REPO_PATH
    readonly GENERATE_REPO_PATH="/workspace/generated_repos/autoware"

    complete_file="$COMPLETE_REPO_PATH/$ros_package_file/$file_path"
    for param4 in deepseek llama gpt-4o; do
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
                    echo "$file"
                    echo "$complete_path/"
                    cd "$COMPLETE_REPO_PATH"
                    cp "$file" "$complete_path/"

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

                    echo $ros_package_name $test
                    colcon test --packages-select "$ros_package_name" --ctest-args -R "$test" --output-on-failure > test_detail.xml 2>&1
                    
                    sudo cp compilation.log "$(dirname "$file")"
                    sudo cp test_detail.xml "$(dirname "$file")"
                    sudo cp -r build/"$ros_package_name"/test_results/"$ros_package_name"/*"$test"* "$(dirname "$file")"
                    # sudo cp build/boost_udp_driver/test_results/boost_udp_driver/* "$(dirname "$file")"

                    # sudo rm -rf build/ log/ install/
                    sudo rm -rf compilation.log test_detail.xml
                    # fi

                done <<< "$generate_file"

            fi
        done
    done

    if echo $ros_package_file | grep -q 'autoware.universe'; then
        sudo rm -rf /workspace/repos/autoware/src/universe/autoware.universe
        sudo cp -r /workspace/autoware.universe-0.41.2 /workspace/repos/autoware/src/universe/
        sudo mv /workspace/repos/autoware/src/universe/autoware.universe-0.41.2 /workspace/repos/autoware/src/universe/autoware.universe
        echo "copy autoware.universe successfully"
    elif echo $ros_package_file | grep -q 'autoware.core'; then
        sudo rm -rf /workspace/repos/autoware/src/core/autoware.core
        sudo cp -r /workspace/autoware_core-0.2.0 /workspace/repos/autoware/src/core/
        sudo mv /workspace/repos/autoware/src/core/autoware_core-0.2.0 /workspace/repos/autoware/src/core/autoware.core
        echo "copy autoware.core successfully"
    elif echo $ros_package_file | grep -q 'autoware_lanelet2_extension'; then
        sudo rm -rf /workspace/repos/autoware/src/core/autoware_lanelet2_extension
        sudo cp -r /workspace/autoware_lanelet2_extension-0.6.3 /workspace/repos/autoware/src/core/
        sudo mv /workspace/repos/autoware/src/core/autoware_lanelet2_extension-0.6.3 /workspace/repos/autoware/src/core/autoware_lanelet2_extension
    elif echo $ros_package_file | grep -q 'llh_converter'; then
        sudo rm -rf /workspace/repos/autoware/src/universe/external/llh_converter
        sudo cp -r /workspace/llh_converter-ros2 /workspace/repos/autoware/src/universe/external/
        sudo mv /workspace/repos/autoware/src/universe/external/llh_converter-ros2 /workspace/repos/autoware/src/universe/external/llh_converter
    elif echo $ros_package_file | grep -q 'nebula'; then 
        sudo rm -rf /workspace/repos/autoware/src/sensor_component/external/nebula
        sudo cp -r /workspace/nebula-0.2.4 /workspace/repos/autoware/src/sensor_component/external/
        sudo mv /workspace/repos/autoware/src/sensor_component/external/nebula-0.2.4 /workspace/repos/autoware/src/sensor_component/external/nebula
    elif echo $ros_package_file | grep -q 'transport_drivers'; then
        sudo rm -rf /workspace/repos/autoware/src/sensor_component/transport_drivers
        sudo cp -r /workspace/transport_drivers-main /workspace/repos/autoware/src/sensor_component/
        sudo mv /workspace/repos/autoware/src/sensor_component/transport_drivers/transport_drivers-main /workspace/repos/autoware/src/sensor_component/transport_drivers/transport_drivers
    else
        echo $ros_package_file $file_path $param3
    fi
done
