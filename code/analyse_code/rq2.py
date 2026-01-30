import yaml
from pathlib import Path

class LiteralString(str):
    """Custom string class for YAML block scalar representation"""
    pass

def literal_presenter(dumper, data):
    """Presenter for multi-line strings using | style"""
    return dumper.represent_scalar('tag:yaml.org,2002:str', data, style='|')

yaml.add_representer(LiteralString, literal_presenter)

def process_yaml_files():
    rosdeval_dir = Path("/home/zyx/republication/rosdeval")
    deepseek_dir = Path("/home/zyx/republication/generated_repos/gpt")
    output_dir = Path("/home/zyx/republication/experiment_result/rq2_result/gpt")

    output_dir.mkdir(parents=True, exist_ok=True)

    for yaml_file in sorted(rosdeval_dir.glob("*.yaml")):
        with open(yaml_file, 'r') as f:
            data = yaml.safe_load(f)

        method_name = data.get('method_name', '')
        function_dir_name = method_name.replace('::', '__')

        v1_dir = deepseek_dir / function_dir_name / f"generated_{function_dir_name}_v1"

        if not v1_dir.exists():
            print(f"Warning: {v1_dir} does not exist, skipping {yaml_file.name}")
            continue

        result_data = {}

        for file_name in ['generate_code.cpp', 'ground_truth.cpp', 'compilation.log']:
            file_path = v1_dir / file_name
            if file_path.exists():
                with open(file_path, 'r') as f:
                    result_data[file_name] = LiteralString(f.read())
            else:
                result_data[file_name] = LiteralString("")

        output_file = output_dir / yaml_file.name
        with open(output_file, 'w') as f:
            yaml.dump(result_data, f, default_flow_style=False, allow_unicode=True)

        # print(f"Processed {yaml_file.name} -> {output_file}")

def process_label():
    output_dir = Path("/home/zyx/republication/experiment_result/rq2_result/deepseek")

    architecture_codes = {}
    functionality_codes = {}
    total_files = 0
    processed_files = 0

    # Process yaml files numbered 1-240
    for i in range(1, 241):
        yaml_file = output_dir / f"{i}.yaml"

        if not yaml_file.exists():
            continue

        total_files += 1

        try:
            with open(yaml_file, 'r') as f:
                data = yaml.safe_load(f)

            if data is None:
                continue

            processed_files += 1

            # Extract and count architecture-codes
            arch_codes = data.get('architecture-codes', [])
            if arch_codes:
                if isinstance(arch_codes, list):
                    for code in arch_codes:
                        architecture_codes[code] = architecture_codes.get(code, 0) + 1

            # Extract and count functionality-codes
            func_codes = data.get('functionality-codes', [])
            if func_codes:
                if isinstance(func_codes, list):
                    for code in func_codes:
                        functionality_codes[code] = functionality_codes.get(code, 0) + 1

        except yaml.YAMLError as e:
            print(f"Error parsing {yaml_file.name}: {e}")
        except Exception as e:
            print(f"Error processing {yaml_file.name}: {e}")

    # Print statistics
    print(f"\n=== Analysis Results ===")
    print(f"Total yaml files found: {total_files}")
    print(f"Successfully processed: {processed_files}")

    print(f"\n--- Architecture Codes Statistics ---")
    print(f"Total unique codes: {len(architecture_codes)}")
    if architecture_codes:
        sorted_arch = sorted(architecture_codes.items(), key=lambda x: x[1], reverse=True)
        for code, count in sorted_arch:
            print(f"  {code}: {count}")

    print(f"\n--- Functionality Codes Statistics ---")
    print(f"Total unique codes: {len(functionality_codes)}")
    if functionality_codes:
        sorted_func = sorted(functionality_codes.items(), key=lambda x: x[1], reverse=True)
        for code, count in sorted_func:
            print(f"  {code}: {count}")

    # Save statistics to file
    stats_file = output_dir / "statistics.txt"
    with open(stats_file, 'w') as f:
        f.write("=== Analysis Results ===\n")
        f.write(f"Total yaml files found: {total_files}\n")
        f.write(f"Successfully processed: {processed_files}\n\n")

        f.write("--- Architecture Codes Statistics ---\n")
        f.write(f"Total unique codes: {len(architecture_codes)}\n")
        if architecture_codes:
            sorted_arch = sorted(architecture_codes.items(), key=lambda x: x[1], reverse=True)
            for code, count in sorted_arch:
                f.write(f"  {code}: {count}\n")

        f.write("\n--- Functionality Codes Statistics ---\n")
        f.write(f"Total unique codes: {len(functionality_codes)}\n")
        if functionality_codes:
            sorted_func = sorted(functionality_codes.items(), key=lambda x: x[1], reverse=True)
            for code, count in sorted_func:
                f.write(f"  {code}: {count}\n")

    print(f"\nStatistics saved to: {stats_file}")

def process_cause():
    output_dir = Path("/home/zyx/republication/experiment_result/rq2_result/deepseek")

    cause_codes = {}
    total_files = 0
    processed_files = 0

    # Process yaml files numbered 1-240
    for i in range(1, 241):
        yaml_file = output_dir / f"{i}.yaml"

        if not yaml_file.exists():
            continue

        total_files += 1

        try:
            with open(yaml_file, 'r') as f:
                data = yaml.safe_load(f)

            if data is None:
                continue

            processed_files += 1

            # Extract and count architecture-codes
            arch_codes = data.get('root-cause-codes', [])
            if arch_codes:
                if isinstance(arch_codes, list):
                    for code in arch_codes:
                        cause_codes[code] = cause_codes.get(code, 0) + 1


        except yaml.YAMLError as e:
            print(f"Error parsing {yaml_file.name}: {e}")
        except Exception as e:
            print(f"Error processing {yaml_file.name}: {e}")

    # Print statistics
    print(f"\n=== Analysis Results ===")
    print(f"Total yaml files found: {total_files}")
    print(f"Successfully processed: {processed_files}")

    print(f"\n--- Cause Codes Statistics ---")
    print(f"Total unique codes: {len(cause_codes)}")
    if cause_codes:
        sorted_arch = sorted(cause_codes.items(), key=lambda x: x[1], reverse=True)
        for code, count in sorted_arch:
            print(f"  {code}: {count}")


    # Save statistics to file
    stats_file = output_dir / "statistics_cause.txt"
    with open(stats_file, 'w') as f:
        f.write("=== Analysis Results ===\n")
        f.write(f"Total yaml files found: {total_files}\n")
        f.write(f"Successfully processed: {processed_files}\n\n")

        f.write("--- Architecture Codes Statistics ---\n")
        f.write(f"Total unique codes: {len(cause_codes)}\n")
        if cause_codes:
            sorted_arch = sorted(cause_codes.items(), key=lambda x: x[1], reverse=True)
            for code, count in sorted_arch:
                f.write(f"  {code}: {count}\n")

    print(f"\nStatistics saved to: {stats_file}")


if __name__ == "__main__":
    # process_yaml_files()
    # process_label()
    process_cause()