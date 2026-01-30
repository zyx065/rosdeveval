# ROS Code Generation Evaluation Research

This repository contains an empirical study on Large Language Models (LLMs) for ROS (Robot Operating System) code generation tasks. The research evaluates three mainstream LLMs (DeepSeek, GPT-4o, and Claude) on generating ROS 2 C++ code.

## Project Overview

This research aims to:
- Evaluate LLM capabilities in generating real-world ROS code
- Analyze the impact of ROS architectural elements on code generation success rates
- Identify and classify root causes of llm-generated failures
- Provide insights for improving LLM performance on domain-specific code generation

### Key Features

- **Benchmark Dataset**: 240 real-world ROS functions from real-world ROS code
- **Multi-Model Evaluation**: Comparative analysis of DeepSeek, GPT-4o, and Claude
- **Statistical Analysis**: Residual regression and partial correlation analysis to identify independent factors
- **Error Taxonomy**: Detailed classification and root cause analysis of llm-generated code failures

## Repository Structure

```
code/                      # Analysis scripts and evaluation framework
├── analyse_code/          # Research question analysis scripts
│   ├── rq1.py            # RQ1: ROS architectural element impact analysis
│   ├── rq1_residual.py   # RQ1: Residual regression and partial correlation
│   ├── rq2.py            # RQ2: Compilation failure classification and statistics
│   └── data_collect.py   # Data extraction utilities
└── humanevalx/           # Code generation and evaluation framework
    ├── generation.py     # Distributed code generation
    ├── evaluation.py     # Functional correctness evaluation
    └── execution.py      # Sandboxed execution environment
rosdeveval/                  # Benchmark dataset (240 YAML files)
result/         # Experimental results
├── rq1_result/           # RQ1 result data
│   └── result_backup.csv # Main results file
└── label_result/           # RQ2 issues classification results and RQ3 root cause classification results
    ├── deepseek/         # DeepSeek error analysis
```

## Dataset Description

### ROSDevEval Benchmark

The `rosdeveval/` directory contains 240 YAML files, each defining a ROS function specification:

- **Source**: Real World ROS code repository
- **Language**: ROS 2 C++
- **Contents**:
  - Method name and signature
  - Functional requirements and documentation
  - ROS architectural elements (topic, parameter, message, timer, logger, etc.)
  - Test cases

**Example Structure**:
```yaml
repo: autoware
package: kinematic_evaluator
method_name: KinematicEvaluatorNode::KinematicEvaluatorNode
category: ros_architecture
elements:
  - has_domain_library
  - has_topic
  - has_parameter
  - has_message
  - has_timer
  - has_logger
requirements: |
  Constructor implementation...
test_cases: [...]
```

## Usage

### Dataset Usage and Verification

To verify the correctness of LLM-generated code using the benchmark dataset, follow these steps:

#### 1. Download ROS 2 Docker Environment

Pull the official ROS 2 development Docker image:
```bash
docker pull osrf/ros2:devel
```

#### 2. Download Robot Software Projects

Download the corresponding robot software projects for each workspace:

| Workspace | Project | Version/Branch |
|-----------|---------|----------------|
| diagnostics_ws | diagnostics | ros2-humble |
| laser_filter_ws | laser_filters | ros2 |
| autoware.universe | autoware.universe | 0.41.2 |
| mcl_ws | mcl_3dl | master |
| moveit_ws | moveit2 | humble |
| navigation2 | navigation2 | humble |

**Download commands**:
```bash
# diagnostics_ws
git clone -b ros2-humble https://github.com/ros/diagnostics.git diagnostics_ws

# laser_filter_ws
git clone -b ros2 https://github.com/ros-perception/laser_filters.git laser_filter_ws

# autoware.universe
git clone -b 0.41.2 https://github.com/autowarefoundation/autoware.universe.git

# mcl_ws
git clone -b master https://github.com/at-wat/mcl_3dl.git mcl_ws

# moveit_ws
git clone -b humble https://github.com/ros-planning/moveit2.git moveit_ws

# navigation2
git clone -b humble https://github.com/ros-planning/navigation2.git
```

#### 3. Verify Generated Code

Use the command files in each generated repository to verify the correctness of LLM-generated code:

```bash
# Navigate to the generated code directory
cd generated_repos/<model_name>/<function_name>

# Execute the verification command
# The command file contains compilation and testing instructions
bash command.sh
```

The verification process will:
- Compile the generated code within the appropriate ROS 2 workspace
- Run test cases defined in the benchmark dataset
- Generate compilation logs and test results

## Research Questions

### RQ1: How do ROS architectural elements affect code generation success rates?

**Analysis Methods**:
- Compare pass@1 and pass@5 success rates across different models
- Use residual regression to isolate independent effects of each ROS element
- Calculate incremental R² to determine each element's contribution

**Key Findings** (see `experiment_result/rq1_result/result_backup.csv`):
- Evaluated the impact of 8 ROS architectural elements
- Analyzed code complexity metrics (cyclomatic complexity, Halstead volume, SLOC)
- Calculated code similarity metrics (code_sim, code_bleu)

### RQ2: What are the root causes of compilation failures?

**Analysis Methods**:
- Manual annotation and classification of compilation errors
- Distinguish between architecture-specific and general functionality issues
- Identify error patterns across different models

**Error Classification** (see `experiment_result/rq2_result/*/statistics.txt`):

**Architecture-Related Errors** (ROS-specific):
- ROS message member access errors
- Non-existent parameter usage
- Missing logger
- Topic subscription/publication issues

**Functionality Errors** (General programming):
- Missing multiple implementation steps
- Condition check logic errors
- Domain library usage errors
- Variable declaration issues

## Usage

### Environment Setup

1. Create Python virtual environment:
```bash
python3 -m venv rostm
source rostm/bin/activate
```

2. Install dependencies:
```bash
pip install pyyaml pandas numpy statsmodels torch
```

### Running Analysis

**RQ1 Analysis**:
```bash
# Basic analysis
python code/analyse_code/rq1.py

# Residual regression analysis
python code/analyse_code/rq1_residual.py
```

**RQ2 Analysis**:
```bash
python code/analyse_code/rq2.py
```

**Data Collection**:
```bash
python code/analyse_code/data_collect.py
```

### Code Generation and Evaluation

**Generate Code**:
```bash
python code/humanevalx/generation.py \
  --model <model_name> \
  --dataset rosdeval \
  --output generated_repos/<model_name>
```

**Evaluate Generated Code**:
```bash
python code/humanevalx/evaluation.py \
  --input generated_repos/<model_name> \
  --dataset rosdeval
```

## Experimental Results

### Main Result Files

- **`experiment_result/rq1_result/result_backup.csv`**: Contains detailed results for all models on 240 functions
  - Success rate metrics: pass@1, pass@5
  - Compilation rate metrics: compile@1, compile@5
  - Code quality metrics: code_sim, code_bleu
  - Complexity metrics: cyclomatic_complexity, halstead_volume, sloc

- **`experiment_result/rq2_result/*/statistics.txt`**: Error statistics summary for each model

### Generated Code

Generation results for each function are stored under `generated_repos/<model>/`:
- `generate_code.cpp`: LLM-generated code
- `ground_truth.cpp`: Reference implementation
- `compilation.log`: Compilation logs

## Technology Stack

- **Python 3.12**: Primary programming language
- **ROS 2**: Target domain framework
- **PyYAML**: Dataset processing
- **Pandas/NumPy**: Data analysis
- **Statsmodels**: Statistical modeling
- **PyTorch**: LLM inference
- **HumanEvalX**: Evaluation framework foundation

## Dependencies

Core dependencies:
- `pyyaml`: YAML file parsing
- `pandas`: Data processing
- `numpy`: Numerical computation
- `statsmodels`: Statistical analysis
- `torch`: Deep learning framework
- `zmq`: Distributed computing

## Research Contributions

1. **ROSDEval Benchmark**: First large-scale evaluation dataset focused on ROS code generation
2. **Multi-dimensional Analysis**: Systematic analysis of how ROS architectural elements affect LLM performance
3. **Error Taxonomy**: Established detailed classification system for ROS code generation failures
4. **Cross-Model Comparison**: Provides comparative analysis of three mainstream LLMs on domain-specific code generation

## License

Please add appropriate license information according to your needs.

## Citation

If you use this dataset or code, please cite our work.

## Contact

For questions or suggestions, please contact us through issues or pull requests.
