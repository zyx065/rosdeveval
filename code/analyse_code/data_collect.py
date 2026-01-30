import yaml
from pathlib import Path

def extract_requirements_from_yaml():
    # 设置目录路径
    directory = Path("/home/zyx/republication/rosdeval")
    
    # 创建输出文件
    output_file = Path("/home/zyx/republication/code/analyse_code/requirements_output.txt")
    
    # 打开文件准备写入
    with open(output_file, 'w', encoding='utf-8') as output:
        # 遍历1到240的YAML文件
        for i in range(1, 241):
            # 构建文件路径
            file_path = directory / f"{i}.yaml"
            
            # 检查文件是否存在
            if not file_path.exists():
                print(f"文件不存在: {file_path}")
                continue
            
            try:
                # 读取YAML文件
                with open(file_path, 'r', encoding='utf-8') as file:
                    data = yaml.safe_load(file)
                
                # 检查category是否为utilities
                if data and 'category' in data and data['category'] == 'utilities':
                    # 输出requirement（如果存在）
                    if 'requirement' in data:
                        requirement = data['method_name']
                        # 写入文件
                        output.write(f"{requirement}\n")
            
            except yaml.YAMLError as e:
                print(f"解析文件 {i}.yaml 时出错: {e}")
            except Exception as e:
                print(f"读取文件 {i}.yaml 时出错: {e}")
    
    print(f"\n所有requirement已保存到: {output_file.absolute()}")

if __name__ == "__main__":
    # 安装必要的库（如果还没有安装）
    try:
        import yaml
    except ImportError:
        print("请先安装PyYAML库: pip install pyyaml")
        exit(1)
    
    extract_requirements_from_yaml()