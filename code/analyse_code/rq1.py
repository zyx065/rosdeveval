import pandas as pd
from tabulate import tabulate
import statsmodels.api as sm
import numpy as np
from itertools import combinations

def rq1_1():
    test_df = pd.read_csv('/home/zyx/republication/emperiment_result/result_backup.csv')

    def get_filtered_data(data,model,label):
        return data[(data['model'] == model)][label].mean()

    def get_result(data,label):  
        table = {
            "deepseek":[get_filtered_data(data,'deepseek',label)],
            "gpt-4o":[get_filtered_data(data,'gpt-4o',label)],
            "claude":[get_filtered_data(data,'claude',label)]
        } 
        return pd.DataFrame(table)

    test_result = get_result(test_df,'pass@1')
    print(tabulate(test_result,headers='keys',tablefmt='grid',showindex=False))

    compile_result = get_result(test_df,'pass@5')
    print(tabulate(compile_result,headers='keys',tablefmt='grid',showindex=False))

    # compile_result = get_result(test_df,'compile@1')
    # print(tabulate(compile_result,headers='keys',tablefmt='grid',showindex=False))

    # compile_result = get_result(test_df,'compile@5')
    # print(tabulate(compile_result,headers='keys',tablefmt='grid',showindex=False))


def get_filtered_data(data, model, context, label):
    """过滤数据并计算平均值"""
    condition = (data['model'] == model)

    if context == '[]':
        # element 等于 '[]'
        condition = condition & (data['element'] == '[]')
    elif context:
        # element 包含特定特性
        condition = condition & (data['element'].str.contains(context, na=False))

    return data[condition][label].mean()


def get_result(data, label):
    """获取各个特性的结果表"""
    contexts = ["has_domain_library","has_service","has_topic","has_parameter","has_message","has_timer","has_logger","has_diagnostic","[]"]

    table = {
        "context/model": contexts,
        "deepseek": [get_filtered_data(data,'deepseek',c,label) for c in contexts],
        "gpt-4o": [get_filtered_data(data,'gpt-4o',c,label) for c in contexts],
        "claude": [get_filtered_data(data,'claude',c,label) for c in contexts]
    }
    return pd.DataFrame(table)


def get_percentage_reduction(data, label):
    """计算每个特性相比于[]的百分比变化"""
    contexts = ["has_domain_library","has_service","has_topic","has_parameter","has_message","has_timer","has_logger","has_diagnostic"]
    models = ["deepseek", "gpt-4o", "claude"]

    # 获取基准值（element为[]的情况）
    baseline = {}
    for model in models:
        baseline[model] = get_filtered_data(data, model, '[]', label)

    # 计算百分比变化
    table = {"context/model": contexts}

    for model in models:
        reductions = []
        for context in contexts:
            feature_value = get_filtered_data(data, model, context, label)
            baseline_value = baseline[model]

            if baseline_value != 0:
                reduction = ((baseline_value - feature_value) / baseline_value) * 100
            else:
                reduction = 0
            reductions.append(f"{reduction:.2f}%")

        table[model] = reductions

    return pd.DataFrame(table)


def rq1_2():
    test_df = pd.read_csv('/home/zyx/republication/experiment_result/rq1_result/result_backup.csv')

    print("\n=== pass@1 结果 ===")
    test_result = get_result(test_df,'pass@1')
    print(tabulate(test_result,headers='keys',tablefmt='grid',showindex=False))

    print("\n=== pass@1 百分比降低 ===")
    reduction_result = get_percentage_reduction(test_df, 'pass@1')
    print(tabulate(reduction_result,headers='keys',tablefmt='grid',showindex=False))

    print("\n=== pass@5 结果 ===")
    compile_result = get_result(test_df,'pass@5')
    print(tabulate(compile_result,headers='keys',tablefmt='grid',showindex=False))

    print("\n=== pass@5 百分比降低 ===")
    reduction_result = get_percentage_reduction(test_df, 'pass@5')
    print(tabulate(reduction_result,headers='keys',tablefmt='grid',showindex=False))


rq1_2()