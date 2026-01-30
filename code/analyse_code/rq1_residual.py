import pandas as pd
import numpy as np
import statsmodels.api as sm
from sklearn.linear_model import LinearRegression

def rq1_2_residual_analysis():
    """RQ1.2 残差分析: 检查每个变量的独立贡献"""

    print("=== RQ1.2 残差分析 - 检查变量的独立贡献 ===\n")

    # 读取数据
    df = pd.read_csv('/home/zyx/republication/experiment_result/rq1_result/result_backup.csv')

    # 准备变量
    df['Pass'] = df['pass@1']

    # 解析element列创建特征
    df['Has_Param'] = df['element'].str.contains('has_parameter', case=False, na=False).astype(int)
    df['Has_Topic'] = df['element'].str.contains('has_topic', case=False, na=False).astype(int)
    print(sum(df['Has_Topic']))
    df['Has_Message'] = df['element'].str.contains('has_message', case=False, na=False).astype(int)
    df['Has_Timer'] = df['element'].str.contains('has_timer', case=False, na=False).astype(int)
    df['Has_Logger'] = df['element'].str.contains('has_logger', case=False, na=False).astype(int)
    df['Has_Domain_Library'] = df['element'].str.contains('has_domain_library', case=False, na=False).astype(int)

    # 所有预测变量
    all_predictors = ['Has_Param', 'Has_Topic', 'Has_Message',
                      'Has_Timer', 'Has_Logger', 'Has_Domain_Library']

    df_clean = df[['Pass'] + all_predictors].dropna()

    print(f"样本数: {len(df_clean)}")
    print(f"Pass率: {df_clean['Pass'].mean():.3f}\n")

    # ============ 方法1: 逐个变量的残差分析 ============
    print("=" * 80)
    print("方法1: 逐个变量的残差分析 - 检查每个变量在控制其他变量后的独立贡献")
    print("=" * 80)

    residual_results = []

    for target_var in all_predictors:
        print(f"\n【分析变量: {target_var}】")
        print("-" * 80)

        # 其他变量（除了目标变量）
        other_vars = [v for v in all_predictors if v != target_var]

        # 步骤1: 用其他变量预测Pass，得到残差
        X_other = df_clean[other_vars]
        y = df_clean['Pass']

        X_other_const = sm.add_constant(X_other)
        model_other = sm.OLS(y, X_other_const).fit()
        y_resid = model_other.resid

        # 步骤2: 用其他变量预测目标变量，得到残差
        x_target = df_clean[target_var]
        model_target = sm.OLS(x_target, X_other_const).fit()
        x_target_resid = model_target.resid

        # 步骤3: 用目标变量的残差预测Pass的残差
        # 这给出了目标变量在控制其他变量后的独立贡献
        X_resid = sm.add_constant(pd.DataFrame({'x_resid': x_target_resid}))
        model_resid = sm.OLS(y_resid, X_resid).fit()

        coef = model_resid.params['x_resid']  # 斜率
        p_val = model_resid.pvalues['x_resid']
        r_squared = model_resid.rsquared

        print(f"残差回归系数: {coef:.4f}")
        print(f"P值: {p_val:.4f}")
        print(f"R²: {r_squared:.4f}")

        if p_val < 0.05:
            print(f"✓ 显著 (p < 0.05) - {target_var}在控制其他变量后仍有独立贡献")
        else:
            print(f"✗ 不显著 (p >= 0.05) - {target_var}的影响可能被其他变量解释")

        residual_results.append({
            'Variable': target_var,
            'Coefficient': coef,
            'P_value': p_val,
            'R_squared': r_squared,
            'Significant': '✓' if p_val < 0.05 else '✗'
        })

    # ============ 总结 ============
    print("\n" + "=" * 80)
    print("总结: 所有变量的独立贡献")
    print("=" * 80)

    results_df = pd.DataFrame(residual_results).sort_values('P_value')

    print(f"\n{'变量':<20} {'残差系数':>12} {'P值':>10} {'R²':>10} {'显著':>5}")
    print("-" * 80)
    for _, row in results_df.iterrows():
        print(f"{row['Variable']:<20} {row['Coefficient']:>12.4f} {row['P_value']:>10.4f} {row['R_squared']:>10.4f} {row['Significant']:>5}")

    significant_vars = results_df[results_df['P_value'] < 0.05]
    if len(significant_vars) > 0:
        print(f"\n✓ 有 {len(significant_vars)} 个变量在控制其他变量后仍有显著的独立贡献:")
        for _, row in significant_vars.iterrows():
            direction = "正向" if row['Coefficient'] > 0 else "负向"
            print(f"  - {row['Variable']}: {direction}贡献 (系数={row['Coefficient']:.4f}, P={row['P_value']:.4f})")
    else:
        print("\n✗ 没有变量在控制其他变量后有显著的独立贡献")

    # ============ 方法2: 增量R²分析 ============
    print("\n" + "=" * 80)
    print("方法2: 增量R²分析 - 每个变量对模型解释力的增量贡献")
    print("=" * 80)

    # 基础模型（不含任何变量）
    X_base = sm.add_constant(np.ones(len(df_clean)))
    y = df_clean['Pass']
    model_base = sm.OLS(y, X_base).fit()
    r2_base = model_base.rsquared

    print(f"\n基础模型 R²: {r2_base:.4f}")

    incremental_results = []

    for target_var in all_predictors:
        # 不含目标变量的模型
        other_vars = [v for v in all_predictors if v != target_var]
        X_without = sm.add_constant(df_clean[other_vars])
        model_without = sm.OLS(y, X_without).fit()
        r2_without = model_without.rsquared

        # 含目标变量的完整模型
        X_with = sm.add_constant(df_clean[all_predictors])
        model_with = sm.OLS(y, X_with).fit()
        r2_with = model_with.rsquared

        # 增量R²
        delta_r2 = r2_with - r2_without

        # F检验增量R²的显著性
        n = len(df_clean)
        k_with = len(all_predictors)
        k_without = len(other_vars)

        f_stat = (delta_r2 / (k_with - k_without)) / ((1 - r2_with) / (n - k_with - 1))
        from scipy import stats
        p_val = 1 - stats.f.cdf(f_stat, k_with - k_without, n - k_with - 1)

        incremental_results.append({
            'Variable': target_var,
            'R²_without': r2_without,
            'R²_with': r2_with,
            'Delta_R²': delta_r2,
            'F_statistic': f_stat,
            'P_value': p_val,
            'Significant': '✓' if p_val < 0.05 else '✗'
        })

    incremental_df = pd.DataFrame(incremental_results).sort_values('Delta_R²', ascending=False)

    print(f"\n{'变量':<20} {'增量R²':>12} {'F统计量':>12} {'P值':>10} {'显著':>5}")
    print("-" * 80)
    for _, row in incremental_df.iterrows():
        print(f"{row['Variable']:<20} {row['Delta_R²']:>12.4f} {row['F_statistic']:>12.2f} {row['P_value']:>10.4f} {row['Significant']:>5}")

    print(f"\n完整模型 R²: {r2_with:.4f}")

    significant_incremental = incremental_df[incremental_df['P_value'] < 0.05]
    if len(significant_incremental) > 0:
        print(f"\n✓ 有 {len(significant_incremental)} 个变量有显著的增量贡献:")
        for _, row in significant_incremental.iterrows():
            pct = row['Delta_R²'] / r2_with * 100
            print(f"  - {row['Variable']}: 增量R²={row['Delta_R²']:.4f} (占总R²的{pct:.1f}%)")

    # ============ 方法3: 偏相关分析 ============
    print("\n" + "=" * 80)
    print("方法3: 偏相关分析 - 控制其他变量后与Pass的相关性")
    print("=" * 80)

    partial_corr_results = []

    for target_var in all_predictors:
        other_vars = [v for v in all_predictors if v != target_var]

        # 用其他变量预测Pass
        X_other = sm.add_constant(df_clean[other_vars])
        y = df_clean['Pass']
        model_y = sm.OLS(y, X_other).fit()
        y_resid = model_y.resid

        # 用其他变量预测目标变量
        x_target = df_clean[target_var]
        model_x = sm.OLS(x_target, X_other).fit()
        x_resid = model_x.resid

        # 计算残差之间的相关系数（偏相关）
        partial_corr = np.corrcoef(x_resid, y_resid)[0, 1]

        # 计算偏相关的显著性
        n = len(df_clean)
        k = len(other_vars)
        t_stat = partial_corr * np.sqrt((n - k - 2) / (1 - partial_corr**2))
        p_val = 2 * (1 - stats.t.cdf(abs(t_stat), n - k - 2))

        partial_corr_results.append({
            'Variable': target_var,
            'Partial_Correlation': partial_corr,
            'T_statistic': t_stat,
            'P_value': p_val,
            'Significant': '✓' if p_val < 0.05 else '✗'
        })

    partial_corr_df = pd.DataFrame(partial_corr_results).sort_values('P_value')

    print(f"\n{'变量':<20} {'偏相关系数':>15} {'T统计量':>12} {'P值':>10} {'显著':>5}")
    print("-" * 80)
    for _, row in partial_corr_df.iterrows():
        print(f"{row['Variable']:<20} {row['Partial_Correlation']:>15.4f} {row['T_statistic']:>12.2f} {row['P_value']:>10.4f} {row['Significant']:>5}")

    significant_partial = partial_corr_df[partial_corr_df['P_value'] < 0.05]
    if len(significant_partial) > 0:
        print(f"\n✓ 有 {len(significant_partial)} 个变量与Pass有显著的偏相关:")
        for _, row in significant_partial.iterrows():
            direction = "正相关" if row['Partial_Correlation'] > 0 else "负相关"
            print(f"  - {row['Variable']}: {direction} (r={row['Partial_Correlation']:.4f}, P={row['P_value']:.4f})")

    return {
        'residual_analysis': results_df,
        'incremental_r2': incremental_df,
        'partial_correlation': partial_corr_df
    }


if __name__ == "__main__":
    results = rq1_2_residual_analysis()
