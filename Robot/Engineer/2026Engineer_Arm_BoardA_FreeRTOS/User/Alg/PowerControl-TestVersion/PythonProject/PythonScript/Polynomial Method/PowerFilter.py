"""
GM6020 电机功率多项式拟合脚本（带滤波）
公式: P_in = k0 + k1*I + k2*w + k3*I*w + k4*I^2 + k5*w^2

CSV文件格式:
  第一列: P_in (输入功率, W)
  第二列: W (转速, rad/s)
  第三列: I (电流, A)

特性:
  - 自动滤波去噪
  - 可选降采样
  - 针对6020直驱电机优化
"""

import numpy as np
import pandas as pd
from sklearn.linear_model import LinearRegression
from sklearn.metrics import r2_score, mean_squared_error
import matplotlib.pyplot as plt
import sys
import os


def load_data(csv_path):
    """加载CSV数据"""
    df = pd.read_csv(csv_path)
    
    try:
        float(df.iloc[0, 0])
    except (ValueError, TypeError):
        df = pd.read_csv(csv_path)
    else:
        df = pd.read_csv(csv_path, header=None)
    
    P_in = pd.to_numeric(df.iloc[:, 0], errors='coerce').values
    w = pd.to_numeric(df.iloc[:, 1], errors='coerce').values
    I = pd.to_numeric(df.iloc[:, 2], errors='coerce').values
    
    mask = ~(np.isnan(P_in) | np.isnan(I) | np.isnan(w))
    P_in = P_in[mask]
    w = w[mask]
    I = I[mask]
    
    return P_in, w, I


def filter_data(P_in, w, I, window=50):
    """滑动平均滤波"""
    P_in_filtered = pd.Series(P_in).rolling(window, center=True, min_periods=1).mean().values
    w_filtered = pd.Series(w).rolling(window, center=True, min_periods=1).mean().values
    I_filtered = pd.Series(I).rolling(window, center=True, min_periods=1).mean().values
    
    # 去除边缘NaN
    mask = ~(np.isnan(P_in_filtered) | np.isnan(w_filtered) | np.isnan(I_filtered))
    return P_in_filtered[mask], w_filtered[mask], I_filtered[mask]


def downsample_data(P_in, w, I, step=10):
    """降采样"""
    return P_in[::step], w[::step], I[::step]


def build_features(I, w):
    """构建特征矩阵"""
    n = len(I)
    X = np.column_stack([
        np.ones(n),
        I,
        w,
        I * w,
        I ** 2,
        w ** 2
    ])
    return X


def calculate_power(k, I, w):
    """使用拟合系数计算功率"""
    return k[0] + k[1]*I + k[2]*w + k[3]*I*w + k[4]*I**2 + k[5]*w**2


def fit_model(csv_path, filter_window=50, downsample_step=1):
    """执行多项式拟合"""
    # 加载原始数据
    P_in_raw, w_raw, I_raw = load_data(csv_path)
    print(f"原始数据点数: {len(P_in_raw)}")
    
    # 滤波
    if filter_window > 1:
        P_in, w, I = filter_data(P_in_raw, w_raw, I_raw, filter_window)
        print(f"滤波后数据点数: {len(P_in)} (窗口={filter_window})")
    else:
        P_in, w, I = P_in_raw, w_raw, I_raw
    
    # 降采样
    if downsample_step > 1:
        P_in, w, I = downsample_data(P_in, w, I, downsample_step)
        print(f"降采样后数据点数: {len(P_in)} (步长={downsample_step})")
    
    # 构建特征矩阵
    X = build_features(I, w)
    
    # 最小二乘拟合
    model = LinearRegression(fit_intercept=False)
    model.fit(X, P_in)
    
    # 获取系数
    k = model.coef_
    print("\n========== 拟合结果 (GM6020) ==========")
    print(f"k0 (常数项)  = {k[0]:.6f}")
    print(f"k1 (I)       = {k[1]:.6f}")
    print(f"k2 (w)       = {k[2]:.6f}")
    print(f"k3 (I*w)     = {k[3]:.6f}")
    print(f"k4 (I^2)     = {k[4]:.6f}")
    print(f"k5 (w^2)     = {k[5]:.6f}")
    
    # 计算预测功率
    P_pred = calculate_power(k, I, w)
    
    # 评估精度
    r2 = r2_score(P_in, P_pred)
    rmse = np.sqrt(mean_squared_error(P_in, P_pred))
    
    print("\n========== 拟合精度 ==========")
    print(f"R² 决定系数  = {r2:.6f}")
    print(f"RMSE 均方根误差 = {rmse:.6f}")
    
    # 输出C语言格式
    print("\n========== C语言代码 ==========")
    print(f"#define K0_6020  {k[0]:.6f}f")
    print(f"#define K1_6020  {k[1]:.6f}f")
    print(f"#define K2_6020  {k[2]:.6f}f")
    print(f"#define K3_6020  {k[3]:.6f}f")
    print(f"#define K4_6020  {k[4]:.6f}f")
    print(f"#define K5_6020  {k[5]:.6f}f")
    print("\n// P_in = K0 + K1*I + K2*w + K3*I*w + K4*I*I + K5*w*w")
    
    # 保存路径
    save_dir = os.path.dirname(os.path.abspath(csv_path))
    save_path = os.path.join(save_dir, 'fitting_6020.png')
    
    # 绘图（对比原始和滤波后）
    plot_results(P_in_raw, P_in, P_pred, k, save_path, filter_window)
    
    return k


def plot_results(P_in_raw, P_in_filtered, P_pred, k, save_path, filter_window):
    """绘制拟合结果"""
    fig, axes = plt.subplots(3, 1, figsize=(14, 10))
    
    # 图1: 原始数据 vs 滤波后数据
    samples_raw = np.arange(len(P_in_raw))
    axes[0].plot(samples_raw, P_in_raw, 'b-', linewidth=0.5, alpha=0.5, label='Raw Data')
    axes[0].set_xlabel('Sample Index')
    axes[0].set_ylabel('Power (W)')
    axes[0].set_title(f'GM6020 Raw Power Data (before filtering)')
    axes[0].legend()
    axes[0].grid(True, alpha=0.3)
    
    # 图2: 滤波后 vs 拟合
    samples = np.arange(len(P_in_filtered))
    axes[1].plot(samples, P_in_filtered, 'b-', linewidth=1.5, label='Filtered Data', alpha=0.8)
    axes[1].plot(samples, P_pred, 'r--', linewidth=1.5, label='Fitted', alpha=0.8)
    axes[1].set_xlabel('Sample Index')
    axes[1].set_ylabel('Power (W)')
    axes[1].set_title(f'GM6020 Filtered vs Fitted (window={filter_window})\n'
                      f'P = {k[0]:.3f} + {k[1]:.3f}*I + {k[2]:.3f}*w + {k[3]:.3f}*I*w + {k[4]:.3f}*I² + {k[5]:.3f}*w²')
    axes[1].legend()
    axes[1].grid(True, alpha=0.3)
    
    # 图3: 误差
    error = P_in_filtered - P_pred
    axes[2].plot(samples, error, 'g-', linewidth=1, alpha=0.8)
    axes[2].axhline(y=0, color='k', linestyle='--', linewidth=0.5)
    axes[2].fill_between(samples, error, 0, alpha=0.3, color='green')
    axes[2].set_xlabel('Sample Index')
    axes[2].set_ylabel('Error (W)')
    axes[2].set_title(f'Fitting Error | RMSE = {np.sqrt(np.mean(error**2)):.4f} W')
    axes[2].grid(True, alpha=0.3)
    
    plt.tight_layout()
    plt.savefig(save_path, dpi=150)
    plt.show()
    print(f"\n图表已保存到: {save_path}")


if __name__ == "__main__":
    print("=" * 50)
    print("GM6020 电机功率拟合工具（带滤波）")
    print("=" * 50)
    print("\nCSV文件格式:")
    print("  第一列: P_in (功率, W)")
    print("  第二列: W (转速, rad/s)")
    print("  第三列: I (电流, A)")
    print("=" * 50)
    
    csv_path = input("\n请输入CSV文件路径 (直接回车尝试寻找默认文件): ").strip().strip('"').strip("'")
    
    # 如果用户没有输入路径，尝试寻找默认文件
    if not csv_path:
        # 获取脚本所在目录的上上级目录作为项目根目录
        current_dir = os.path.dirname(os.path.abspath(__file__))
        project_root = os.path.dirname(os.path.dirname(current_dir))
        
        # 默认数据路径
        default_csv = os.path.join(project_root, "data", "power_data.csv")
        
        if os.path.exists(default_csv):
            print(f"使用默认文件: {default_csv}")
            csv_path = default_csv
        else:
            print("错误: 未输入文件路径，且默认文件不存在")
            print(f"默认查找路径: {default_csv}")
            sys.exit(1)
            
    if not os.path.exists(csv_path):
        print(f"错误: 文件不存在 - {csv_path}")
        sys.exit(1)
    
    # 滤波窗口
    window_input = input("滤波窗口大小 (默认50，越大越平滑): ").strip()
    filter_window = int(window_input) if window_input else 50
    
    # 降采样
    step_input = input("降采样步长 (默认1，不降采样): ").strip()
    downsample_step = int(step_input) if step_input else 1
    
    fit_model(csv_path, filter_window, downsample_step)
