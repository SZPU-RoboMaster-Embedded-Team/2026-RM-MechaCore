"""
电机功率多项式拟合脚本
公式: P_in = k0 + k1*I + k2*w + k3*I*w + k4*I^2 + k5*w^2

CSV文件格式:
  第一列: P_in (输入功率, W)
  第二列: W (转速, rad/s)
  第三列: I (电流, A)

使用方法:
  python Power.py
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
    # 尝试自动检测是否有表头
    df = pd.read_csv(csv_path)
    
    # 检查第一行是否为数字，如果不是则认为有表头
    try:
        float(df.iloc[0, 0])
    except (ValueError, TypeError):
        df = pd.read_csv(csv_path)
    else:
        df = pd.read_csv(csv_path, header=None)
    
    # 强制转换为数值类型
    P_in = pd.to_numeric(df.iloc[:, 0], errors='coerce').values  # 第一列: P_in (功率, W)
    w = pd.to_numeric(df.iloc[:, 1], errors='coerce').values     # 第二列: W (转速, rad/s)
    I = pd.to_numeric(df.iloc[:, 2], errors='coerce').values     # 第三列: I (电流, A)
    
    # 移除包含NaN的行
    mask = ~(np.isnan(P_in) | np.isnan(I) | np.isnan(w))
    P_in = P_in[mask]
    w = w[mask]
    I = I[mask]
    
    return P_in, w, I


def build_features(I, w):
    """构建特征矩阵 [1, I, w, I*w, I^2, w^2]"""
    n = len(I)
    X = np.column_stack([
        np.ones(n),    # k0: 常数项
        I,             # k1: I
        w,             # k2: w
        I * w,         # k3: I*w
        I ** 2,        # k4: I^2
        w ** 2         # k5: w^2
    ])
    return X


def calculate_power(k, I, w):
    """使用拟合系数计算功率"""
    return k[0] + k[1]*I + k[2]*w + k[3]*I*w + k[4]*I**2 + k[5]*w**2


def fit_model(csv_path):
    """执行多项式拟合"""
    # 加载数据
    P_in, w, I = load_data(csv_path)
    print(f"加载数据点数: {len(P_in)}")
    
    # 构建特征矩阵
    X = build_features(I, w)
    
    # 最小二乘拟合
    model = LinearRegression(fit_intercept=False)
    model.fit(X, P_in)
    
    # 获取系数
    k = model.coef_
    print("\n========== 拟合结果 ==========")
    print(f"k0 (常数项)  = {k[0]:.6f}")
    print(f"k1 (I)       = {k[1]:.6f}")
    print(f"k2 (w)       = {k[2]:.6f}")
    print(f"k3 (I*w)     = {k[3]:.6f}")
    print(f"k4 (I^2)     = {k[4]:.6f}")
    print(f"k5 (w^2)     = {k[5]:.6f}")
    
    # 使用k0-k5计算预测功率
    P_pred = calculate_power(k, I, w)
    
    # 评估精度
    r2 = r2_score(P_in, P_pred)
    rmse = np.sqrt(mean_squared_error(P_in, P_pred))
    
    print("\n========== 拟合精度 ==========")
    print(f"R² 决定系数  = {r2:.6f}")
    print(f"RMSE 均方根误差 = {rmse:.6f}")
    
    # 输出C语言格式
    print("\n========== C语言代码 ==========")
    print(f"#define K0  {k[0]:.6f}f")
    print(f"#define K1  {k[1]:.6f}f")
    print(f"#define K2  {k[2]:.6f}f")
    print(f"#define K3  {k[3]:.6f}f")
    print(f"#define K4  {k[4]:.6f}f")
    print(f"#define K5  {k[5]:.6f}f")
    print("\n// P_in = K0 + K1*I + K2*w + K3*I*w + K4*I*I + K5*w*w")
    
    # 获取保存路径（与CSV文件同目录）
    save_dir = os.path.dirname(os.path.abspath(csv_path))
    save_path = os.path.join(save_dir, 'fitting.png')
    
    # 绘图
    plot_results(P_in, P_pred, k, save_path)
    
    return k


def plot_results(P_in, P_pred, k, save_path):
    """绘制拟合结果 - P_in实际值与拟合值对比"""
    fig, axes = plt.subplots(2, 1, figsize=(12, 8))
    
    # 生成样本索引
    samples = np.arange(len(P_in))
    
    # 图1: P_in 实际值 vs 拟合值 曲线对比
    axes[0].plot(samples, P_in, 'b-', linewidth=1.5, label='P_in Actual (实际功率)', alpha=0.8)
    axes[0].plot(samples, P_pred, 'r--', linewidth=1.5, label='P_in Fitted (拟合功率)', alpha=0.8)
    axes[0].set_xlabel('Sample Index (采样点)')
    axes[0].set_ylabel('Power (W)')
    axes[0].set_title('P_in Actual vs Fitted Power Curve\n'
                      f'公式: P = {k[0]:.4f} + {k[1]:.4f}*I + {k[2]:.4f}*w + {k[3]:.4f}*I*w + {k[4]:.4f}*I² + {k[5]:.4f}*w²')
    axes[0].legend(loc='upper right')
    axes[0].grid(True, alpha=0.3)
    
    # 图2: 误差曲线
    error = P_in - P_pred
    axes[1].plot(samples, error, 'g-', linewidth=1, label='Error (误差)', alpha=0.8)
    axes[1].axhline(y=0, color='k', linestyle='--', linewidth=0.5)
    axes[1].fill_between(samples, error, 0, alpha=0.3, color='green')
    axes[1].set_xlabel('Sample Index (采样点)')
    axes[1].set_ylabel('Error (W)')
    axes[1].set_title(f'Fitting Error (拟合误差)  |  RMSE = {np.sqrt(np.mean(error**2)):.4f} W')
    axes[1].legend(loc='upper right')
    axes[1].grid(True, alpha=0.3)
    
    plt.tight_layout()
    plt.savefig(save_path, dpi=150)
    plt.show()
    print(f"\n图表已保存到: {save_path}")


if __name__ == "__main__":
    print("=" * 50)
    print("电机功率多项式拟合工具")
    print("=" * 50)
    print("\nCSV文件格式要求:")
    print("  第一列: P_in (输入功率, W)")
    print("  第二列: W (转速, rad/s)")
    print("  第三列: I (电流, A)")
    print("=" * 50)
    
    # 交互式输入文件路径
    csv_path = input("\n请输入CSV文件路径 (直接回车尝试寻找默认文件): ").strip()
    
    # 去除可能的引号
    csv_path = csv_path.strip('"').strip("'")
    
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
    
    fit_model(csv_path)
