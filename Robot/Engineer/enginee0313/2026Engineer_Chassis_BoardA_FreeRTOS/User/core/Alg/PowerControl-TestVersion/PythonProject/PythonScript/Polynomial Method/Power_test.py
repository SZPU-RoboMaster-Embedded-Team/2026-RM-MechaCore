import numpy as np
import pandas as pd
from sklearn.linear_model import LinearRegression
from sklearn.metrics import r2_score, mean_squared_error
import matplotlib.pyplot as plt
import os

def load_and_preprocess(csv_path, window=30):
    """加载数据并进行预处理：去噪 + 滤波"""
    df = pd.read_csv(csv_path, header=None if pd.read_csv(csv_path, nrows=1).iloc[0,0].dtype != np.float64 else 'infer')
    
    # 提取并转换数据
    P_raw = pd.to_numeric(df.iloc[:, 0], errors='coerce')
    w_raw = pd.to_numeric(df.iloc[:, 1], errors='coerce')
    I_raw = pd.to_numeric(df.iloc[:, 2], errors='coerce')
    
    # 1. 剔除无效行
    valid_mask = ~(P_raw.isna() | w_raw.isna() | I_raw.isna())
    P, w, I = P_raw[valid_mask].values, w_raw[valid_mask].values, I_raw[valid_mask].values

    # 2. 异常值剔除 (简单 Z-score 过滤)
    # 目的：防止传感器瞬时尖峰拉低 R^2
    def remove_outliers(arr, threshold=3):
        mean, std = np.mean(arr), np.std(arr)
        return np.abs(arr - mean) < threshold * std

    mask = remove_outliers(P) & remove_outliers(I)
    P, w, I = P[mask], w[mask], I[mask]

    # 3. 滑动平均滤波 (集成自 PowerFilter.py 的精髓)
    # 使用 pandas 的 rolling 实现，设置 center=True 避免拟合曲线位移
    P_smooth = pd.Series(P).rolling(window=window, center=True, min_periods=1).mean().values
    w_smooth = pd.Series(w).rolling(window=window, center=True, min_periods=1).mean().values
    I_smooth = pd.Series(I).rolling(window=window, center=True, min_periods=1).mean().values

    return (P, w, I), (P_smooth, w_smooth, I_smooth)

def build_features(I, w):
    """构建多项式特征矩阵"""
    return np.column_stack([np.ones(len(I)), I, w, I*w, I**2, w**2])

def fit_and_plot(csv_path):
    # 参数设置
    FILTER_WINDOW = 40  # 滤波窗口，数值越大越平滑，R^2 越高
    
    # 加载数据
    raw, smooth = load_and_preprocess(csv_path, window=FILTER_WINDOW)
    P_raw, w_raw, I_raw = raw
    P_s, w_s, I_s = smooth

    # 使用滤波后的平滑数据进行拟合 (核心优化点)
    X = build_features(I_s, w_s)
    model = LinearRegression(fit_intercept=False)
    model.fit(X, P_s)
    k = model.coef_
    
    # 计算拟合值
    P_pred = X @ k
    
    # 计算评估指标
    r2 = r2_score(P_s, P_pred)
    rmse = np.sqrt(mean_squared_error(P_s, P_pred))

    print(f"\n拟合成功 | R²: {r2:.6f} | RMSE: {rmse:.4f}")
    print(f"系数 k0~k5: {[f'{val:.6f}' for val in k]}")

    # --- 改进的绘图逻辑 ---
    plt.figure(figsize=(12, 7))
    
    # 主图：对比原始、滤波与拟合
    samples = np.arange(len(P_raw))
    plt.scatter(samples, P_raw, s=2, color='gray', alpha=0.3, label='Raw Data (原始噪声)')
    plt.plot(samples, P_s, color='cyan', linewidth=1, label='Filtered Data (平滑采样)')
    plt.plot(samples, P_pred, color='red', linestyle='--', linewidth=2, label='Polynomial Fit (拟合曲线)')
    
    plt.title(f'Motor Power Fitting Optimization\n$R^2 = {r2:.5f}$ | Window Size = {FILTER_WINDOW}', fontsize=14)
    plt.xlabel('Sample Index')
    plt.ylabel('Input Power (W)')
    plt.legend(loc='upper right')
    plt.grid(True, alpha=0.2)
    
    # 输出 C 语言格式代码
    print("\n// Copy to your C code:")
    names = ['K0','K1','K2','K3','K4','K5']
    for n, v in zip(names, k):
        print(f"#define {n} {v:.6f}f")

    plt.tight_layout()
    plt.show()

if __name__ == "__main__":
    path = input("请输入CSV文件路径: ").strip().strip('"')
    if os.path.exists(path):
        fit_and_plot(path)
    else:
        print("路径无效！")