import pandas as pd
import numpy as np
from sklearn.linear_model import LinearRegression

# ================= 配置 =================
CSV_FILE_PATH = '3508_test5.csv'
COL_POWER = 'power'
MOTOR_COLS = [
    ('torque_0', 'speed_0'),
    ('torque_1', 'speed_1'),
    ('torque_2', 'speed_2'),
    ('torque_3', 'speed_3')
]


# ========================================

def main():
    try:
        df = pd.read_csv(CSV_FILE_PATH)
        # 清洗列名
        df.columns = [c.strip().replace('`', '') for c in df.columns]
    except Exception as e:
        print(f"读取失败: {e}")
        return

    # 1. 预计算原始特征
    sum_T2 = np.zeros(len(df))
    sum_Tw = np.zeros(len(df))
    sum_w2 = np.zeros(len(df))
    sum_T_abs = np.zeros(len(df))
    sum_w_abs = np.zeros(len(df))

    for t_col, w_col in MOTOR_COLS:
        T = df[t_col].values
        w = df[w_col].values
        sum_T2 += T ** 2
        sum_Tw += T * w
        sum_w2 += w ** 2
        sum_T_abs += np.abs(T)
        sum_w_abs += np.abs(w)

    X_orig = np.column_stack((sum_T2, sum_Tw, sum_w2, sum_T_abs, sum_w_abs))
    Y_orig = df[COL_POWER].values

    # ================= 自动寻找最佳延迟 =================
    best_score = -100
    best_shift = 0
    best_model = None

    # 尝试从 -10 (往后挪) 到 +10 (往前挪) 的偏移量
    # 裁判数据通常滞后，所以一般需要让 Y 往前挪 (负数 shift)
    print("正在自动校准时间延迟...")

    for shift in range(-15, 5):  # 搜索范围：滞后15个采样点 ~ 超前5个采样点
        # 移位操作
        if shift == 0:
            X_shifted = X_orig
            Y_shifted = Y_orig
        elif shift > 0:
            X_shifted = X_orig[:-shift]
            Y_shifted = Y_orig[shift:]
        else:  # shift < 0
            X_shifted = X_orig[-shift:]  # 切掉前面
            Y_shifted = Y_orig[:shift]  # 切掉后面（实际上是把Y往前移）

        if len(Y_shifted) < 100: continue

        model = LinearRegression()
        model.fit(X_shifted, Y_shifted)
        score = model.score(X_shifted, Y_shifted)

        if score > best_score:
            best_score = score
            best_shift = shift
            best_model = model

    # ================= 输出最优结果 =================
    print(f"\n校准完成！最佳时间偏移: {best_shift} 个采样点")
    print(f"校准后 R^2 提升至: {best_score:.4f}")

    # 获取系数
    k4, k3, k5, k1, k2 = best_model.coef_
    intercept_total = best_model.intercept_
    k0_per_motor = intercept_total / 4.0

    print("-" * 40)
    print(f"float k4 = {k4:.8f}f;")
    print(f"float k3 = {k3:.8f}f;")
    print(f"float k5 = {k5:.8f}f;")
    print(f"float k1 = {k1:.8f}f;")
    print(f"float k2 = {k2:.8f}f;")
    print(f"float k0 = {k0_per_motor:.8f}f;")
    print("-" * 40)


if __name__ == '__main__':
    main()