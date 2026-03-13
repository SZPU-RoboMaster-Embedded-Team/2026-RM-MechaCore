import numpy as np
import matplotlib.pyplot as plt
from scipy.optimize import curve_fit
import pandas as pd


# 读取数据
def read_efficiency_data(filename):
    """读取效率曲线数据文件"""
    data = []
    with open(filename, 'r', encoding='utf-8') as f:
        lines = f.readlines()

        # 找到数据开始的位置
        start_reading = False
        for line in lines:
            # 跳过表头
            if line.startswith('----'):
                if start_reading:
                    break  # 遇到第二个分隔线，停止读取
                start_reading = True
                continue
            if line.startswith('峰值效率') or line.startswith('总数据点'):
                break

            if start_reading and line.strip():
                # 解析数据行
                parts = line.split()
                if len(parts) >= 2:
                    try:
                        torque = float(parts[0])
                        efficiency = float(parts[1])
                        data.append((torque, efficiency))
                    except ValueError:
                        continue

    return np.array(data)


# 多项式拟合函数
def polynomial_fit(x, *coeffs):
    """多项式函数"""
    y = 0
    for i, c in enumerate(coeffs):
        y += c * (x ** i)
    return y


def fit_and_save(input_file, output_file, motor_name):
    """对指定数据文件进行8阶拟合并保存结果"""
    import os
    
    # 读取数据
    data = read_efficiency_data(input_file)

    if len(data) == 0:
        print(f"未读取到数据: {input_file}")
        return False

    # 分离扭矩和效率数据
    torque = data[:, 0]
    efficiency = data[:, 1]

    print(f"\n{'='*50}")
    print(f"处理 {motor_name} 数据")
    print(f"{'='*50}")
    print(f"读取到 {len(data)} 个数据点")
    print(f"扭矩范围: {torque.min():.6f} - {torque.max():.6f} N·m")
    print(f"效率范围: {efficiency.min():.2f}% - {efficiency.max():.2f}%")

    # 使用8阶多项式拟合
    degree = 8

    try:
        # 多项式拟合
        coeffs = np.polyfit(torque, efficiency, degree)
        p = np.poly1d(coeffs)

        # 计算拟合值
        efficiency_fit = p(torque)
        
        # 生成平滑曲线用于绘图
        torque_smooth = np.linspace(torque.min(), torque.max(), 500)
        efficiency_smooth = p(torque_smooth)

        # 计算R²值
        residuals = efficiency - efficiency_fit
        ss_res = np.sum(residuals ** 2)
        ss_tot = np.sum((efficiency - np.mean(efficiency)) ** 2)
        r_squared = 1 - (ss_res / ss_tot)

        # 计算均方根误差
        rmse = np.sqrt(np.mean(residuals ** 2))

        # 峰值效率
        peak_idx = np.argmax(efficiency)
        
        # 绘制拟合图
        plt.rcParams['font.sans-serif'] = ['SimHei', 'Microsoft YaHei']
        plt.rcParams['axes.unicode_minus'] = False
        
        plt.figure(figsize=(12, 8))
        plt.scatter(torque, efficiency, s=10, alpha=0.6, label='原始数据', color='blue')
        plt.plot(torque_smooth, efficiency_smooth, 'r-', linewidth=2, 
                 label=f'8阶拟合 (R²={r_squared:.4f})')
        plt.scatter(torque[peak_idx], efficiency[peak_idx], s=100, color='green', 
                    zorder=5, label=f'峰值: {efficiency[peak_idx]:.2f}%')
        
        plt.xlabel('扭矩 T (N·m)', fontsize=12)
        plt.ylabel('效率 η (%)', fontsize=12)
        plt.title(f'{motor_name} 效率曲线 8阶多项式拟合', fontsize=14)
        plt.legend(fontsize=10)
        plt.grid(True, alpha=0.3)
        plt.tight_layout()
        
        # 获取脚本所在目录的上上级目录作为项目根目录
        # .../PythonProject/PythonScript/Efficiency method/8th-order_fitting.py
        current_dir = os.path.dirname(os.path.abspath(__file__))
        project_root = os.path.dirname(os.path.dirname(current_dir))
        
        # 图片保存路径
        img_dir = os.path.join(project_root, "picture")
        if not os.path.exists(img_dir):
            os.makedirs(img_dir)
            
        img_file = os.path.join(img_dir, f"fitting_{motor_name}.png")
        plt.savefig(img_file, dpi=150)
        plt.close()
        print(f"拟合图已保存到 {img_file}")

        # 保存结果到文件
        with open(output_file, 'w', encoding='utf-8') as f:
            f.write("效率曲线多项式拟合结果\n")
            f.write("=" * 50 + "\n")
            f.write(f"原始数据点数: {len(data)}\n")
            f.write(f"扭矩范围: {torque.min():.6f} - {torque.max():.6f} N·m\n")
            f.write(f"效率范围: {efficiency.min():.2f}% - {efficiency.max():.2f}%\n")
            f.write(f"峰值效率: {efficiency[peak_idx]:.2f}% @ {torque[peak_idx]:.6f} N·m\n\n")

            f.write("各阶多项式拟合结果:\n")
            f.write(f"\n{degree}阶多项式:\n")
            f.write(f"  R² = {r_squared:.6f}\n")
            f.write(f"  RMSE = {rmse:.6f}\n")
            f.write(f"  多项式系数:\n")
            for i, coeff in enumerate(coeffs):
                power = len(coeffs) - i - 1
                f.write(f"    a_{power} = {coeff:.10e}\n")

            f.write(f"\n{'=' * 50}\n")
            f.write(f"推荐使用 {degree} 阶多项式拟合\n")
            f.write("多项式函数: η(T) = ")
            terms = []
            for i, coeff in enumerate(coeffs):
                power = len(coeffs) - i - 1
                if power > 1:
                    terms.append(f"{coeff:.6e}·T^{power}")
                elif power == 1:
                    terms.append(f"{coeff:.6e}·T")
                else:
                    terms.append(f"{coeff:.6e}")
            f.write(" + ".join(terms) + "\n")

        print(f"结果已保存到 {output_file}")
        print(f"R² = {r_squared:.6f}, RMSE = {rmse:.6f}")
        return True

    except Exception as e:
        print(f"拟合失败: {e}")
        return False


def main():
    import os
    
    # 获取脚本所在目录的上上级目录作为项目根目录
    # .../PythonProject/PythonScript/Efficiency method/8th-order_fitting.py
    current_dir = os.path.dirname(os.path.abspath(__file__))
    project_root = os.path.dirname(os.path.dirname(current_dir))
    
    # 数据文件路径
    data_dir = os.path.join(project_root, "data")
    
    if not os.path.exists(data_dir):
        print(f"警告：找不到数据目录 {data_dir}")
        # 尝试在当前目录查找 data（兼容性）
        if os.path.exists("data"):
            data_dir = "data"
            print("使用当前目录下的 data 文件夹")
    
    # 3508数据
    input_3508 = os.path.join(data_dir, "efficiency_curve3508.txt")
    output_3508 = os.path.join(data_dir, "polynomial_fit_3508.txt")
    
    # 6020数据
    input_6020 = os.path.join(data_dir, "efficiency_curve6020.txt")
    output_6020 = os.path.join(data_dir, "polynomial_fit_6020.txt")
    
    # 处理3508
    if os.path.exists(input_3508):
        fit_and_save(input_3508, output_3508, "3508")
    else:
        print(f"文件不存在: {input_3508}")
    
    # 处理6020
    if os.path.exists(input_6020):
        fit_and_save(input_6020, output_6020, "6020")
    else:
        print(f"文件不存在: {input_6020}")
    
    print(f"\n{'='*50}")
    print("处理完成!")
    print(f"{'='*50}")


if __name__ == "__main__":
    main()