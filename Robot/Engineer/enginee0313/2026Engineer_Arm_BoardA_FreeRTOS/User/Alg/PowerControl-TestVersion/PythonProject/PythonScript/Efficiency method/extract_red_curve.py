import cv2
import numpy as np
import matplotlib.pyplot as plt

"""
自动提取图片中红色效率曲线的数据点
"""

def extract_red_curve(image_path, num_points=400):
    """
    从图片中提取红色曲线数据
    
    参数:
        image_path: 图片路径
        num_points: 需要提取的数据点数量
    """
    # 读取图片（支持中文路径）
    img = cv2.imdecode(np.fromfile(image_path, dtype=np.uint8), cv2.IMREAD_COLOR)
    if img is None:
        print(f"无法读取图片: {image_path}")
        return None
    
    print(f"图片尺寸: {img.shape[1]} x {img.shape[0]}")
    
    # 转换到HSV颜色空间
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    
    # 红色在HSV中有两个范围（因为H值是环形的）
    # 红色范围1: H=0-10
    lower_red1 = np.array([0, 50, 50])
    upper_red1 = np.array([10, 255, 255])
    
    # 红色范围2: H=170-180
    lower_red2 = np.array([170, 50, 50])
    upper_red2 = np.array([180, 255, 255])
    
    # 创建红色掩码
    mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
    mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
    red_mask = cv2.bitwise_or(mask1, mask2)
    
    # 形态学操作去噪
    kernel = np.ones((3, 3), np.uint8)
    red_mask = cv2.morphologyEx(red_mask, cv2.MORPH_CLOSE, kernel)
    red_mask = cv2.morphologyEx(red_mask, cv2.MORPH_OPEN, kernel)
    
    # 找到红色像素的坐标
    red_points = np.where(red_mask > 0)
    
    if len(red_points[0]) == 0:
        print("未检测到红色曲线")
        return None
    
    print(f"检测到 {len(red_points[0])} 个红色像素")
    
    # 图片坐标系标定（需要根据实际图片调整）
    # 这些值定义了坐标轴在图片中的像素位置
    img_height, img_width = img.shape[:2]
    
    # 坐标轴像素范围（根据图片调整）
    pixel_left = int(img_width * 0.08)    # X轴起点
    pixel_right = int(img_width * 0.90)   # X轴终点
    pixel_top = int(img_height * 0.12)    # Y轴顶部
    pixel_bottom = int(img_height * 0.88) # Y轴底部
    
    # 数据范围
    x_min, x_max = 0.0, 1.6      # T(N·m)
    y_min, y_max = 0.0, 45.0     # η(%)
    
    # 提取曲线数据点
    # 按X坐标分组，每组取Y的中值
    curve_data = {}
    
    for i in range(len(red_points[0])):
        py = red_points[0][i]  # 像素Y坐标
        px = red_points[1][i]  # 像素X坐标
        
        # 只处理坐标轴范围内的点
        if pixel_left <= px <= pixel_right and pixel_top <= py <= pixel_bottom:
            if px not in curve_data:
                curve_data[px] = []
            curve_data[px].append(py)
    
    # 对每个X取Y的中值
    x_pixels = sorted(curve_data.keys())
    data_points = []
    
    for px in x_pixels:
        py = np.median(curve_data[px])
        
        # 转换为实际坐标
        t_value = x_min + (px - pixel_left) / (pixel_right - pixel_left) * (x_max - x_min)
        eta_value = y_max - (py - pixel_top) / (pixel_bottom - pixel_top) * (y_max - y_min)
        
        # 过滤合理范围
        if 0 <= eta_value <= 45:
            data_points.append((t_value, eta_value))
    
    print(f"提取到 {len(data_points)} 个原始数据点")
    
    # 重采样到指定数量的点
    if len(data_points) > num_points:
        # 均匀采样
        indices = np.linspace(0, len(data_points) - 1, num_points, dtype=int)
        data_points = [data_points[i] for i in indices]
    
    print(f"重采样后 {len(data_points)} 个数据点")
    
    return data_points, img, red_mask


def save_to_file(data_points, filename):
    """保存数据到文件"""
    with open(filename, 'w', encoding='utf-8') as f:
        f.write("扭矩(N·m)\t效率(%)\n")
        f.write("-" * 30 + "\n")
        for t, eta in data_points:
            f.write(f"{t:.6f}\t{eta:.2f}\n")
    print(f"数据已保存到 {filename}")


def main():
    print("=" * 50)
    print("红色曲线自动提取工具")
    print("=" * 50)
    
    image_path = input("请输入图片路径: ").strip()
    if not image_path:
        print("未输入路径")
        return
    
    num_points = input("需要提取多少个数据点? (默认400): ").strip()
    num_points = int(num_points) if num_points else 400
    
    result = extract_red_curve(image_path, num_points)
    
    if result is None:
        return
    
    data_points, img, red_mask = result
    
    # 设置中文字体
    plt.rcParams['font.sans-serif'] = ['SimHei', 'Microsoft YaHei']
    plt.rcParams['axes.unicode_minus'] = False
    
    # 显示结果
    fig, axes = plt.subplots(2, 2, figsize=(14, 10))
    
    # 原图
    axes[0, 0].imshow(cv2.cvtColor(img, cv2.COLOR_BGR2RGB))
    axes[0, 0].set_title('原图')
    axes[0, 0].axis('off')
    
    # 红色掩码
    axes[0, 1].imshow(red_mask, cmap='gray')
    axes[0, 1].set_title('红色检测掩码')
    axes[0, 1].axis('off')
    
    # 提取的数据点
    data = np.array(data_points)
    axes[1, 0].plot(data[:, 0], data[:, 1], 'r.-', markersize=2)
    axes[1, 0].set_xlabel('扭矩 T (N·m)')
    axes[1, 0].set_ylabel('效率 η (%)')
    axes[1, 0].set_title(f'提取的数据点 ({len(data_points)}个)')
    axes[1, 0].grid(True, alpha=0.3)
    
    # 叠加显示
    img_rgb = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
    axes[1, 1].imshow(img_rgb)
    # 将数据点转换回像素坐标显示
    img_height, img_width = img.shape[:2]
    pixel_left = int(img_width * 0.08)
    pixel_right = int(img_width * 0.90)
    pixel_top = int(img_height * 0.12)
    pixel_bottom = int(img_height * 0.88)
    
    px = pixel_left + (data[:, 0] / 1.6) * (pixel_right - pixel_left)
    py = pixel_bottom - (data[:, 1] / 45.0) * (pixel_bottom - pixel_top)
    axes[1, 1].plot(px, py, 'g.', markersize=3, alpha=0.7)
    axes[1, 1].set_title('数据点叠加到原图')
    axes[1, 1].axis('off')
    
    plt.tight_layout()
    plt.show()
    
    # 保存数据
    save_choice = input("\n是否保存数据? (y/n): ").lower()
    if save_choice == 'y':
        import os
        
        # 获取脚本所在目录的上上级目录作为项目根目录
        current_dir = os.path.dirname(os.path.abspath(__file__))
        project_root = os.path.dirname(os.path.dirname(current_dir))
        
        data_dir = os.path.join(project_root, "data")
        
        if not os.path.exists(data_dir):
            os.makedirs(data_dir)
            
        output_file = os.path.join(data_dir, "6020.txt")
        save_to_file(data_points, output_file)


if __name__ == "__main__":
    main()
