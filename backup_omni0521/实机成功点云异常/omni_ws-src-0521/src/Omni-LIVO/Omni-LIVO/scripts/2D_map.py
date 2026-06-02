#python3 2D.py all_raw_points.pcd -r 0.1 --nb-neighbors 30 --std-ratio 1.0 --save-pgm


#!/usr/bin/env python3
import numpy as np
import open3d as o3d
from PIL import Image
import argparse
import os

def denoise_pointcloud(pcd, method='statistical', **kwargs):
    """
    对点云进行降噪处理
    
    Args:
        pcd: Open3D点云对象
        method: 降噪方法 ('statistical', 'radius', 'voxel', 'combined')
        **kwargs: 各种降噪参数
    
    Returns:
        降噪后的点云对象
    """
    
    original_points = len(pcd.points)
    print(f"原始点云数量: {original_points}")
    
    if method == 'statistical':
        # 统计学滤波 - 去除离群点
        nb_neighbors = kwargs.get('nb_neighbors', 20)  # 邻居点数量
        std_ratio = kwargs.get('std_ratio', 2.0)       # 标准差倍数
        
        print(f"执行统计学滤波 (邻居点数: {nb_neighbors}, 标准差倍数: {std_ratio})")
        pcd_clean, _ = pcd.remove_statistical_outlier(nb_neighbors=nb_neighbors, std_ratio=std_ratio)
        
    elif method == 'radius':
        # 半径滤波 - 去除邻居数量少的点
        nb_points = kwargs.get('nb_points', 16)    # 最小邻居点数
        radius = kwargs.get('radius', 0.05)        # 搜索半径
        
        print(f"执行半径滤波 (搜索半径: {radius}m, 最小邻居数: {nb_points})")
        pcd_clean, _ = pcd.remove_radius_outlier(nb_points=nb_points, radius=radius)
        
    elif method == 'voxel':
        # 体素滤波 - 下采样的同时去除噪声
        voxel_size = kwargs.get('voxel_size', 0.01)  # 体素大小
        
        print(f"执行体素滤波 (体素大小: {voxel_size}m)")
        pcd_clean = pcd.voxel_down_sample(voxel_size=voxel_size)
        
    elif method == 'combined':
        # 组合降噪：先体素滤波，再统计学滤波
        voxel_size = kwargs.get('voxel_size', 0.008)
        nb_neighbors = kwargs.get('nb_neighbors', 20)
        std_ratio = kwargs.get('std_ratio', 2.0)
        
        print(f"执行组合降噪:")
        print(f"  1. 体素滤波 (体素大小: {voxel_size}m)")
        pcd_temp = pcd.voxel_down_sample(voxel_size=voxel_size)
        temp_points = len(pcd_temp.points)
        print(f"     体素滤波后点数: {temp_points}")
        
        print(f"  2. 统计学滤波 (邻居点数: {nb_neighbors}, 标准差倍数: {std_ratio})")
        pcd_clean, _ = pcd_temp.remove_statistical_outlier(nb_neighbors=nb_neighbors, std_ratio=std_ratio)
        
    else:
        print(f"未知的降噪方法: {method}, 跳过降噪")
        return pcd
    
    final_points = len(pcd_clean.points)
    removed_points = original_points - final_points
    removal_rate = (removed_points / original_points) * 100
    
    print(f"降噪完成:")
    print(f"  移除点数: {removed_points}")
    print(f"  剩余点数: {final_points}")
    print(f"  移除率: {removal_rate:.2f}%")
    
    return pcd_clean

def save_pgm_with_yaml(image_array, output_path, resolution, x_min, y_min, occupied_color=0, free_color=255):
    """
    保存PGM格式的地图文件及对应的YAML配置文件
    
    Args:
        image_array: 图像数组
        output_path: 输出路径（不含扩展名）
        resolution: 地图分辨率（米/像素）
        x_min, y_min: 地图原点在世界坐标系中的位置
        occupied_color: 占用区域颜色值
        free_color: 空闲区域颜色值
    """
    
    pgm_file = f"{output_path}.pgm"
    yaml_file = f"{output_path}.yaml"
    
    height, width = image_array.shape
    
    # 保存PGM文件
    try:
        with open(pgm_file, 'wb') as f:
            # PGM头部
            f.write(b'P5\n')
            f.write(f'{width} {height}\n'.encode())
            f.write(b'255\n')
            
            # 图像数据
            f.write(image_array.tobytes())
        
        print(f"成功保存PGM文件: {pgm_file}")
        
    except Exception as e:
        print(f"保存PGM文件失败: {e}")
        return False
    
    # 生成YAML配置文件
    try:
        yaml_content = f"""image: {os.path.basename(pgm_file)}
resolution: {resolution}
origin: [{x_min}, {y_min}, 0.0]
negate: 0
occupied_thresh: 0.65
free_thresh: 0.196
"""
        
        with open(yaml_file, 'w') as f:
            f.write(yaml_content)
        
        print(f"成功保存YAML文件: {yaml_file}")
        
        # 打印YAML内容
        print("YAML配置内容:")
        print(yaml_content)
        
        return True
        
    except Exception as e:
        print(f"保存YAML文件失败: {e}")
        return False

def pcd_to_2d_image(pcd_file, output_image, resolution=0.01, occupied_color=0, free_color=255,
                   denoise=True, denoise_method='combined', denoise_params=None, 
                   save_pgm=False, pgm_output=None, z_min=None, z_max=None):
    """
    将PCD点云文件直接投影为2D图像
    
    Args:
        pcd_file: 输入PCD文件路径
        output_image: 输出图像文件路径
        resolution: 分辨率，单位米/像素 (默认1cm/pixel)
        occupied_color: 有点的区域颜色值 (0-255)
        free_color: 无点的区域颜色值 (0-255)
        denoise: 是否进行点云降噪
        denoise_method: 降噪方法
        denoise_params: 降噪参数字典
        save_pgm: 是否同时保存PGM格式
        pgm_output: PGM输出路径（不含扩展名）
        z_min: Z轴最小高度，用于过滤
        z_max: Z轴最大高度，用于过滤
    """
    
    print(f"正在读取PCD文件: {pcd_file}")
    
    # 读取PCD文件
    try:
        pcd = o3d.io.read_point_cloud(pcd_file)
        
        if len(pcd.points) == 0:
            raise ValueError("PCD文件中没有点云数据")
            
        print(f"成功读取 {len(pcd.points)} 个点")
        
    except Exception as e:
        print(f"读取PCD文件失败: {e}")
        return False
    
    # Z轴高度滤波
    if z_min is not None or z_max is not None:
        print("-" * 40)
        original_count = len(pcd.points)
        z_min_str = f"{z_min:.2f}" if z_min is not None else "-inf"
        z_max_str = f"{z_max:.2f}" if z_max is not None else "+inf"
        print(f"执行Z轴高度滤波 (范围: Z[{z_min_str}, {z_max_str}])")
        
        points = np.asarray(pcd.points)
        mask = np.ones(len(points), dtype=bool)
        if z_min is not None:
            mask &= (points[:, 2] >= z_min)
        if z_max is not None:
            mask &= (points[:, 2] <= z_max)
            
        pcd.points = o3d.utility.Vector3dVector(points[mask])
        filtered_count = len(pcd.points)
        removed_count = original_count - filtered_count
        print(f"  移除了 {removed_count} 个点, 剩余 {filtered_count} 个点")
        print("-" * 40)

    # 点云降噪处理
    if denoise:
        if denoise_params is None:
            denoise_params = {}
        
        print("-" * 40)
        pcd = denoise_pointcloud(pcd, denoise_method, **denoise_params)
        print("-" * 40)
    
    # 获取点云坐标
    points = np.asarray(pcd.points)
    
    if len(points) == 0:
        print("错误: 降噪后没有剩余点云数据")
        return False
    
    # 提取X,Y坐标 (忽略Z轴，直接投影到XY平面)
    x_coords = points[:, 0]
    y_coords = points[:, 1]
    
    # 计算边界
    x_min, x_max = np.min(x_coords), np.max(x_coords)
    y_min, y_max = np.min(y_coords), np.max(y_coords)
    
    print(f"点云范围: X[{x_min:.3f}, {x_max:.3f}], Y[{y_min:.3f}, {y_max:.3f}]")
    
    # 计算图像尺寸
    width = int((x_max - x_min) / resolution) + 1
    height = int((y_max - y_min) / resolution) + 1
    
    print(f"生成图像尺寸: {width} x {height} 像素")
    print(f"分辨率: {resolution}m/pixel ({resolution*100:.1f}cm/pixel)")
    
    # 创建空白图像 (初始化为free_color)
    image_array = np.full((height, width), free_color, dtype=np.uint8)
    
    # 将点投影到图像坐标
    pixel_x = ((x_coords - x_min) / resolution).astype(int)
    pixel_y = ((y_coords - y_min) / resolution).astype(int)
    
    # 确保坐标在图像范围内
    pixel_x = np.clip(pixel_x, 0, width - 1)
    pixel_y = np.clip(pixel_y, 0, height - 1)
    
    # 标记有点的位置 (注意Y轴需要翻转，图像坐标系Y轴向下)
    image_array[height - 1 - pixel_y, pixel_x] = occupied_color
    
    # 转换为PIL图像并保存
    image = Image.fromarray(image_array, mode='L')
    
    # 保存常规图像格式
    try:
        image.save(output_image)
        print(f"成功保存图像: {output_image}")
        
    except Exception as e:
        print(f"保存图像失败: {e}")
        return False
    
    # 保存PGM格式（如果需要）
    if save_pgm:
        if pgm_output is None:
            # 自动生成PGM文件名
            base_name = os.path.splitext(output_image)[0]
            pgm_output = f"{base_name}_map"
        
        print("-" * 40)
        print("保存PGM格式地图文件...")
        pgm_success = save_pgm_with_yaml(image_array, pgm_output, resolution, x_min, y_min, 
                                       occupied_color, free_color)
        if not pgm_success:
            print("PGM文件保存失败，但常规图像已保存")
        print("-" * 40)
    
    # 打印统计信息
    occupied_pixels = np.sum(image_array == occupied_color)
    total_pixels = width * height
    coverage = (occupied_pixels / total_pixels) * 100
    
    print(f"图像统计:")
    print(f"  总像素数: {total_pixels}")
    print(f"  占用像素数: {occupied_pixels}")
    print(f"  覆盖率: {coverage:.2f}%")
    
    return True

def main():
    parser = argparse.ArgumentParser(description='将PCD点云文件转换为2D图像（支持点云降噪）')
    parser.add_argument('input_pcd', help='输入PCD文件路径')
    parser.add_argument('-o', '--output', help='输出图像文件路径 (默认: input_name.png)')
    parser.add_argument('-r', '--resolution', type=float, default=0.01, 
                       help='分辨率，米/像素 (默认: 0.01, 即1cm/pixel)')
    parser.add_argument('--occupied-color', type=int, default=0, 
                       help='有点区域的颜色值 0-255 (默认: 0, 黑色)')
    parser.add_argument('--free-color', type=int, default=255, 
                       help='无点区域的颜色值 0-255 (默认: 255, 白色)')
    
    # PGM输出相关参数
    parser.add_argument('--save-pgm', action='store_true', 
                       help='同时保存PGM格式地图文件（用于ROS）')
    parser.add_argument('--pgm-output', 
                       help='PGM输出路径前缀（不含扩展名，默认基于输入文件名）')
    
    # 降噪相关参数
    parser.add_argument('--no-denoise', action='store_true', 
                       help='禁用点云降噪')
    parser.add_argument('--denoise-method', choices=['statistical', 'radius', 'voxel', 'combined'], 
                       default='combined', help='降噪方法 (默认: combined)')
    
    # 统计学滤波参数
    parser.add_argument('--nb-neighbors', type=int, default=20, 
                       help='统计学滤波邻居点数 (默认: 20)')
    parser.add_argument('--std-ratio', type=float, default=2.0, 
                       help='统计学滤波标准差倍数 (默认: 2.0)')
    
    # 半径滤波参数
    parser.add_argument('--radius', type=float, default=0.05, 
                       help='半径滤波搜索半径,米 (默认: 0.05)')
    parser.add_argument('--nb-points', type=int, default=16, 
                       help='半径滤波最小邻居点数 (默认: 16)')
    
    # 体素滤波参数
    parser.add_argument('--voxel-size', type=float, default=0.008, 
                       help='体素滤波大小,米 (默认: 0.008)')
    
    # Z轴滤波参数
    parser.add_argument('--z-min', type=float, help='Z轴最小高度，用于过滤点云，截取一个高度范围内的点 (例如: 0.1)')
    parser.add_argument('--z-max', type=float, help='Z轴最大高度，用于过滤点云，截取一个高度范围内的点 (例如: 2.0)')


    args = parser.parse_args()
    
    # 检查输入文件
    if not os.path.exists(args.input_pcd):
        print(f"错误: 输入文件不存在 - {args.input_pcd}")
        return
    
    # 生成输出文件名
    if args.output:
        output_image = args.output
    else:
        base_name = os.path.splitext(args.input_pcd)[0]
        denoise_suffix = "_denoised" if not args.no_denoise else ""
        output_image = f"{base_name}{denoise_suffix}_2d_map.png"
    
    # 验证颜色值范围
    if not (0 <= args.occupied_color <= 255) or not (0 <= args.free_color <= 255):
        print("错误: 颜色值必须在0-255范围内")
        return
    
    # 准备降噪参数
    denoise_params = {
        'nb_neighbors': args.nb_neighbors,
        'std_ratio': args.std_ratio,
        'radius': args.radius,
        'nb_points': args.nb_points,
        'voxel_size': args.voxel_size
    }
    
    print("=== PCD转2D图像工具 (支持点云降噪 + PGM输出) ===")
    print(f"输入文件: {args.input_pcd}")
    print(f"输出文件: {output_image}")
    if args.save_pgm:
        pgm_path = args.pgm_output if args.pgm_output else f"{os.path.splitext(output_image)[0]}_map"
        print(f"PGM输出: {pgm_path}.pgm + {pgm_path}.yaml")
    print(f"分辨率: {args.resolution}m/pixel")
    if args.z_min is not None or args.z_max is not None:
        z_range = f"Z-min={args.z_min if args.z_min is not None else 'N/A'}, Z-max={args.z_max if args.z_max is not None else 'N/A'}"
        print(f"高度滤波: 启用 ({z_range})")
    print(f"颜色设置: 占用={args.occupied_color}, 空闲={args.free_color}")
    if not args.no_denoise:
        print(f"降噪方法: {args.denoise_method}")
        if args.denoise_method in ['statistical', 'combined']:
            print(f"  统计学滤波: 邻居={args.nb_neighbors}, 标准差倍数={args.std_ratio}")
        if args.denoise_method == 'radius':
            print(f"  半径滤波: 半径={args.radius}m, 最小邻居={args.nb_points}")
        if args.denoise_method in ['voxel', 'combined']:
            print(f"  体素滤波: 大小={args.voxel_size}m")
    else:
        print("降噪: 禁用")
    print("=" * 50)
    
    # 执行转换
    success = pcd_to_2d_image(
        args.input_pcd, 
        output_image, 
        args.resolution,
        args.occupied_color,
        args.free_color,
        denoise=not args.no_denoise,
        denoise_method=args.denoise_method,
        denoise_params=denoise_params,
        save_pgm=args.save_pgm,
        pgm_output=args.pgm_output,
        z_min=args.z_min,
        z_max=args.z_max
    )
    
    if success:
        print("\n转换完成!")
    else:
        print("\n转换失败!")

if __name__ == "__main__":
    main()
