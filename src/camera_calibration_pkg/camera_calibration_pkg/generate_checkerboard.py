#!/usr/bin/env python3
"""
生成可打印的棋盘格标定板图像
"""

import cv2
import numpy as np
import sys


def generate_checkerboard(cols=7, rows=5, square_size=0.041, output_file='checkerboard.png'):
    """
    生成棋盘格标定板
    
    参数:
        cols: 棋盘格列数（内角点）
        rows: 棋盘格行数（内角点）
        square_size: 每个方格的边长（米）
        output_file: 输出文件名
    """
    # 创建白色背景
    img_size = (1920, 1080)
    img = np.ones((img_size[1], img_size[0]), dtype=np.uint8) * 255
    
    # 计算棋盘格大小
    checker_width = cols * 100  # 像素
    checker_height = rows * 100
    
    # 起始位置（居中）
    start_x = (img_size[0] - checker_width) // 2
    start_y = (img_size[1] - checker_height) // 2
    
    # 绘制棋盘格
    for i in range(rows):
        for j in range(cols):
            x = start_x + j * 100
            y = start_y + i * 100
            
            # 黑白交替
            if (i + j) % 2 == 0:
                color = 0  # 黑色
            else:
                color = 255  # 白色
            
            cv2.rectangle(img, (x, y), (x + 100, y + 100), color, -1)
    
    # 添加边框
    cv2.rectangle(img, (start_x - 10, start_y - 10), 
                 (start_x + checker_width + 10, start_y + checker_height + 10), 
                 0, 5)
    
    # 添加文字信息
    cv2.putText(img, f'Checkerboard: {cols}x{rows}', (50, 50), 
               cv2.FONT_HERSHEY_SIMPLEX, 1.5, 0, 3)
    cv2.putText(img, f'Square Size: {square_size*1000:.1f}mm', (50, 100), 
               cv2.FONT_HERSHEY_SIMPLEX, 1.5, 0, 3)
    
    # 保存图像
    cv2.imwrite(output_file, img)
    print(f'✅ Checkerboard saved to: {output_file}')
    print(f'📏 Print this image and measure the actual square size!')
    print(f'📐 Recommended paper size: A4 or Letter')


def main(args=None):
    """主函数（ROS 2 entry_point 要求）"""
    # 默认参数
    cols = 7
    rows = 5
    square_size = 0.041  # 41mm
    output_file = 'checkerboard.png'
    
    # 从命令行读取参数
    if len(sys.argv) > 1:
        cols = int(sys.argv[1])
    if len(sys.argv) > 2:
        rows = int(sys.argv[2])
    if len(sys.argv) > 3:
        square_size = float(sys.argv[3])
    if len(sys.argv) > 4:
        output_file = sys.argv[4]
    
    # 生成棋盘格
    generate_checkerboard(cols, rows, square_size, output_file)


if __name__ == '__main__':
    main()
