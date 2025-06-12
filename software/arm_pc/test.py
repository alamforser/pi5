#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# 文件：test_pyqt5_fixed.py

import sys
import os
import locale

# 首先检查系统编码设置
print("=== 系统编码信息 ===")
print(f"默认编码: {sys.getdefaultencoding()}")
print(f"文件系统编码: {sys.getfilesystemencoding()}")
print(f"标准输出编码: {sys.stdout.encoding}")
print(f"区域设置: {locale.getlocale()}")
print()

# 正确的导入方式
print("=== PyQt5 导入测试 ===")
try:
    # 方式1：导入整个 PyQt5 包（注意：这不会导入子模块）
    import PyQt5
    print(f"PyQt5 包路径: {PyQt5.__file__}")
    
    # 方式2：正确导入需要的子模块
    from PyQt5 import QtCore
    print(f"PyQt5 版本: {QtCore.PYQT_VERSION_STR}")
    print(f"Qt 版本: {QtCore.QT_VERSION_STR}")
    
    # 列出 PyQt5 包含的主要模块
    print("\n可用的 PyQt5 子模块:")
    pyqt5_modules = ['QtCore', 'QtGui', 'QtWidgets', 'QtNetwork', 
                     'QtXml', 'QtSvg', 'QtSql', 'QtMultimedia']
    for module_name in pyqt5_modules:
        try:
            module = __import__(f'PyQt5.{module_name}', fromlist=[module_name])
            print(f"  ? {module_name}")
        except ImportError:
            print(f"  ? {module_name} (未安装)")
            
except ImportError as e:
    print(f"导入错误: {e}")
    print("\n可能的原因：")
    print("1. PyQt5 安装不完整")
    print("2. Python 路径配置问题")
    print("3. 系统依赖缺失")

# 检查 PyQt5 的安装位置和内容
print("\n=== PyQt5 安装详情 ===")
try:
    import PyQt5
    pyqt5_dir = os.path.dirname(PyQt5.__file__)
    print(f"PyQt5 目录: {pyqt5_dir}")
    
    # 列出目录内容
    print("\n目录内容（前10个文件）:")
    files = os.listdir(pyqt5_dir)
    for i, file in enumerate(files[:10]):
        print(f"  - {file}")
    if len(files) > 10:
        print(f"  ... 还有 {len(files) - 10} 个文件")
        
    # 检查关键的 .so 文件（Linux 下的共享库）
    print("\n关键的共享库文件:")
    so_files = [f for f in files if f.endswith('.so')]
    for so_file in so_files[:5]:
        print(f"  - {so_file}")
        
except Exception as e:
    print(f"检查时出错: {e}")