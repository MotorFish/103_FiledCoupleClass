#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
PMSM场路耦合计算器使用示例 - 从外部CSV文件加载参数

本示例展示了如何从外部CSV文件读取电机参数，创建PMSMFieldCoupledCalculator实例，
并运行完整的测试套件。

文件结构：
    测试文件夹/
    ├── baseParams.csv  - 基本参数（Udc, Imax, connectType等）
    ├── psid.csv       - d轴总磁链-id曲线
    ├── ld.csv         - d轴电感-id曲线
    └── lq.csv         - q轴电感-iq曲线

使用方法：
    # 使用默认测试文件夹
    python example_usage.py
    
    # 指定测试文件夹
    python example_usage.py --folder ../ExternalValidation/example2
"""

import os
import sys
import argparse
import numpy as np
import logging

# 配置日志
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger(__name__)


def read_base_params(csv_path: str) -> dict:
    """
    读取基本参数CSV文件
    
    文件格式：
        Udc,540
        Imax,300
        connectType,'Y'
        modulationType,'SVPWM'
        polePairs,4
        Lsigma,1.50E-05
        Rs,0.02
    
    Args:
        csv_path: baseParams.csv文件路径
    
    Returns:
        包含基本参数的字典
    """
    logger.info(f"读取基本参数文件: {csv_path}")
    
    params = {}
    
    with open(csv_path, 'r', encoding='utf-8-sig') as f:  # utf-8-sig 自动处理BOM
        for line in f:
            line = line.strip()
            if not line:  # 跳过空行
                continue
            
            # 分割键值对
            parts = line.split(',')
            if len(parts) < 2:
                continue
            
            key = parts[0].strip()
            value = ','.join(parts[1:]).strip()  # 处理可能含逗号的值
            
            # 处理不同类型的值
            if key in ['connectType', 'modulationType']:
                # 字符串类型，去掉单引号
                value = value.strip("'\"")
            elif key in ['polePairs']:
                # 整数类型
                value = int(value)
            else:
                # 浮点数类型（支持科学计数法）
                value = float(value)
            
            params[key] = value
    
    logger.info(f"✓ 基本参数读取完成，共{len(params)}个参数")
    for key, value in params.items():
        logger.info(f"  {key} = {value}")
    
    return params


def read_lookup_table(csv_path: str) -> np.ndarray:
    """
    读取查表数据CSV文件
    
    文件格式（跳过前4行标题）：
        直轴磁链(Psid)曲线
        Psid(Iq=0)
        true
        直轴电流Id[amp],直轴磁链Psid[weber]
        -3.227180004119873,0.34487593173980713
        -3.0120346546173096,0.3605777323246002
        ...
    
    Args:
        csv_path: 查表数据CSV文件路径
    
    Returns:
        N×2 的numpy数组，第1列为自变量，第2列为因变量
    """
    logger.info(f"读取查表文件: {csv_path}")
    
    data = []
    with open(csv_path, 'r', encoding='utf-8-sig') as f:  # utf-8-sig 自动处理BOM
        # 跳过前4行标题
        for _ in range(4):
            next(f)
        
        # 读取数据行
        for line in f:
            line = line.strip()
            if not line:  # 跳过空行
                continue
            
            # 分割并转换为浮点数
            parts = line.split(',')
            if len(parts) >= 2:
                try:
                    x = float(parts[0])
                    y = float(parts[1])
                    data.append([x, y])
                except ValueError:
                    logger.warning(f"跳过无效数据行: {line}")
                    continue
    
    result = np.array(data)
    logger.info(f"✓ 查表数据读取完成，共{len(result)}个数据点")
    logger.info(f"  自变量范围: [{result[0, 0]:.2f}, {result[-1, 0]:.2f}]")
    logger.info(f"  因变量范围: [{result[:, 1].min():.6f}, {result[:, 1].max():.6f}]")
    
    return result


def main(test_folder_path: str = None):
    """
    主函数：从外部CSV文件加载参数并运行测试
    
    Args:
        test_folder_path: 测试文件夹路径，如果为None则使用默认路径
    """
    logger.info("="*70)
    logger.info("PMSM场路耦合计算器 - 外部数据加载示例")
    logger.info("="*70)
    
    # 1. 确定测试文件夹路径
    if test_folder_path is None:
        # 默认路径：相对于当前脚本的位置
        script_dir = os.path.dirname(os.path.abspath(__file__))
        test_folder_path = os.path.join(script_dir, "..", "ExternalValidation", "example1")
    
    test_folder_path = os.path.abspath(test_folder_path)
    logger.info(f"\n测试文件夹: {test_folder_path}")
    
    # 检查文件夹是否存在
    if not os.path.exists(test_folder_path):
        logger.error(f"测试文件夹不存在: {test_folder_path}")
        return False
    
    # 2. 读取基本参数
    logger.info("\n" + "="*70)
    logger.info("步骤1: 读取基本参数")
    logger.info("="*70)
    
    base_params_path = os.path.join(test_folder_path, "baseParams.csv")
    if not os.path.exists(base_params_path):
        logger.error(f"baseParams.csv文件不存在: {base_params_path}")
        return False
    
    base_params = read_base_params(base_params_path)
    
    # 3. 读取查表数据
    logger.info("\n" + "="*70)
    logger.info("步骤2: 读取查表数据")
    logger.info("="*70)
    
    # 读取 psid.csv
    psid_path = os.path.join(test_folder_path, "psid.csv")
    if not os.path.exists(psid_path):
        logger.error(f"psid.csv文件不存在: {psid_path}")
        return False
    PsiD_vs_id = read_lookup_table(psid_path)
    
    # 读取 ld.csv
    ld_path = os.path.join(test_folder_path, "ld.csv")
    if not os.path.exists(ld_path):
        logger.error(f"ld.csv文件不存在: {ld_path}")
        return False
    Lad_vs_id = read_lookup_table(ld_path)
    
    # 读取 lq.csv
    lq_path = os.path.join(test_folder_path, "lq.csv")
    if not os.path.exists(lq_path):
        logger.error(f"lq.csv文件不存在: {lq_path}")
        return False
    Laq_vs_iq = read_lookup_table(lq_path)
    
    # 4. 组装参数字典
    logger.info("\n" + "="*70)
    logger.info("步骤3: 组装参数")
    logger.info("="*70)
    
    params = {
        'PsiD_vs_id': PsiD_vs_id,  # 注意：传入PsiD，不是PsiR
        'Lad_vs_id': Lad_vs_id,
        'Laq_vs_iq': Laq_vs_iq,
        **base_params  # 展开基本参数
    }
    
    logger.info(f"✓ 参数组装完成")
    logger.info(f"  PsiD_vs_id: {PsiD_vs_id.shape}")
    logger.info(f"  Lad_vs_id: {Lad_vs_id.shape}")
    logger.info(f"  Laq_vs_iq: {Laq_vs_iq.shape}")
    
    # 5. 创建输出目录（在测试文件夹下）
    logger.info("\n" + "="*70)
    logger.info("步骤4: 创建输出目录")
    logger.info("="*70)
    
    output_dir = os.path.join(test_folder_path, "test_results")
    logger.info(f"输出目录: {output_dir}")
    
    # 6. 导入并配置测试模块
    logger.info("\n" + "="*70)
    logger.info("步骤5: 配置测试模块")
    logger.info("="*70)
    
    # 动态导入test_pmsm_fcc
    import test_pmsm_fcc
    
    # 设置输出目录
    test_pmsm_fcc.setup_output_directory(output_dir)
    logger.info(f"✓ 测试输出目录设置为: {test_pmsm_fcc.OUTPUT_DIR}")
    
    # 重新配置日志（输出到新目录）
    test_pmsm_fcc.setup_logging(output_dir)
    logger.info(f"✓ 日志系统已重新配置")
    
    # 修改 prepare_test_motor_parameters 函数以使用外部参数
    original_prepare = test_pmsm_fcc.prepare_test_motor_parameters
    
    def prepare_external_params():
        """使用外部加载的参数"""
        test_pmsm_fcc.logger.info("使用外部加载的电机参数")
        return params
    
    # 临时替换函数
    test_pmsm_fcc.prepare_test_motor_parameters = prepare_external_params
    
    # 7. 运行所有测试
    logger.info("\n" + "="*70)
    logger.info("步骤6: 运行完整测试套件")
    logger.info("="*70)
    
    try:
        test_pmsm_fcc.run_all_tests()
        logger.info("\n" + "="*70)
        logger.info("✓ 测试执行完成！")
        logger.info("="*70)
        logger.info(f"\n测试结果保存在: {output_dir}/")
        logger.info("\n查看生成的图表和日志文件以了解详细结果。")
        logger.info("="*70)
        
        # 恢复原函数
        test_pmsm_fcc.prepare_test_motor_parameters = original_prepare
        
        return True
        
    except Exception as e:
        logger.error(f"\n测试执行过程中发生错误: {e}", exc_info=True)
        # 恢复原函数
        test_pmsm_fcc.prepare_test_motor_parameters = original_prepare
        return False


if __name__ == '__main__':
    # 命令行参数解析
    parser = argparse.ArgumentParser(
        description='从外部CSV文件加载PMSM参数并运行测试',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
示例:
    # 使用默认测试文件夹 (../ExternalValidation/example1)
    python example_usage.py
    
    # 指定测试文件夹
    python example_usage.py --folder ../ExternalValidation/example2
    
    # 使用绝对路径
    python example_usage.py --folder "G:/Easimotor/WorkingFolder/SoftwareFunctionDesign/103_FiledCoupleClass/ExternalValidation/example1"
        """
    )
    parser.add_argument(
        '--folder', '-f',
        type=str,
        default=None,
        help='测试文件夹路径（包含baseParams.csv、psid.csv、ld.csv、lq.csv）'
    )
    
    args = parser.parse_args()
    
    # 执行主函数
    success = main(args.folder)
    
    # 设置退出码
    sys.exit(0 if success else 1)
