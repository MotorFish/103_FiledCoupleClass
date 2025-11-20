#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
PMSM场路耦合计算器使用示例

本示例展示了如何使用PMSMFieldCoupledCalculator进行基本计算。
"""

import numpy as np
import logging
from pmsm_fcc_calculator import PMSMFieldCoupledCalculator

# 配置日志
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger(__name__)


def main():
    """主函数"""
    logger.info("="*70)
    logger.info("PMSM场路耦合计算器使用示例")
    logger.info("="*70)
    
    # ========================================
    # 步骤1: 准备电机参数
    # ========================================
    logger.info("\n步骤1: 准备电机参数")
    
    # 生成电流范围
    id_range = np.linspace(-300, 100, 61)
    iq_range = np.linspace(-300, 300, 71)
    
    # 磁链-id特性（二次函数近似）
    PsiR_vs_id = np.column_stack([
        id_range,
        1.0 - 2.0e-4 * id_range - 2.5e-7 * id_range**2
    ])
    
    # d轴电感-id特性
    id0 = -133.33
    Ld0 = 0.006
    a = -1.5e-8
    Lad_vs_id = np.column_stack([
        id_range,
        Ld0 + a * (id_range - id0) ** 2
    ])
    
    # q轴电感-iq特性
    Laq_vs_iq = np.column_stack([
        iq_range,
        9e-3 - 8.0e-9 * iq_range**2
    ])
    
    logger.info("✓ 电机参数准备完成")
    
    # ========================================
    # 步骤2: 创建计算器实例
    # ========================================
    logger.info("\n步骤2: 创建计算器实例")
    
    calculator = PMSMFieldCoupledCalculator(
        PsiR_vs_id=PsiR_vs_id,
        Lad_vs_id=Lad_vs_id,
        Laq_vs_iq=Laq_vs_iq,
        Udc=540.0,              # 母线电压 [V]
        Imax=300.0,             # 限幅电流 [A]
        connectType='Y',        # Y型连接
        modulationType='SVPWM', # SVPWM调制
        polePairs=4,            # 4对极
        Lsigma=0.015e-3,        # 漏抗 [H]
        Rs=0.02,                # 相电阻 [Ω]
        We_base=100.0           # 基速 [rad/s]
    )
    
    logger.info(f"✓ 计算器创建成功")
    logger.info(f"  - UPmax = {calculator.UPmax:.2f} V")
    logger.info(f"  - IPmax = {calculator.IPmax:.2f} A")
    logger.info(f"  - 是否退磁 = {calculator.isDemag}")
    if calculator.isDemag:
        logger.info(f"  - 去磁电流 = {calculator.id_demag:.2f} A")
    
    # ========================================
    # 步骤3: 计算MTPA轨迹
    # ========================================
    logger.info("\n步骤3: 计算MTPA控制轨迹")
    
    mtpa_track = calculator.calc_mtpa_track()
    
    logger.info(f"✓ MTPA轨迹计算完成")
    logger.info(f"  - 轨迹点数: {len(mtpa_track)}")
    logger.info(f"  - 最大转矩: {mtpa_track[-1, 3]:.2f} N·m")
    logger.info(f"  - 最大电流: {mtpa_track[-1, 2]:.2f} A")
    logger.info(f"  - 基速转速: {mtpa_track[-1, 4]:.2f} rad/s")
    
    # ========================================
    # 步骤4: 生成非弱磁控制结果
    # ========================================
    logger.info("\n步骤4: 生成非弱磁控制结果（恒转矩负载）")
    
    temTarget = 500.0  # 目标转矩 [N·m]
    weTarget = 200.0   # 目标转速 [rad/s]
    
    result = calculator.generate_non_fw_result(
        controlType='mtpa',
        temTarget=temTarget,
        weTarget=weTarget,
        loadType='constTem'
    )
    
    logger.info(f"✓ 非弱磁控制结果生成完成")
    logger.info(f"  - 电流范围点数: {len(result['IsRange'])}")
    logger.info(f"  - 最大容量轨迹点数: {len(result['IsTrackAtMaxCapacity'])}")
    logger.info(f"  - 恒转矩轨迹点数: {len(result['equTemTrack'])}")
    logger.info(f"  - 恒转速轨迹点数: {len(result['equWeTrack'])}")
    
    givenPoint = result['givenPoint']
    logger.info(f"\n  工况点参数:")
    logger.info(f"  - iq = {givenPoint[0]:.2f} A")
    logger.info(f"  - id = {givenPoint[1]:.2f} A")
    logger.info(f"  - Tem = {givenPoint[2]:.2f} N·m (目标: {temTarget:.2f} N·m)")
    logger.info(f"  - We = {givenPoint[3]:.2f} rad/s (目标: {weTarget:.2f} rad/s)")
    
    # 验证计算结果
    Tem_calc = calculator._calc_Tem(givenPoint[1], givenPoint[0])
    Up_calc = calculator._calc_Up(givenPoint[1], givenPoint[0], givenPoint[3])
    logger.info(f"\n  验证计算:")
    logger.info(f"  - 转矩校验: {Tem_calc:.2f} N·m")
    logger.info(f"  - 电压校验: {Up_calc:.2f} V (限值: {calculator.UPmax:.2f} V)")
    
    # ========================================
    # 步骤5: 生成弱磁控制结果（如果适用）
    # ========================================
    if calculator.isDemag:
        logger.info("\n步骤5: 生成弱磁控制结果（恒转速负载）")
        
        temTarget_fw = 300.0  # 目标转矩 [N·m]
        weTarget_fw = 500.0   # 目标转速 [rad/s]（高速）
        
        fw_result = calculator.generate_fw_result(
            controlType='mtpa',
            temTarget=temTarget_fw,
            weTarget=weTarget_fw,
            loadType='constWe'
        )
        
        logger.info(f"✓ 弱磁控制结果生成完成")
        logger.info(f"  - 电流范围点数: {len(fw_result['IsRange'])}")
        logger.info(f"  - 最大容量轨迹点数: {len(fw_result['IsTrackAtMaxCapacity'])}")
        
        fw_givenPoint = fw_result['givenPoint']
        logger.info(f"\n  弱磁工况点参数:")
        logger.info(f"  - iq = {fw_givenPoint[0]:.2f} A")
        logger.info(f"  - id = {fw_givenPoint[1]:.2f} A")
        logger.info(f"  - Tem = {fw_givenPoint[2]:.2f} N·m (目标: {temTarget_fw:.2f} N·m)")
        logger.info(f"  - We = {fw_givenPoint[3]:.2f} rad/s (目标: {weTarget_fw:.2f} rad/s)")
        
        # 验证弱磁计算结果
        Tem_fw_calc = calculator._calc_Tem(fw_givenPoint[1], fw_givenPoint[0])
        Up_fw_calc = calculator._calc_Up(fw_givenPoint[1], fw_givenPoint[0], fw_givenPoint[3])
        logger.info(f"\n  弱磁验证计算:")
        logger.info(f"  - 转矩校验: {Tem_fw_calc:.2f} N·m")
        logger.info(f"  - 电压校验: {Up_fw_calc:.2f} V (限值: {calculator.UPmax:.2f} V)")
    else:
        logger.info("\n步骤5: 未发生完全退磁，跳过弱磁控制")
    
    # ========================================
    # 总结
    # ========================================
    logger.info("\n" + "="*70)
    logger.info("示例运行完成！")
    logger.info("="*70)
    logger.info("\n要运行完整测试和生成可视化图表，请执行:")
    logger.info("  python test_pmsm_fcc.py")
    logger.info("\n要查看详细文档，请查看:")
    logger.info("  README.md")
    logger.info("="*70)


if __name__ == '__main__':
    main()

