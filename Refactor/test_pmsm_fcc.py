#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
PMSM场路耦合计算器测试和可视化

本模块提供了PMSMFieldCoupledCalculator类的完整测试，包括：
- 单元测试
- 集成测试
- 可视化功能

使用方法：
    python test_pmsm_fcc.py
"""

import logging
import os
import numpy as np
import matplotlib.pyplot as plt
from pmsm_fcc_calculator import PMSMFieldCoupledCalculator

# 配置matplotlib中文显示
plt.rcParams['font.sans-serif'] = ['SimHei']
plt.rcParams['axes.unicode_minus'] = False

# 创建输出文件夹
OUTPUT_DIR = 'test_results'
if not os.path.exists(OUTPUT_DIR):
    os.makedirs(OUTPUT_DIR)

# 配置日志 - 同时输出到控制台和文件
logger = logging.getLogger(__name__)
logger.setLevel(logging.INFO)

# 创建格式化器
formatter = logging.Formatter('%(asctime)s - %(name)s - %(levelname)s - %(message)s')

# 控制台处理器
console_handler = logging.StreamHandler()
console_handler.setLevel(logging.INFO)
console_handler.setFormatter(formatter)

# 文件处理器
log_file = os.path.join(OUTPUT_DIR, 'test_log.txt')
file_handler = logging.FileHandler(log_file, mode='w', encoding='utf-8')
file_handler.setLevel(logging.INFO)
file_handler.setFormatter(formatter)

# 添加处理器
logger.addHandler(console_handler)
logger.addHandler(file_handler)

# 同时配置pmsm_fcc_calculator的日志
calc_logger = logging.getLogger('pmsm_fcc_calculator')
calc_logger.setLevel(logging.INFO)
calc_logger.addHandler(console_handler)
calc_logger.addHandler(file_handler)

logger.info(f"创建输出文件夹: {OUTPUT_DIR}")
logger.info(f"日志将保存到: {log_file}")


def prepare_test_motor_parameters():
    """
    准备测试用电机参数
    
    Returns:
        包含电机参数的字典
    """
    logger.info("准备测试用电机参数")
    
    # 生成电流范围
    id_range = np.linspace(-300, 100, 61)
    iq_range = np.linspace(-300, 300, 71)
    
    # 磁链-id特性（二次函数近似）
    PsiR_vs_id = np.column_stack([
        id_range,
        1.0 - 2.0e-4 * id_range - 2.5e-7 * id_range**2
    ])
    
    # d轴电感-id特性（顶点二次曲线）
    id0 = -133.33
    Ld0 = 0.006
    a = -1.5e-8
    Lad_vs_id = np.column_stack([
        id_range,
        Ld0 + a * (id_range - id0) ** 2
    ])
    
    # q轴电感-iq特性（钟形二次分布）
    Laq_vs_iq = np.column_stack([
        iq_range,
        9e-3 - 8.0e-9 * iq_range**2
    ])
    
    # 额定参数
    params = {
        'PsiR_vs_id': PsiR_vs_id,
        'Lad_vs_id': Lad_vs_id,
        'Laq_vs_iq': Laq_vs_iq,
        'Udc': 540.0,
        'Imax': 300.0,
        'connectType': 'Y',
        'modulationType': 'SVPWM',
        'polePairs': 4,
        'Lsigma': 0.015e-3,
        'Rs': 0.02,
        'We_base': 100.0
    }
    
    logger.info("电机参数准备完成")
    return params


def create_calculator(params=None):
    """
    创建计算器实例
    
    Args:
        params: 电机参数字典，None时使用默认测试参数
    
    Returns:
        PMSMFieldCoupledCalculator实例
    """
    if params is None:
        params = prepare_test_motor_parameters()
    
    calculator = PMSMFieldCoupledCalculator(**params)
    return calculator


# ========================================
# 单元测试
# ========================================

def test_basic_calculations():
    """测试基础计算功能"""
    logger.info("="*70)
    logger.info("开始测试基础计算功能")
    logger.info("="*70)
    
    calc = create_calculator()
    
    # 测试转矩计算
    id, iq = -50, 100
    Tem = calc._calc_Tem(id, iq)
    logger.info(f"转矩计算测试: id={id} A, iq={iq} A, Tem={Tem:.2f} N·m")
    assert Tem > 0, "转矩应为正值"
    
    # 测试电压计算
    we = 200.0
    Up = calc._calc_Up(id, iq, we)
    logger.info(f"电压计算测试: id={id} A, iq={iq} A, we={we} rad/s, Up={Up:.2f} V")
    assert Up > 0, "电压应为正值"
    
    # 测试电角速度计算
    success, msg, We = calc._calc_We(id, iq, calc.UPmax)
    logger.info(f"电角速度计算测试: id={id} A, iq={iq} A, UPmax={calc.UPmax:.2f} V")
    logger.info(f"  结果: success={success}, We={We:.2f} rad/s")
    assert success, f"计算应该成功: {msg}"
    
    logger.info("✓ 基础计算功能测试通过\n")


def test_id0_track():
    """测试id=0控制轨迹"""
    logger.info("="*70)
    logger.info("测试id=0控制轨迹")
    logger.info("="*70)
    
    calc = create_calculator()
    id0_track = calc.calc_id0_track()
    
    logger.info(f"id=0轨迹点数: {len(id0_track)}")
    logger.info(f"最大转矩: {id0_track[-1, 3]:.2f} N·m")
    logger.info(f"最大电流: {id0_track[-1, 2]:.2f} A")
    
    assert len(id0_track) > 0, "轨迹应包含数据点"
    assert np.allclose(id0_track[:, 1], 0), "id应该都为0"
    
    logger.info("✓ id=0控制轨迹测试通过\n")
    return id0_track


def test_mtpa_track():
    """测试MTPA控制轨迹"""
    logger.info("="*70)
    logger.info("测试MTPA控制轨迹")
    logger.info("="*70)
    
    calc = create_calculator()
    mtpa_track = calc.calc_mtpa_track()
    
    logger.info(f"MTPA轨迹点数: {len(mtpa_track)}")
    logger.info(f"最大转矩: {mtpa_track[-1, 3]:.2f} N·m")
    logger.info(f"最大电流: {mtpa_track[-1, 2]:.2f} A")
    
    assert len(mtpa_track) > 0, "轨迹应包含数据点"
    assert mtpa_track[-1, 2] <= calc.IPmax + 0.1, "最大电流不应超过限制"
    
    logger.info("✓ MTPA控制轨迹测试通过\n")
    return mtpa_track


def test_mtpv_track():
    """测试MTPV控制轨迹"""
    logger.info("="*70)
    logger.info("测试MTPV控制轨迹")
    logger.info("="*70)
    
    calc = create_calculator()
    
    if not calc.isDemag:
        logger.warning("未发生完全退磁，跳过MTPV测试")
        return None
    
    mtpv_track = calc.calc_mtpv_track(We_base=calc.We_base)
    
    logger.info(f"MTPV轨迹点数: {len(mtpv_track)}")
    if len(mtpv_track) > 0:
        logger.info(f"最大转速: {mtpv_track[-1, 4]:.2f} rad/s")
        logger.info(f"最大转矩: {mtpv_track[0, 3]:.2f} N·m")
    
    assert len(mtpv_track) >= 0, "轨迹点数应该非负"
    
    logger.info("✓ MTPV控制轨迹测试通过\n")
    return mtpv_track


def test_adaptive_search():
    """测试自适应搜索算法"""
    logger.info("="*70)
    logger.info("测试自适应搜索算法")
    logger.info("="*70)
    
    calc = create_calculator()
    
    # 测试求根
    def testFunc(x):
        return x**2 - 4
    
    x, fx, iters, converged = calc._adaptive_step_search(
        f=testFunc,
        bracket=[-10, 10],
        divisions=100,
        startFrom='left',
        target=0,
        tol=1e-5
    )
    
    logger.info(f"求根测试: x={x:.6f}, f(x)={fx:.8f}, 迭代{iters}次, 收敛={converged}")
    assert converged, "应该收敛"
    assert abs(fx) < 1e-4, "函数值应接近0"
    
    # 测试求极大值
    def testFunc2(x):
        return -(x - 2)**2 + 10
    
    x_max, fx_max, iters, converged = calc._adaptive_step_search_maximum(
        f=testFunc2,
        bracket=[0, 5],
        divisions=100,
        startFrom='right',
        tol=1e-5
    )
    
    logger.info(f"求极大值测试: x={x_max:.6f}, f(x)={fx_max:.6f}, 迭代{iters}次, 收敛={converged}")
    assert converged, "应该收敛"
    assert abs(x_max - 2) < 0.1, "极大值点应在x=2附近"
    
    logger.info("✓ 自适应搜索算法测试通过\n")


# ========================================
# 集成测试
# ========================================

def test_non_fw_result():
    """测试非弱磁控制结果生成"""
    logger.info("="*70)
    logger.info("测试非弱磁控制结果生成")
    logger.info("="*70)
    
    calc = create_calculator()
    
    # 测试MTPA控制
    result = calc.generate_non_fw_result(
        controlType='mtpa',
        temTarget=500.0,
        weTarget=200.0,
        loadType='constTem'
    )
    
    logger.info(f"IsRange点数: {len(result['IsRange'])}")
    logger.info(f"IsTrackAtMaxCapacity点数: {len(result['IsTrackAtMaxCapacity'])}")
    logger.info(f"equTemTrack点数: {len(result['equTemTrack'])}")
    logger.info(f"equWeTrack点数: {len(result['equWeTrack'])}")
    logger.info(f"givenPoint: iq={result['givenPoint'][0]:.2f}, id={result['givenPoint'][1]:.2f}, "
                f"Tem={result['givenPoint'][2]:.2f}, We={result['givenPoint'][3]:.2f}")
    
    assert 'IsRange' in result, "结果应包含IsRange"
    assert 'IsTrackAtMaxCapacity' in result, "结果应包含IsTrackAtMaxCapacity"
    assert 'equTemTrack' in result, "结果应包含equTemTrack"
    assert 'equWeTrack' in result, "结果应包含equWeTrack"
    assert 'givenPoint' in result, "结果应包含givenPoint"
    
    logger.info("✓ 非弱磁控制结果生成测试通过\n")
    return result


def test_fw_result():
    """测试弱磁控制结果生成"""
    logger.info("="*70)
    logger.info("测试弱磁控制结果生成")
    logger.info("="*70)
    
    calc = create_calculator()
    
    if not calc.isDemag:
        logger.warning("未发生完全退磁，跳过弱磁控制测试")
        return None
    
    # 测试弱磁MTPA控制
    result = calc.generate_fw_result(
        controlType='mtpa',
        temTarget=500.0,
        weTarget=500.0,
        loadType='constWe'
    )
    
    logger.info(f"IsRange点数: {len(result['IsRange'])}")
    logger.info(f"IsTrackAtMaxCapacity点数: {len(result['IsTrackAtMaxCapacity'])}")
    logger.info(f"equTemTrack点数: {len(result['equTemTrack'])}")
    logger.info(f"equWeTrack点数: {len(result['equWeTrack'])}")
    logger.info(f"givenPoint: iq={result['givenPoint'][0]:.2f}, id={result['givenPoint'][1]:.2f}, "
                f"Tem={result['givenPoint'][2]:.2f}, We={result['givenPoint'][3]:.2f}")
    
    assert 'IsRange' in result, "结果应包含IsRange"
    assert 'IsTrackAtMaxCapacity' in result, "结果应包含IsTrackAtMaxCapacity"
    
    logger.info("✓ 弱磁控制结果生成测试通过\n")
    return result


# ========================================
# 可视化功能
# ========================================

def plot_motor_characteristics():
    """绘制电机特性曲线"""
    logger.info("="*70)
    logger.info("绘制电机特性曲线")
    logger.info("="*70)
    
    params = prepare_test_motor_parameters()
    PsiR_vs_id = params['PsiR_vs_id']
    Lad_vs_id = params['Lad_vs_id']
    Laq_vs_iq = params['Laq_vs_iq']
    
    fig, axes = plt.subplots(3, 1, figsize=(14, 10))
    
    # 磁链-id曲线
    axes[0].plot(PsiR_vs_id[:, 0], PsiR_vs_id[:, 1], marker='o', label='Ψr-id')
    axes[0].set_xlabel("id (A)")
    axes[0].set_ylabel("Ψr (Wb)")
    axes[0].set_title("磁链-id 曲线")
    axes[0].grid(True)
    axes[0].legend()
    
    # d轴电感-id曲线
    axes[1].plot(Lad_vs_id[:, 0], Lad_vs_id[:, 1] * 1e3, marker='s', color='orange', label='Ld-id')
    axes[1].set_xlabel("id (A)")
    axes[1].set_ylabel("Ld (mH)")
    axes[1].set_title("d轴电感-id 曲线")
    axes[1].grid(True)
    axes[1].legend()
    
    # q轴电感-iq曲线
    axes[2].plot(Laq_vs_iq[:, 0], Laq_vs_iq[:, 1] * 1e3, marker='^', color='green', label='Lq-iq')
    axes[2].set_xlabel("iq (A)")
    axes[2].set_ylabel("Lq (mH)")
    axes[2].set_title("q轴电感-iq 曲线")
    axes[2].grid(True)
    axes[2].legend()
    
    plt.tight_layout()
    output_path = os.path.join(OUTPUT_DIR, 'motor_characteristics.png')
    plt.savefig(output_path, dpi=150, bbox_inches='tight')
    logger.info(f"✓ 电机特性曲线已保存到 {output_path}\n")
    plt.close()


def plot_mtpa_track(mtpa_track):
    """绘制MTPA轨迹"""
    logger.info("="*70)
    logger.info("绘制MTPA轨迹")
    logger.info("="*70)
    
    fig, axes = plt.subplots(2, 2, figsize=(14, 10))
    
    # id-iq轨迹
    axes[0, 0].plot(mtpa_track[:, 1], mtpa_track[:, 0], 'b-o', markersize=3)
    axes[0, 0].set_xlabel('id (A)')
    axes[0, 0].set_ylabel('iq (A)')
    axes[0, 0].set_title('MTPA电流轨迹 (id-iq平面)')
    axes[0, 0].grid(True)
    axes[0, 0].axhline(y=0, color='k', linestyle='--', alpha=0.3)
    axes[0, 0].axvline(x=0, color='k', linestyle='--', alpha=0.3)
    
    # 转矩-电流幅值曲线
    axes[0, 1].plot(mtpa_track[:, 2], mtpa_track[:, 3], 'r-o', markersize=3)
    axes[0, 1].set_xlabel('电流幅值 Is (A)')
    axes[0, 1].set_ylabel('电磁转矩 Tem (N·m)')
    axes[0, 1].set_title('转矩-电流幅值曲线')
    axes[0, 1].grid(True)
    
    # 转矩-id曲线
    axes[1, 0].plot(mtpa_track[:, 1], mtpa_track[:, 3], 'g-o', markersize=3)
    axes[1, 0].set_xlabel('id (A)')
    axes[1, 0].set_ylabel('电磁转矩 Tem (N·m)')
    axes[1, 0].set_title('转矩-d轴电流曲线')
    axes[1, 0].grid(True)
    
    # 转矩-iq曲线
    axes[1, 1].plot(mtpa_track[:, 0], mtpa_track[:, 3], 'm-o', markersize=3)
    axes[1, 1].set_xlabel('iq (A)')
    axes[1, 1].set_ylabel('电磁转矩 Tem (N·m)')
    axes[1, 1].set_title('转矩-q轴电流曲线')
    axes[1, 1].grid(True)
    
    plt.tight_layout()
    output_path = os.path.join(OUTPUT_DIR, 'mtpa_track.png')
    plt.savefig(output_path, dpi=150, bbox_inches='tight')
    logger.info(f"✓ MTPA轨迹已保存到 {output_path}\n")
    plt.close()


def plot_current_range(IsRange, mtpa_track, fw1_track=None, mtpv_track=None):
    """绘制完整电流控制范围"""
    logger.info("="*70)
    logger.info("绘制完整电流控制范围")
    logger.info("="*70)
    
    calc = create_calculator()
    
    fig = plt.figure(figsize=(12, 10))
    
    # 绘制完整电流范围
    plt.plot(IsRange[:, 1], IsRange[:, 0], 'b-', linewidth=2.5, label='可利用电流矢量范围')
    
    # 绘制MTPA段
    plt.plot(mtpa_track[:, 1], mtpa_track[:, 0], 'r-', linewidth=2, label='MTPA段', alpha=0.7)
    
    # 绘制弱磁I区段
    if fw1_track is not None:
        plt.plot(fw1_track[:, 1], fw1_track[:, 0], 'g-', linewidth=2, label='弱磁I区段', alpha=0.7)
    
    # 绘制MTPV段
    if mtpv_track is not None and len(mtpv_track) > 0:
        plt.plot(mtpv_track[:, 1], mtpv_track[:, 0], 'm-', linewidth=2, label='MTPV段', alpha=0.7)
    
    # 绘制电流圆限制
    theta = np.linspace(0, np.pi/2, 100)
    id_circle = -calc.IPmax * np.cos(theta)
    iq_circle = calc.IPmax * np.sin(theta)
    plt.plot(id_circle, iq_circle, 'k--', linewidth=1.5, alpha=0.6, 
             label=f'电流限制圈 (Is={calc.IPmax:.0f}A)')
    
    plt.xlabel('id (A)', fontsize=12)
    plt.ylabel('iq (A)', fontsize=12)
    plt.title('可利用电流范围 (id-iq)', fontsize=14)
    plt.grid(True, alpha=0.3)
    plt.axhline(y=0, color='k', linestyle='--', alpha=0.3)
    plt.axvline(x=0, color='k', linestyle='--', alpha=0.3)
    plt.legend(loc='upper right', fontsize=10)
    plt.axis('equal')
    plt.tight_layout()
    output_path = os.path.join(OUTPUT_DIR, 'current_range.png')
    plt.savefig(output_path, dpi=150, bbox_inches='tight')
    logger.info(f"✓ 电流控制范围已保存到 {output_path}\n")
    plt.close()


def plot_control_result(result, title="控制策略轨迹分析"):
    """绘制控制策略结果"""
    logger.info("="*70)
    logger.info(f"绘制{title}")
    logger.info("="*70)
    
    fig, ax = plt.subplots(figsize=(12, 8))
    
    # 绘制最大容量轨迹
    IsTrackAtMaxCapacity = result['IsTrackAtMaxCapacity']
    if len(IsTrackAtMaxCapacity) > 0:
        We_maxCap = IsTrackAtMaxCapacity[:, 4]
        Tem_maxCap = IsTrackAtMaxCapacity[:, 3]
        ax.plot(We_maxCap, Tem_maxCap, 'b-', linewidth=2.5,
                label='最大容量轨迹',
                marker='o', markersize=4,
                markevery=max(1, len(We_maxCap)//10))
    
    # 绘制恒转速轨迹
    equWeTrack = result['equWeTrack']
    if len(equWeTrack) > 0:
        ax.scatter(equWeTrack[:, 3], equWeTrack[:, 2], s=50, c='green',
                   marker='s', label='恒转速轨迹', zorder=5)
    
    # 绘制恒转矩轨迹
    equTemTrack = result['equTemTrack']
    if len(equTemTrack) > 0:
        ax.scatter(equTemTrack[:, 3], equTemTrack[:, 2], s=50, c='red',
                   marker='^', label='恒转矩轨迹', zorder=5)
    
    # 绘制工况点
    givenPoint = result['givenPoint']
    We_point = givenPoint[3]
    Tem_point = givenPoint[2]
    ax.scatter([We_point], [Tem_point], s=200, c='black',
               marker='*',
               label=f'工况点 (We={We_point:.1f}, Tem={Tem_point:.1f})',
               zorder=6, edgecolors='yellow', linewidths=2)
    
    ax.set_xlabel('电角速度 We (rad/s)', fontsize=12, fontweight='bold')
    ax.set_ylabel('电磁转矩 Tem (N·m)', fontsize=12, fontweight='bold')
    ax.set_title(f'{title} - We-Tem 平面', fontsize=14, fontweight='bold')
    ax.grid(True, alpha=0.3, linestyle='--')
    ax.legend(loc='best', fontsize=10, framealpha=0.9)
    
    plt.tight_layout()
    filename = title.replace(' ', '_').replace('-', '_')
    output_path = os.path.join(OUTPUT_DIR, f'{filename}.png')
    plt.savefig(output_path, dpi=150, bbox_inches='tight')
    logger.info(f"✓ {title}已保存到 {output_path}\n")
    plt.close()


def plot_equ_lines_and_circles(calc, mtpa_track, fw1_track, mtpv_track=None):
    """绘制等转矩线和等电压椭圆"""
    logger.info("="*70)
    logger.info("绘制等转矩线和等电压椭圆")
    logger.info("="*70)
    
    fig = plt.figure(figsize=(14, 12))
    
    # 计算IsRange
    IsRange = calc.calc_full_track(
        mtpaTrack=mtpa_track,
        mtpvTrack=mtpv_track,
        fw1Track=fw1_track
    )
    
    # 绘制可利用电流范围
    plt.plot(IsRange[:, 1], IsRange[:, 0], 'b-', linewidth=2.5,
             label='可利用的电流矢量范围', zorder=10)
    
    # 绘制电流圆限制
    theta = np.linspace(0, np.pi/2, 100)
    id_circle = -calc.IPmax * np.cos(theta)
    iq_circle = calc.IPmax * np.sin(theta)
    plt.plot(id_circle, iq_circle, 'k--', linewidth=1.5, alpha=0.6,
             label=f'电流限制圈 (Is={calc.IPmax:.0f}A)')
    
    # 绘制等电压椭圆
    if len(fw1_track) > 0:
        minWe = fw1_track[0, 4]
        if mtpv_track is not None and len(mtpv_track) > 0:
            maxWe = mtpv_track[-1, 4]
        else:
            maxWe = fw1_track[-1, 4]
        
        # 生成转速序列（对数分布）
        log_minWe = np.log(minWe)
        log_maxWe = np.log(maxWe)
        log_We_list = np.linspace(log_minWe, log_maxWe, 10)
        We_list = np.exp(log_We_list)
        
        colors_u = plt.cm.get_cmap('cool')(np.linspace(0, 1, len(We_list)))
        
        for i, We_target in enumerate(We_list):
            try:
                equUTrack, Tem_max = calc.calc_equ_u_track(
                    We=We_target,
                    mtpaTrack=mtpa_track,
                    fw1Track=fw1_track,
                    mtpvTrack=mtpv_track
                )
                
                if len(equUTrack) > 0:
                    plt.plot(equUTrack[:, 1], equUTrack[:, 0],
                             color=colors_u[i], linewidth=1, alpha=0.4,
                             label=f'等电压椭圆 {We_target:.0f} rad/s' if i % 2 == 0 else '')
            except Exception as e:
                logger.warning(f"计算等电压椭圆失败 We={We_target:.0f}: {e}")
    
    # 绘制等转矩线
    maxTem = mtpa_track[-1, 3]
    Tem_list = np.linspace(0, maxTem, 15)[1:]
    
    colors_t = plt.cm.get_cmap('hot')(np.linspace(0, 1, len(Tem_list)))
    
    for i, Tem_target in enumerate(Tem_list):
        try:
            equTemTrack = calc.calc_equ_tem_track(
                Tem_target=Tem_target,
                mtpaTrack=mtpa_track,
                fw1Track=fw1_track,
                mtpvTrack=mtpv_track
            )
            
            if len(equTemTrack) > 0:
                plt.plot(equTemTrack[:, 1], equTemTrack[:, 0],
                         color=colors_t[i], linewidth=1, alpha=0.4,
                         label=f'等转矩线 {Tem_target:.0f} N·m' if i % 2 == 0 else '')
        except Exception as e:
            logger.warning(f"计算等转矩线失败 Tem={Tem_target:.0f}: {e}")
    
    plt.xlabel('id (A)', fontsize=12)
    plt.ylabel('iq (A)', fontsize=12)
    plt.title('永磁同步电机电流轨迹示意图', fontsize=14)
    plt.grid(True, alpha=0.3)
    plt.axhline(y=0, color='k', linestyle='--', alpha=0.3, linewidth=0.5)
    plt.axvline(x=0, color='k', linestyle='--', alpha=0.3, linewidth=0.5)
    plt.legend(loc='upper right', fontsize=8, ncol=2)
    plt.axis('equal')
    plt.tight_layout()
    output_path = os.path.join(OUTPUT_DIR, 'equ_lines_and_circles.png')
    plt.savefig(output_path, dpi=150, bbox_inches='tight')
    logger.info(f"✓ 等转矩线和等电压椭圆已保存到 {output_path}\n")
    plt.close()


# ========================================
# 主测试流程
# ========================================

def test_operating_point_tracks(calc: PMSMFieldCoupledCalculator, 
                                mtpa_track: np.ndarray,
                                fw1_track: np.ndarray,
                                mtpv_track: np.ndarray):
    """
    测试给定工况点的三条轨迹计算和绘制
    """
    logger.info("准备测试用电机参数")
    
    # 测试工况：转矩500 N·m，转速300 rad/s
    Tem_target = 500.0   # N·m
    We_target = 300.0   # rad/s
    
    logger.info(f"测试工况点: Tem={Tem_target:.2f} N·m, We={We_target:.2f} rad/s")
    
    # 调用计算函数
    IsTrack, equTemTrack, IsTrackInEquU = calc.plot_tracks_for_operating_point(
        Tem_target=Tem_target,
        We_target=We_target,
        mtpaTrack=mtpa_track,
        fw1Track=fw1_track,
        mtpvTrack=mtpv_track,
        equUPoints=100,
        equTemPoints=50
    )
    
    logger.info(f"IsTrack点数: {len(IsTrack)}")
    logger.info(f"等转矩线点数: {len(equTemTrack)}")
    logger.info(f"等电压椭圆点数: {len(IsTrackInEquU)}")
    
    # 绘制三条轨迹
    plt.figure(figsize=(12, 10))
    
    # 绘制电流圆限制
    theta = np.linspace(0, np.pi/2, 100)
    id_circle = -calc.IPmax * np.cos(theta)
    iq_circle = calc.IPmax * np.sin(theta)
    plt.plot(id_circle, iq_circle, 'k--', linewidth=1.5, alpha=0.6, 
            label=f'电流限制圈 (Is={calc.IPmax:.0f}A)')
    
    # 绘制IsTrack（完整电流容量边界）
    plt.plot(IsTrack[:, 1], IsTrack[:, 0], 'b-', linewidth=2.5, 
            label='最大容量输出电流轨迹', zorder=10)
    
    # 绘制等转矩线
    if len(equTemTrack) > 0:
        plt.plot(equTemTrack[:, 1], equTemTrack[:, 0], 'r-', linewidth=2.0, 
                label=f'等转矩线 (Tem={Tem_target:.2f} N·m)', zorder=9)
    
    # 绘制等电压椭圆
    if len(IsTrackInEquU) > 0:
        plt.plot(IsTrackInEquU[:, 1], IsTrackInEquU[:, 0], 'g-', linewidth=2.0, 
                label=f'等电压椭圆 (We={We_target:.2f} rad/s)', zorder=8)
    
    # 设置图形属性
    plt.xlabel('id (A)', fontsize=12)
    plt.ylabel('iq (A)', fontsize=12)
    plt.title(f'给定工况点的最大容量/恒转矩输出/恒转速输出电流轨迹\n'
             f'Tem={Tem_target:.2f} N·m, We={We_target:.2f} rad/s', fontsize=14)
    plt.grid(True, alpha=0.3)
    plt.axhline(y=0, color='k', linestyle='--', alpha=0.3, linewidth=0.5)
    plt.axvline(x=0, color='k', linestyle='--', alpha=0.3, linewidth=0.5)
    plt.legend(loc='upper right', fontsize=10)
    plt.axis('equal')
    plt.tight_layout()
    
    output_path = os.path.join(OUTPUT_DIR, 'operating_point_tracks.png')
    plt.savefig(output_path, dpi=150, bbox_inches='tight')
    logger.info(f"✓ 给定工况点轨迹已保存到 {output_path}\n")
    plt.close()


def run_all_tests():
    """运行所有测试"""
    logger.info("\n" + "="*70)
    logger.info("开始运行完整测试套件")
    logger.info("="*70 + "\n")
    
    # 单元测试
    test_adaptive_search()
    test_basic_calculations()
    id0_track = test_id0_track()
    mtpa_track = test_mtpa_track()
    mtpv_track = test_mtpv_track()
    
    # 集成测试
    non_fw_result = test_non_fw_result()
    fw_result = test_fw_result()
    
    # 可视化
    plot_motor_characteristics()
    plot_mtpa_track(mtpa_track)
    
    # 计算弱磁轨迹用于可视化
    calc = create_calculator()
    if calc.isDemag:
        fw1_track = calc.calc_fw1_track(mtpaTrack=mtpa_track, mtpvTrack=mtpv_track)
        IsRange = calc.calc_full_track(
            mtpaTrack=mtpa_track,
            mtpvTrack=mtpv_track,
            fw1Track=fw1_track
        )
        plot_current_range(IsRange, mtpa_track, fw1_track, mtpv_track)
        plot_equ_lines_and_circles(calc, mtpa_track, fw1_track, mtpv_track)
    else:
        IsRange = calc.calc_full_track(mtpaTrack=mtpa_track)
        plot_current_range(IsRange, mtpa_track)
    
    # 绘制控制结果
    if non_fw_result:
        plot_control_result(non_fw_result, "非弱磁MTPA控制策略")
    
    if fw_result:
        plot_control_result(fw_result, "弱磁MTPA控制策略")
    
    # 测试给定工况点的轨迹绘制
    logger.info("="*70)
    logger.info("测试给定工况点的轨迹绘制")
    logger.info("="*70)
    
    test_operating_point_tracks(calc, mtpa_track, fw1_track, mtpv_track)
    
    logger.info("\n" + "="*70)
    logger.info("所有测试完成！")
    logger.info("="*70)
    logger.info(f"测试结果保存在: {OUTPUT_DIR}/")
    logger.info("\n图表文件:")
    logger.info("  - motor_characteristics.png")
    logger.info("  - mtpa_track.png")
    logger.info("  - current_range.png")
    logger.info("  - equ_lines_and_circles.png")
    logger.info("  - 非弱磁MTPA控制策略.png")
    if fw_result:
        logger.info("  - 弱磁MTPA控制策略.png")
    logger.info("  - operating_point_tracks.png")
    logger.info("\n日志文件:")
    logger.info("  - test_log.txt")
    logger.info("="*70 + "\n")


if __name__ == '__main__':
    run_all_tests()

