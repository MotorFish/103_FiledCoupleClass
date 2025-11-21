#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
永磁同步电机场路耦合性能计算类

本模块提供了PMSM（永磁同步电机）场路耦合性能计算的完整实现，包括：
- 电压/电流限幅计算
- 电磁转矩计算
- 电角速度计算
- MTPA（最大转矩/电流比）控制轨迹计算
- MTPV（最大转矩/功率比）控制轨迹计算
- 等转矩线计算
- 等电压椭圆计算
- T-n（转矩-转速）曲线计算

使用示例：
    >>> import numpy as np
    >>> from pmsm_fcc_calculator import PMSMFieldCoupledCalculator
    >>> 
    >>> # 准备电机参数
    >>> id_range = np.linspace(-300, 100, 61)
    >>> iq_range = np.linspace(-300, 300, 71)
    >>> PsiR_vs_id = np.column_stack([id_range, 1.0 - 2.0e-4 * id_range - 2.5e-7 * id_range**2])
    >>> Lad_vs_id = np.column_stack([id_range, 0.006 + (-1.5e-8) * (id_range + 133.33)**2])
    >>> Laq_vs_iq = np.column_stack([iq_range, 9e-3 - 8.0e-9 * iq_range**2])
    >>> 
    >>> # 方式1: 使用永磁磁链PsiR创建计算器实例
    >>> calculator = PMSMFieldCoupledCalculator(
    ...     Lad_vs_id=Lad_vs_id,
    ...     Laq_vs_iq=Laq_vs_iq,
    ...     Udc=540.0,
    ...     Imax=300.0,
    ...     connectType='Y',
    ...     modulationType='SVPWM',
    ...     polePairs=4,
    ...     Lsigma=0.015e-3,
    ...     Rs=0.02,
    ...     PsiR_vs_id=PsiR_vs_id
    ... )
    >>> 
    >>> # 方式2: 使用d轴总磁链PsiD创建计算器（会自动转换为PsiR）
    >>> PsiD_vs_id = np.column_stack([id_range, 0.5 + 0.006 * id_range])  # 示例数据
    >>> calculator = PMSMFieldCoupledCalculator(
    ...     Lad_vs_id=Lad_vs_id,
    ...     Laq_vs_iq=Laq_vs_iq,
    ...     Udc=540.0,
    ...     Imax=300.0,
    ...     connectType='Y',
    ...     modulationType='SVPWM',
    ...     polePairs=4,
    ...     Lsigma=0.015e-3,
    ...     Rs=0.02,
    ...     PsiD_vs_id=PsiD_vs_id
    ... )
    >>> 
    >>> # 计算MTPA轨迹
    >>> mtpa_track = calculator.calc_mtpa_track()
    >>> 
    >>> # 生成弱磁控制结果
    >>> result = calculator.generate_fw_result(
    ...     controlType='mtpa',
    ...     temTarget=100.0,
    ...     weTarget=500.0,
    ...     loadType='constTem'
    ... )
"""

import logging
from typing import Tuple, Optional, Dict, Literal
import numpy as np
from scipy.interpolate import CubicSpline, interp1d


class PMSMFieldCoupledCalculator:
    """
    永磁同步电机场路耦合性能计算器
    
    本类封装了PMSM性能计算的所有功能，包括基础电机参数计算、
    控制策略轨迹生成等。
    
    Attributes:
        PsiR_vs_id: 磁链-d轴电流曲线 (N×2)
        Lad_vs_id: d轴激磁电感-d轴电流曲线 (N×2)
        Laq_vs_iq: q轴激磁电感-q轴电流曲线 (N×2)
        Udc: 变频器母线电压 [V]
        Imax: 变频器限幅电流(线电流幅值) [A]
        connectType: 绕组连接方式 ('Y' 或 'D')
        modulationType: 调制方式 ('SVPWM' 或 'SPWM')
        polePairs: 极对数
        Lsigma: 定子漏抗 [H]
        Rs: 定子相电阻 [Ω]
    """
    
    def __init__(
        self,
        Lad_vs_id: np.ndarray,
        Laq_vs_iq: np.ndarray,
        Udc: float,
        Imax: float,
        connectType: Literal['Y', 'D'],
        modulationType: Literal['SVPWM', 'SPWM'],
        polePairs: int,
        Lsigma: float,
        Rs: float,
        PsiR_vs_id: Optional[np.ndarray] = None,
        PsiD_vs_id: Optional[np.ndarray] = None,
        # 计算常量配置（可选）
        id0Points: int = 200,
        mtpaPoints: int = 200,
        idPointsAtICircle: int = 1000,
        deMagidPoints: int = 10,
        deMagidTol: float = 1e-5,
        deMagidMaxIter: int = 10,
        equUPoints: int = 2000,
        mtpvWeScanRatio: float = 20,
        mtpvWePoints: int = 200,
        fw1iqPoints: int = 200,
        equTemIsPoints: int = 300,
        equTemPoints: int = 100,
        equTemTracksNum: int = 30,
        constTemPointsAtNonFW: int = 100,
        constWePointsAtNonFW: int = 100,
        constTemPointsAtFW: int = 200,
        constWePointsAtFW: int = 200,
        IsSmoothInterp: bool = True,
        logger: Optional[logging.Logger] = None
    ):
        """
        初始化PMSM场路耦合计算器
        
        Args:
            Lad_vs_id: d轴激磁电感-d轴电流曲线，第1列为id，第2列为Ld
            Laq_vs_iq: q轴激磁电感-q轴电流曲线，第1列为iq，第2列为Lq
            Udc: 变频器母线电压 [V]
            Imax: 变频器限幅电流(线电流幅值) [A]
            connectType: 绕组连接方式，取值: 'Y' 或 'D'
            modulationType: 调制方式，取值: 'SVPWM' 或 'SPWM'
            polePairs: 极对数
            Lsigma: 定子漏抗 [H]
            Rs: 定子相电阻 [Ω]
            PsiR_vs_id: 永磁磁链-d轴电流曲线，第1列为id，第2列为PsiR（可选，与PsiD_vs_id二选一）
            PsiD_vs_id: d轴总磁链-d轴电流曲线，第1列为id，第2列为PsiD（可选，与PsiR_vs_id二选一）
            id0Points: id=0控制策略iq扫描点数
            mtpaPoints: MTPA曲线扫描点数
            idPointsAtICircle: 定子电流圆id扫描点数
            deMagidPoints: 去磁电流迭代初始等分数
            deMagidTol: d轴电流迭代允许误差
            deMagidMaxIter: d轴电流最大迭代次数
            equUPoints: 等电压椭圆iq扫描点数
            mtpvWeScanRatio: MTPV转速扫描倍数(相对基速，电角速度)
            mtpvWePoints: MTPV转速扫描范围等分点数
            fw1iqPoints: 弱磁I区iq扫描点数
            equTemIsPoints: 指定转矩，计算id、iq的初始等分数
            equTemPoints: 等转矩线扫描点数
            equTemTracksNum: 等转矩线数量
            constTemPointsAtNonFW: 非弱磁区的恒转矩点数
            constWePointsAtNonFW: 非弱磁区的恒电角速度点数
            constTemPointsAtFW: 弱磁区的恒转矩点数
            constWePointsAtFW: 弱磁区的恒电角速度点数
            IsSmoothInterp: 是否使用三次插值
            logger: 自定义日志记录器
        """
        # 配置日志
        self.logger = logger if logger is not None else logging.getLogger(__name__)
        
        # 验证：必须传入 PsiR_vs_id 或 PsiD_vs_id 之一（且仅一个）
        if (PsiR_vs_id is None) == (PsiD_vs_id is None):
            raise ValueError("必须传入 PsiR_vs_id 或 PsiD_vs_id 之一（且仅一个）")
        
        # 如果传入 PsiD_vs_id，转换为 PsiR_vs_id
        if PsiD_vs_id is not None:
            self.logger.info("检测到输入为 PsiD_vs_id，开始转换为 PsiR_vs_id")
            PsiR_vs_id = self._convert_PsiD_to_PsiR(PsiD_vs_id, Lad_vs_id)
            self.logger.info(f"转换完成，PsiR_vs_id 点数: {len(PsiR_vs_id)}")
        
        # 保存电机参数
        self.PsiR_vs_id = PsiR_vs_id
        self.Lad_vs_id = Lad_vs_id
        self.Laq_vs_iq = Laq_vs_iq
        self.Udc = Udc
        self.Imax = Imax
        self.connectType = connectType
        self.modulationType = modulationType
        self.polePairs = polePairs
        self.Lsigma = Lsigma
        self.Rs = Rs
        self.We_base = None
        
        # 保存计算常量
        self.id0Points = id0Points
        self.mtpaPoints = mtpaPoints
        self.idPointsAtICircle = idPointsAtICircle
        self.deMagidPoints = deMagidPoints
        self.deMagidTol = deMagidTol
        self.deMagidMaxIter = deMagidMaxIter
        self.equUPoints = equUPoints
        self.mtpvWeScanRatio = mtpvWeScanRatio
        self.mtpvWePoints = mtpvWePoints
        self.fw1iqPoints = fw1iqPoints
        self.equTemIsPoints = equTemIsPoints
        self.equTemPoints = equTemPoints
        self.equTemTracksNum = equTemTracksNum
        self.constTemPointsAtNonFW = constTemPointsAtNonFW
        self.constWePointsAtNonFW = constWePointsAtNonFW
        self.constTemPointsAtFW = constTemPointsAtFW
        self.constWePointsAtFW = constWePointsAtFW
        self.IsSmoothInterp = IsSmoothInterp
        
        self.logger.info("初始化PMSM场路耦合计算器")
        
        # 预计算电压/电流限幅
        self.UPmax, self.IPmax = self._calc_vi_limit()
        self.logger.info(f"电压/电流限幅: UPmax={self.UPmax:.2f} V, IPmax={self.IPmax:.2f} A")
        
        # 创建插值函数
        self._create_interp_functions()
        self.logger.debug("插值函数创建完成")
        
        # 预计算去磁电流
        self.isDemag, self.id_demag = self._check_and_calc_demag_id()
        if self.isDemag:
            self.logger.info(f"发生完全退磁，去磁电流: id_demag={self.id_demag:.2f} A")
        else:
            self.logger.info("未发生完全退磁")
    
    def _create_interp_functions(self):
        """创建插值函数（线性和三次样条）"""
        # 线性插值器
        self._Lad_interp_linear = interp1d(
            self.Lad_vs_id[:, 0], self.Lad_vs_id[:, 1],
            kind='linear', fill_value='extrapolate'
        )
        self._Laq_interp_linear = interp1d(
            self.Laq_vs_iq[:, 0], self.Laq_vs_iq[:, 1],
            kind='linear', fill_value='extrapolate'
        )
        self._PsiR_interp_linear = interp1d(
            self.PsiR_vs_id[:, 0], self.PsiR_vs_id[:, 1],
            kind='linear', fill_value='extrapolate'
        )
        
        # 三次样条插值器
        self._Lad_interp_cubic = CubicSpline(
            self.Lad_vs_id[:, 0], self.Lad_vs_id[:, 1], bc_type='natural'
        )
        self._Laq_interp_cubic = CubicSpline(
            self.Laq_vs_iq[:, 0], self.Laq_vs_iq[:, 1], bc_type='natural'
        )
        self._PsiR_interp_cubic = CubicSpline(
            self.PsiR_vs_id[:, 0], self.PsiR_vs_id[:, 1], bc_type='natural'
        )
    
    # ========================================
    # 第一部分：通用计算方法（私有）
    # ========================================
    
    def _convert_PsiD_to_PsiR(
        self,
        PsiD_vs_id: np.ndarray,
        Lad_vs_id: np.ndarray
    ) -> np.ndarray:
        """
        将d轴总磁链PsiD转换为永磁磁链PsiR
        
        转换公式: PsiR = PsiD - Ld * id
        
        Args:
            PsiD_vs_id: d轴总磁链-d轴电流曲线 (N×2)，第1列为id，第2列为PsiD
            Lad_vs_id: d轴激磁电感-d轴电流曲线 (N×2)，第1列为id，第2列为Ld
        
        Returns:
            PsiR_vs_id: 永磁磁链-d轴电流曲线 (M×2)，第1列为id，第2列为PsiR
                       注意：如果某些id点在Lad_vs_id范围外，会被跳过，因此M可能<N
        """
        # 提取Lad_vs_id的范围
        id_Ld_min = Lad_vs_id[:, 0].min()
        id_Ld_max = Lad_vs_id[:, 0].max()
        
        # 创建插值函数（使用线性插值更稳定）
        Ld_interp = interp1d(
            Lad_vs_id[:, 0], 
            Lad_vs_id[:, 1], 
            kind='linear',
            fill_value='extrapolate'
        )
        
        PsiR_list = []
        skipped_count = 0
        
        for i in range(len(PsiD_vs_id)):
            id_val = PsiD_vs_id[i, 0]
            PsiD_val = PsiD_vs_id[i, 1]
            
            # 检查id是否在Lad_vs_id的范围内
            if id_val < id_Ld_min or id_val > id_Ld_max:
                self.logger.warning(
                    f"id={id_val:.2f} A 超出Lad_vs_id范围 [{id_Ld_min:.2f}, {id_Ld_max:.2f}]，跳过此点"
                )
                skipped_count += 1
                continue
            
            # 插值获取对应的Ld
            Ld_val = float(Ld_interp(id_val))
            
            # 计算 PsiR = PsiD - Ld * id
            PsiR_val = PsiD_val - Ld_val * id_val
            
            PsiR_list.append([id_val, PsiR_val])
        
        if skipped_count > 0:
            self.logger.info(f"PsiD转PsiR: 跳过 {skipped_count} 个点，保留 {len(PsiR_list)} 个点")
        
        if len(PsiR_list) == 0:
            raise ValueError("PsiD_vs_id中没有任何点在Lad_vs_id范围内，无法转换")
        
        return np.array(PsiR_list)
    
    def _interp_sorted(
        self,
        x_new: float,
        x_data: np.ndarray,
        y_data: np.ndarray,
        smooth: Optional[bool] = None
    ) -> float:
        """
        对数据按x排序后再插值
        
        Args:
            x_new: 要插值的新x值
            x_data: 原始x数据（可以是无序的）
            y_data: 原始y数据
            smooth: 是否使用光滑插值（三次样条）。None时使用类默认配置
        
        Returns:
            插值结果
        """
        if smooth is None:
            smooth = self.IsSmoothInterp
        
        # 按x排序
        sorted_indices = np.argsort(x_data)
        x_sorted = x_data[sorted_indices]
        y_sorted = y_data[sorted_indices]
        
        # 插值
        if smooth:
            cs = CubicSpline(x_sorted, y_sorted, bc_type='natural')
            result = cs(x_new)
        else:
            result = np.interp(x_new, x_sorted, y_sorted)
        
        # 转换为标量
        if isinstance(result, np.ndarray) and result.ndim == 0:
            return float(result)
        return result
    
    def _adaptive_step_search(
        self,
        f,
        bracket: Tuple[float, float],
        divisions: int = 100,
        startFrom: Literal['left', 'right'] = 'left',
        target: float = 0,
        tol: float = 1e-5,
        maxIter: int = 100
    ) -> Tuple[float, float, int, bool]:
        """
        自适应步长单侧搜索算法
        
        在指定闭区间内从一端开始单侧搜索。当搜索方向一致时保持步长，
        方向改变时自动缩小步长（0.1倍），实现快速收敛。
        
        Args:
            f: 目标函数
            bracket: 搜索区间 [a, b]（闭区间）
            divisions: 初始分段数
            startFrom: 起始方向，'left'从左向右，'right'从右向左
            target: 目标值
            tol: 收敛容差
            maxIter: 最大迭代次数
        
        Returns:
            (x, fx, iterations, converged): 解、函数值、迭代次数、是否收敛
        """
        a, b = bracket
        if a >= b:
            raise ValueError(f"区间错误: bracket[0] ({a}) 必须小于 bracket[1] ({b})")
        
        if startFrom == 'left':
            x = a
            step = abs(b - a) / divisions
        elif startFrom == 'right':
            x = b
            step = -abs(b - a) / divisions
        else:
            raise ValueError(f"startFrom参数错误: 只能为 'left' 或 'right'")
        
        lastError = None
        lastX = None
        
        for i in range(maxIter):
            fx = f(x)
            error = fx - target
            
            if abs(error) < tol:
                self.logger.debug(f"搜索收敛: x={x:.6f}, fx={fx:.6f}, 迭代{i+1}次")
                return x, fx, i + 1, True
            
            if lastError is not None and error * lastError < 0:
                x = lastX
                step *= 0.1
            
            lastX = x
            lastError = error
            x += step
        
        self.logger.warning(f"搜索未收敛: 达到最大迭代次数{maxIter}，误差={error:.6e}")
        return x, f(x), maxIter, False
    
    def _adaptive_step_search_maximum(
        self,
        f,
        bracket: Tuple[float, float],
        divisions: int = 1000,
        startFrom: Literal['left', 'right'] = 'right',
        tol: float = 1e-5,
        maxIter: int = 200
    ) -> Tuple[float, float, int, bool]:
        """
        自适应步长搜索局部极大值
        
        Args:
            f: 目标函数（需要求极大值）
            bracket: 搜索区间 [a, b]
            divisions: 初始分段数
            startFrom: 起始方向
            tol: 收敛容差
            maxIter: 最大迭代次数
        
        Returns:
            (x, fx, iterations, converged): 极值点、函数值、迭代次数、是否收敛
        """
        a, b = bracket
        if a >= b:
            raise ValueError(f"区间错误: bracket[0] ({a}) 必须小于 bracket[1] ({b})")
        
        if startFrom == 'left':
            x = a
            step = abs(b - a) / divisions
        elif startFrom == 'right':
            x = b
            step = -abs(b - a) / divisions
        else:
            raise ValueError(f"startFrom参数错误")
        
        lastFx = f(x)
        lastX = x
        
        for i in range(maxIter):
            if abs(step) < tol * abs(b - a):
                self.logger.debug(f"极值搜索收敛: x={lastX:.6f}, f(x)={lastFx:.6f}")
                return lastX, lastFx, i + 1, True
            
            x_new = lastX + step
            
            if startFrom == 'left' and x_new > b:
                x_new = b
            elif startFrom == 'right' and x_new < a:
                x_new = a
            
            fx_new = f(x_new)
            
            if fx_new < lastFx:
                lastX = lastX - step
                step = step * 0.1
            else:
                lastX = x_new
                lastFx = fx_new
                
                if (startFrom == 'left' and x >= b) or (startFrom == 'right' and x <= a):
                    return lastX, lastFx, i + 1, True
        
        return lastX, lastFx, maxIter, False
    
    def _calc_vi_limit(self) -> Tuple[float, float]:
        """
        计算电压/电流限幅
        
        Returns:
            (UPmax, IPmax): 最大相电压幅值和最大相电流幅值
        """
        if self.modulationType == 'SVPWM':
            ULmax = self.Udc
        elif self.modulationType == 'SPWM':
            ULmax = self.Udc / 2.0
        else:
            raise ValueError(f"不支持的调制方式: {self.modulationType}")
        
        if self.connectType == 'Y':
            UPmax = ULmax / np.sqrt(3.0)
        elif self.connectType == 'D':
            UPmax = ULmax
        else:
            raise ValueError(f"不支持的连接方式: {self.connectType}")
        
        ILmax = self.Imax
        if self.connectType == 'Y':
            IPmax = ILmax
        elif self.connectType == 'D':
            IPmax = ILmax * np.sqrt(2.0)
        
        return UPmax, IPmax
    
    def _check_and_calc_demag_id(self) -> Tuple[bool, float]:
        """
        校核并计算去磁电流
        
        Returns:
            (isDemag, id_demag): 是否发生完全退磁，去磁电流
        """
        PsiR = self._calc_PsiR(-self.IPmax)
        Lad = self._calc_Lad(-self.IPmax)
        
        if PsiR + Lad * (-self.IPmax) < 0:
            self.logger.debug("检测到完全退磁，计算去磁电流")
            
            def PsiD_pu(id):
                Lad = self._calc_Lad(id)
                PsiR = self._calc_PsiR(id)
                PsiD = Lad * id + PsiR
                PsiD_pu = PsiD / PsiR
                return PsiD_pu
            
            id_demag, _, _, _ = self._adaptive_step_search(
                PsiD_pu,
                [-self.IPmax, 0],
                divisions=self.deMagidPoints,
                startFrom='right',
                target=0,
                tol=1e-5
            )
            return True, id_demag
        else:
            return False, 0.0
    
    # ========================================
    # 第二部分：电机性能计算（私有）
    # ========================================
    
    def _calc_Lad(self, id: float, smooth: Optional[bool] = None) -> float:
        """计算d轴电枢反应电感"""
        if smooth is None:
            smooth = self.IsSmoothInterp
        
        if smooth:
            Lad = self._Lad_interp_cubic(id)
        else:
            Lad = self._Lad_interp_linear(id)
        return float(Lad)
    
    def _calc_Laq(self, iq: float, smooth: Optional[bool] = None) -> float:
        """计算q轴电枢反应电感"""
        if smooth is None:
            smooth = self.IsSmoothInterp
        
        if smooth:
            Laq = self._Laq_interp_cubic(iq)
        else:
            Laq = self._Laq_interp_linear(iq)
        return float(Laq)
    
    def _calc_PsiR(self, id: float, smooth: Optional[bool] = None) -> float:
        """计算永磁体磁链"""
        if smooth is None:
            smooth = self.IsSmoothInterp
        
        if smooth:
            PsiR = self._PsiR_interp_cubic(id)
        else:
            PsiR = self._PsiR_interp_linear(id)
        return float(PsiR)
    
    def _calc_Ld(self, id: float, smooth: Optional[bool] = None) -> float:
        """计算d轴电感"""
        Lad = self._calc_Lad(id, smooth=smooth)
        Ld = Lad + self.Lsigma
        return Ld
    
    def _calc_Lq(self, iq: float, smooth: Optional[bool] = None) -> float:
        """计算q轴电感"""
        Laq = self._calc_Laq(iq, smooth=smooth)
        Lq = Laq + self.Lsigma
        return Lq
    
    def _calc_Tem(self, id: float, iq: float) -> float:
        """
        计算电磁转矩
        
        Args:
            id: d轴电流 [A]
            iq: q轴电流 [A]
        
        Returns:
            Tem: 电磁转矩 [N·m]
        """
        Lad = self._calc_Lad(id)
        Laq = self._calc_Laq(iq)
        PsiR = self._calc_PsiR(id)
        Tem = 1.5 * self.polePairs * (PsiR * iq + (Lad - Laq) * id * iq)
        return Tem
    
    def _calc_Up(self, id: float, iq: float, we: float) -> float:
        """
        计算定子相电压幅值
        
        Args:
            id: d轴电流 [A]
            iq: q轴电流 [A]
            we: 电角速度 [rad/s]
        
        Returns:
            Up: 定子电压幅值 [V]
        """
        Ld = self._calc_Ld(id)
        Lq = self._calc_Lq(iq)
        phi_f = self._calc_PsiR(id)
        
        Ud = -we * Lq * iq + self.Rs * id
        Uq = we * (phi_f + Ld * id) + self.Rs * iq
        
        Up = np.sqrt(Ud**2 + Uq**2)
        return Up
    
    def _calc_We(self, id: float, iq: float, UP: float) -> Tuple[bool, str, float]:
        """
        给定dq电流和相电压幅值，计算电角速度
        
        Args:
            id: d轴电流 [A]
            iq: q轴电流 [A]
            UP: 相电压幅值 [V]
        
        Returns:
            (success, error_msg, We): 是否成功、错误信息、电角速度 [rad/s]
        """
        Ld = self._calc_Ld(id)
        Lq = self._calc_Lq(iq)
        PsiR = self._calc_PsiR(id)
        
        # 一元二次方程系数
        a = (Lq * iq)**2 + (PsiR + Ld * id)**2
        b = 2 * self.Rs * iq * (PsiR + Ld * id - id * Lq)
        c = (self.Rs * iq)**2 + (self.Rs * id)**2 - UP**2
        
        delta = b**2 - 4*a*c
        
        if delta < 0:
            return False, "方程无实数解", 0.0
        else:
            We = (-b + np.sqrt(delta)) / (2*a)
            return True, "", We
    
    def _calc_id(
        self,
        iq: float,
        We: float,
        Us: float,
        id_init: float = -100.0,
        maxIter: int = 50,
        tol: float = 1e-5,
        root: Literal['L', 'R'] = 'L'
    ) -> Tuple[bool, str, float]:
        """
        给定q轴电流、电角速度、相电压幅值，迭代计算d轴电流
        
        Args:
            iq: q轴电流 [A]
            We: 电角速度 [rad/s]
            Us: 相电压幅值 [V]
            id_init: d轴电流迭代初始值 [A]
            maxIter: 最大迭代次数
            tol: 收敛容差
            root: 'L'取左根（较小的/负根），'R'取右根（较大的/正根）
        
        Returns:
            (success, error_msg, id): 是否成功、错误信息、d轴电流
        """
        id = id_init
        Lq = self._calc_Lq(iq)
        
        for i in range(maxIter):
            Ld = self._calc_Ld(id)
            PsiR = self._calc_PsiR(id)
            
            a = self.Rs**2 + We**2 * Ld**2
            b = 2 * We * (Ld * (We * PsiR + self.Rs * iq) - self.Rs * Lq * iq)
            c = (We * Lq * iq)**2 + (We * PsiR + self.Rs * iq)**2 - Us**2
            
            delta = b**2 - 4 * a * c
            if delta < 0:
                return False, "delta<0", 0.0
            
            sqrt_delta = np.sqrt(delta)
            if root.upper() == 'L':
                id_new = (-b - sqrt_delta) / (2 * a)
            elif root.upper() == 'R':
                id_new = (-b + sqrt_delta) / (2 * a)
            else:
                return False, "Unknown root param", 0.0
            
            if abs(id_new - id) / (abs(id) + 1e-8) < tol:
                self.logger.debug(f"id计算收敛: id={id_new:.4f}, 迭代{i+1}次")
                return True, "", id_new
            
            damping = 0.5
            id = id + damping * (id_new - id)
        
        self.logger.warning(f"id计算未收敛: 达到最大迭代次数{maxIter}")
        return False, "maxIter", id
    
    def _calc_iq(
        self,
        id: float,
        We: float,
        Us: float,
        iq_init: float = 0,
        maxIter: int = 50,
        tol: float = 1e-6
    ) -> Tuple[bool, str, float]:
        """
        给定d轴电流、电角速度、相电压幅值，计算q轴电流
        
        Args:
            id: d轴电流 [A]
            We: 电角速度 [rad/s]
            Us: 相电压幅值 [V]
            iq_init: q轴电流迭代初始值 [A]
            maxIter: 最大迭代次数
            tol: 收敛容差
        
        Returns:
            (success, error_msg, iq): 是否成功、错误信息、q轴电流
        """
        iq = iq_init
        Ld = self._calc_Ld(id)
        PsiR = self._calc_PsiR(id)
        
        for i in range(maxIter):
            Lq = self._calc_Lq(iq)
            
            a = We**2 * Lq**2 + self.Rs**2
            b = 2 * We * self.Rs * (PsiR + (Ld - Lq) * id)
            c = self.Rs**2 * id**2 + We**2 * (PsiR + Ld * id)**2 - Us**2
            
            delta = b**2 - 4 * a * c
            
            if delta < 0:
                return False, f"判别式为负（Δ={delta:.4e}）", 0.0
            
            iq_new = (-b + np.sqrt(delta)) / (2 * a)
            
            if abs(iq_new - iq)/max(abs(iq), 1e-8) < tol:
                self.logger.debug(f"iq计算收敛: iq={iq_new:.4f}, 迭代{i+1}次")
                return True, "", iq_new
            
            damping = 0.5
            iq = iq + damping * (iq_new - iq)
        
        self.logger.warning(f"iq计算未收敛: 达到最大迭代次数{maxIter}")
        return False, "maxIter", iq
    
    def _calc_id_by_Tem_iq(
        self,
        Tem_target: float,
        iq: float,
        id_min: float,
        id_init: float = 0.0,
        divisions: int = 10000,
        maxIter: int = 1000
    ) -> float:
        """
        指定Tem、iq，计算对应的id
        
        Args:
            Tem_target: 目标电磁转矩 [N·m]
            iq: q轴电流 [A]
            id_min: 最小id [A]
            id_init: 初始id [A]
            divisions: 搜索分段数
            maxIter: 最大迭代次数
        
        Returns:
            id: 对应的d轴电流 [A]
        """
        id, _, iterations, converged = self._adaptive_step_search(
            f=lambda id: self._calc_Tem(id, iq) - Tem_target,
            bracket=[id_min, id_init],
            divisions=divisions,
            startFrom='right',
            target=0,
            tol=1e-5*abs(Tem_target),
            maxIter=maxIter
        )
        return id
    
    def _calc_iq_by_Tem_id(
        self,
        Tem_target: float,
        id: float,
        iq_init: float,
        iq_min: float = 0.0,
        divisions: int = 10000,
        maxIter: int = 1000
    ) -> float:
        """
        指定Tem、id，计算对应的iq
        
        Args:
            Tem_target: 目标电磁转矩 [N·m]
            id: d轴电流 [A]
            iq_init: 初始iq [A]
            iq_min: 最小iq [A]
            divisions: 搜索分段数
            maxIter: 最大迭代次数
        
        Returns:
            iq: 对应的q轴电流 [A]
        """
        iq, _, iterations, converged = self._adaptive_step_search(
            f=lambda iq: self._calc_Tem(id, iq) - Tem_target,
            bracket=[iq_min, iq_init],
            divisions=divisions,
            startFrom='right',
            target=0,
            tol=1e-5*abs(Tem_target),
            maxIter=maxIter
        )
        return iq
    
    # ========================================
    # 第三部分：控制策略轨迹计算（公开）
    # ========================================
    
    def calc_id0_track(self) -> np.ndarray:
        """
        计算id=0时的电流轨迹
        
        Returns:
            id0Track: id=0电流轨迹 (N×5)，格式：[[iq, id, Is, Tem, maxWe], ...]
        """
        self.logger.info(f"开始计算id=0电流轨迹，扫描点数={self.id0Points}")
        
        id0Track = []
        
        for i in range(self.id0Points + 1):
            iq = self.IPmax * i / self.id0Points
            id = 0.0
            Is = np.sqrt(id**2 + iq**2)
            Tem = self._calc_Tem(id=id, iq=iq)
            
            success, error_msg, maxWe = self._calc_We(id=id, iq=iq, UP=self.UPmax)
            
            if not success:
                self.logger.debug(f"id=0轨迹点{i+1}计算失败: iq={iq:.2f}, {error_msg}")
                maxWe = 0.0
            
            id0Track.append([iq, id, Is, Tem, maxWe])
        
        id0Track = np.array(id0Track)
        if self.We_base is None:
            self.We_base = id0Track[-1, 4]
            self.logger.info(f"基速自动获取为{self.We_base:.2f} rad/s")
        self.logger.info(f"id=0电流轨迹计算完成，轨迹点数={len(id0Track)}")
        
        return id0Track
    
    def calc_mtpa_track(self) -> np.ndarray:
        """
        计算MTPA电流轨迹
        
        Returns:
            mtpaTrack: MTPA电流轨迹 (N×5)，格式：[[iq, id, Is, Tem, maxWe], ...]
        """
        self.logger.info(f"开始计算MTPA电流轨迹，扫描点数={self.mtpaPoints}")
        
        mtpaTrack = []
        last_id_max = None
        
        for i in range(self.mtpaPoints + 1):
            I_temp = self.IPmax * i / self.mtpaPoints
            
            if I_temp < 1e-6:
                maxWe = self._calc_We(id=0.0, iq=0.0, UP=self.UPmax)[2]
                mtpaTrack.append([0.0, 0.0, 0.0, 0.0, maxWe])
                last_id_max = 0.0
                continue
            
            def temFunc(id):
                iq = np.sqrt(I_temp**2 - id**2)
                return self._calc_Tem(id, iq)
            
            if last_id_max is None:
                bracket_right = 0.0
            else:
                bracket_right = last_id_max
            
            id_max, T_max, iterations, converged = self._adaptive_step_search_maximum(
                f=temFunc,
                bracket=[-I_temp, bracket_right],
                divisions=1000,
                startFrom='right',
                tol=1e-4,
                maxIter=200
            )
            
            iq_max = np.sqrt(I_temp**2 - id_max**2)
            maxWe = self._calc_We(id=id_max, iq=iq_max, UP=self.UPmax)[2]
            
            mtpaTrack.append([iq_max, id_max, I_temp, T_max, maxWe])
            last_id_max = id_max
        
        mtpaTrack = np.array(mtpaTrack)
        if self.We_base is None:
            self.We_base = mtpaTrack[-1, 4]
            self.logger.info(f"基速自动获取为{self.We_base:.2f} rad/s")
        self.logger.info(f"MTPA电流轨迹计算完成，轨迹点数={len(mtpaTrack)}, 最大转矩={mtpaTrack[-1, 3]:.2f} N·m")
        
        return mtpaTrack
    
    def calc_mtpv_track(self, nonFWTrack: np.ndarray) -> np.ndarray:
        """
        计算MTPV(最大转矩/功率比)电流轨迹
        
        Args:
            nonFWTrack: 基础轨迹（id0或MTPA轨迹），用于获取基速
                      格式：(N×5)，[[iq, id, Is, Tem, We], ...]
                      基速从轨迹最后一个点的We获取（序列中的最小角速度）
        
        Returns:
            mtpvTrack: MTPV电流轨迹 (N×5)，格式：[[iq, id, Is, Tem, We], ...]
        """
        # 从基础轨迹的最后一个点获取基速（最小角速度）
        We_base = self.We_base
        self.logger.info(f"开始计算MTPV电流轨迹，基速={We_base:.2f} rad/s（从基础轨迹获取）")
        
        if self.IPmax <= abs(self.id_demag):
            raise ValueError(
                f"最大电流约束IPmax={self.IPmax:.2f} A必须大于去磁电流|id_demag|={abs(self.id_demag):.2f} A"
            )
        
        We_max = We_base * self.mtpvWeScanRatio
        mtpvTrack = []
        
        id_init = self.id_demag
        iq_init = 0.0
        
        for i in range(self.mtpvWePoints + 1):
            We_current = We_max - (We_max - We_base) * i / self.mtpvWePoints
            
            def TemWithidAtMaxUs(id: float) -> float:
                nonlocal iq_init
                iq_result = self._calc_iq(id=id, We=We_current, Us=self.UPmax, iq_init=iq_init)[2]
                
                if iq_result is None or np.isnan(iq_result):
                    return -np.inf
                
                iq_init = iq_result
                Tem = self._calc_Tem(id=id, iq=iq_result)
                return Tem
            
            id_LowBound = self._calc_id(iq=0, We=We_current, Us=self.UPmax, id_init=self.id_demag, root='L')[2]
            
            id_opt, Tem_max, maxIter, success = self._adaptive_step_search_maximum(
                f=TemWithidAtMaxUs,
                bracket=[id_LowBound, id_init],
                divisions=1000,
                maxIter=100,
                tol=1e-4,
                startFrom='right'
            )
            
            iq_opt = self._calc_iq(id=id_opt, We=We_current, Us=self.UPmax, iq_init=iq_init)[2]
            
            if iq_opt is None or np.isnan(iq_opt) or Tem_max == -np.inf:
                continue
            
            Is_opt = np.sqrt(id_opt**2 + iq_opt**2)
            
            if Is_opt > self.IPmax:
                break
            
            mtpvTrack.append([iq_opt, id_opt, Is_opt, Tem_max, We_current])
            
            id_init = id_opt
            iq_init = iq_opt
        
        mtpvTrack = np.array(mtpvTrack)
        
        if len(mtpvTrack) > 0:
            mtpvTrack = np.flipud(mtpvTrack)
        
        self.logger.info(f"MTPV电流轨迹计算完成，轨迹点数={len(mtpvTrack)}")
        
        return mtpvTrack
    
    def calc_fw1_track(
        self,
        mtpaTrack: np.ndarray,
        mtpvTrack: Optional[np.ndarray] = None
    ) -> np.ndarray:
        """
        计算弱磁I区的电流轨迹
        
        Args:
            mtpaTrack: MTPA电流轨迹
            mtpvTrack: MTPV电流轨迹（可选）
        
        Returns:
            fw1Track: 弱磁I区电流轨迹 (N×5)，格式：[[iq, id, Is, Tem, We], ...]
        """
        self.logger.info(f"开始计算弱磁I区电流轨迹，扫描点数={self.fw1iqPoints}")
        
        iq_right = mtpaTrack[-1, 0]
        
        if mtpvTrack is not None:
            iq_left = mtpvTrack[0, 0]
        else:
            iq_left = 0.0
        
        fw1Track = []
        for iq in np.linspace(iq_right, iq_left, self.fw1iqPoints):
            id = -np.sqrt(max(0.0, self.IPmax**2 - iq**2))
            Is = np.sqrt(id**2 + iq**2)
            Tem = self._calc_Tem(id, iq)
            We = self._calc_We(id, iq, self.UPmax)[2]
            fw1Track.append([iq, id, Is, Tem, We])
        
        fw1Track = np.array(fw1Track)
        self.logger.info(f"弱磁I区电流轨迹计算完成，轨迹点数={len(fw1Track)}")
        
        return fw1Track
    
    def calc_full_track(
        self,
        mtpaTrack: np.ndarray,
        mtpvTrack: Optional[np.ndarray] = None,
        fw1Track: Optional[np.ndarray] = None
    ) -> np.ndarray:
        """
        计算完整的电流轨迹
        
        Args:
            mtpaTrack: MTPA轨迹
            mtpvTrack: MTPV轨迹（可选）
            fw1Track: 弱磁I区轨迹（可选）
        
        Returns:
            IsRange: 完整电流轨迹 (N×2)，格式：[[iq, id], ...]
        """
        self.logger.info("开始拼接完整电流轨迹")
        
        if mtpaTrack is None:
            raise ValueError("mtpaTrack必须提供")
        
        IsRange = []
        IsRange.extend(mtpaTrack[:, :2])
        
        if fw1Track is not None:
            IsRange.extend(fw1Track[:, :2])
        
        if mtpvTrack is not None:
            IsRange.extend(mtpvTrack[:, :2])
        
        IsRange = np.array(IsRange)
        self.logger.info(f"完整电流轨迹拼接完成，总点数={len(IsRange)}")
        
        return IsRange
    
    def calc_equ_tem_track(
        self,
        Tem_target: float,
        mtpaTrack: np.ndarray,
        fw1Track: np.ndarray,
        mtpvTrack: Optional[np.ndarray] = None
    ) -> np.ndarray:
        """
        计算等转矩线
        
        Args:
            Tem_target: 目标电磁转矩 [N·m]
            mtpaTrack: MTPA轨迹
            fw1Track: 弱磁I区轨迹
            mtpvTrack: MTPV轨迹（可选）
        
        Returns:
            equTemTrack: 等转矩轨迹 (N×5)，格式：[[iq, id, Is, Tem, We], ...]
        """
        self.logger.info(f"开始计算等转矩线，目标转矩={Tem_target:.2f} N·m")
        
        Tem_max = fw1Track[0, 3]
        Tem_splitFW = fw1Track[-1, 3]
        
        if Tem_target > Tem_max:
            raise ValueError(f"目标转矩{Tem_target:.2f} N·m大于最大转矩{Tem_max:.2f} N·m")
        
        id_mtpa = self._interp_sorted(Tem_target, mtpaTrack[:, 3], mtpaTrack[:, 1])
        iq_mtpa = self._interp_sorted(Tem_target, mtpaTrack[:, 3], mtpaTrack[:, 0])
        
        self.logger.debug(f"MTPA上插值点: iq={iq_mtpa:.2f}, id={id_mtpa:.2f}")
        
        lastPoint = []
        
        if Tem_target > Tem_splitFW or mtpvTrack is None:
            iq_fw1 = self._interp_sorted(Tem_target, fw1Track[:, 3], fw1Track[:, 0])
            id_fw1 = self._interp_sorted(Tem_target, fw1Track[:, 3], fw1Track[:, 1])
            Is_fw1 = self._interp_sorted(Tem_target, fw1Track[:, 3], fw1Track[:, 2])
            Tem_fw1 = self._interp_sorted(Tem_target, fw1Track[:, 3], fw1Track[:, 3])
            We_fw1 = self._interp_sorted(Tem_target, fw1Track[:, 3], fw1Track[:, 4])
            lastPoint = [iq_fw1, id_fw1, Is_fw1, Tem_fw1, We_fw1]
            
            self.logger.debug(f"左端点在弱磁I区: iq={iq_fw1:.2f}, id={id_fw1:.2f}")
            
            id_diff = abs(id_mtpa - id_fw1)
            iq_diff = abs(iq_mtpa - iq_fw1)
            
            if id_diff >= iq_diff:
                scan_by_id = True
                id_step = (id_mtpa - id_fw1 * 0.95) / self.equTemPoints
            else:
                scan_by_id = False
                iq_step = (iq_mtpa - iq_fw1 * 0.95) / self.equTemPoints
        else:
            minTemAtMtpv = mtpvTrack[-1, 3]
            minidAtMtpv = mtpvTrack[-1, 1]
            
            if Tem_target < minTemAtMtpv:
                iq_mtpv = 0.0
                id_mtpv = minidAtMtpv
            else:
                iq_mtpv = self._interp_sorted(Tem_target, mtpvTrack[:, 3], mtpvTrack[:, 0])
                id_mtpv = self._interp_sorted(Tem_target, mtpvTrack[:, 3], mtpvTrack[:, 1])
                Is_mtpv = self._interp_sorted(Tem_target, mtpvTrack[:, 3], mtpvTrack[:, 2])
                Tem_mtpv = self._interp_sorted(Tem_target, mtpvTrack[:, 3], mtpvTrack[:, 3])
                We_mtpv = self._interp_sorted(Tem_target, mtpvTrack[:, 3], mtpvTrack[:, 4])
                lastPoint = [iq_mtpv, id_mtpv, Is_mtpv, Tem_mtpv, We_mtpv]
            
            self.logger.debug(f"左端点在弱磁II区: iq={iq_mtpv:.2f}, id={id_mtpv:.2f}")
            
            id_diff = abs(id_mtpa - id_mtpv)
            iq_diff = abs(iq_mtpa - iq_mtpv)
            
            if id_diff >= iq_diff:
                scan_by_id = True
                id_step = (id_mtpa - id_mtpv * 0.95) / self.equTemPoints
            else:
                scan_by_id = False
                iq_step = (iq_mtpa - iq_mtpv * 0.95) / self.equTemPoints
        
        equTemTrack = []
        
        if scan_by_id:
            iq_init = iq_mtpa * 1.05
            
            for i in range(self.equTemPoints + 1):
                id = id_mtpa - i * id_step
                iq = self._calc_iq_by_Tem_id(Tem_target, id, iq_init)
                
                if iq < 0.0:
                    self.logger.debug(f"iq<0，结束等转矩线计算")
                    break
                
                Tem = self._calc_Tem(id, iq)
                if abs(Tem - Tem_target)/Tem_target > 1e-2:
                    self.logger.warning(f"转矩误差过大: 计算={Tem:.2f}, 目标={Tem_target:.2f}")
                
                Is = np.sqrt(id**2 + iq**2)
                if Is > self.IPmax:
                    self.logger.debug(f"电流超限: Is={Is:.2f} > IPmax={self.IPmax:.2f}")
                    break
                
                success, error_msg, We = self._calc_We(id, iq, self.UPmax)
                if not success:
                    self.logger.debug(f"We计算失败: {error_msg}")
                    break
                
                equTemTrack.append([iq, id, Is, Tem, We])
                iq_init = iq
        else:
            id_init = id_mtpa * 1.05
            
            for i in range(self.equTemPoints + 1):
                iq = iq_mtpa - i * iq_step
                id = self._calc_id_by_Tem_iq(Tem_target, iq, -self.IPmax, id_init)
                
                if id > -1e-6:
                    break
                
                Tem = self._calc_Tem(id, iq)
                if abs(Tem - Tem_target)/Tem_target > 1e-2:
                    self.logger.warning(f"转矩误差过大: 计算={Tem:.2f}, 目标={Tem_target:.2f}")
                
                Is = np.sqrt(id**2 + iq**2)
                if Is > self.IPmax:
                    self.logger.debug(f"电流超限: Is={Is:.2f} > IPmax={self.IPmax:.2f}")
                    break
                
                success, error_msg, We = self._calc_We(id, iq, self.UPmax)
                if not success:
                    self.logger.debug(f"We计算失败: {error_msg}")
                    break
                
                equTemTrack.append([iq, id, Is, Tem, We])
                id_init = id
        
        if lastPoint and len(lastPoint) > 0:
            equTemTrack.append(lastPoint)
        
        equTemTrack = np.array(equTemTrack)
        self.logger.info(f"等转矩线计算完成，轨迹点数={len(equTemTrack)}")
        
        return equTemTrack
    
    def calc_equ_u_track(
        self,
        We: float,
        mtpaTrack: np.ndarray,
        fw1Track: np.ndarray,
        mtpvTrack: Optional[np.ndarray] = None,
        splitRatio: int = 5
    ) -> Tuple[np.ndarray, float]:
        """
        计算指定转速下电流控制范围内的等电压椭圆轨迹
        
        Args:
            We: 电角速度 [rad/s]
            mtpaTrack: MTPA轨迹
            fw1Track: 弱磁I区轨迹
            mtpvTrack: MTPV轨迹（可选）
            splitRatio: 分段比例
        
        Returns:
            (IsTrackInEquU, Tem_max): 等电压椭圆轨迹、最大转矩
        """
        self.logger.info(f"开始计算等电压椭圆，转速={We:.2f} rad/s")
        
        # 列索引定义
        iqIdx, idIdx, IsIdx, temIdx, weIdx = 0, 1, 2, 3, 4
        
        maxWeAtMtpaTrackEnd = 0
        if len(mtpaTrack) > 0:
            id_end = mtpaTrack[-1, idIdx]
            iq_end = mtpaTrack[-1, iqIdx]
            maxWeAtMtpaTrackEnd = self._calc_We(id_end, iq_end, self.UPmax)[2]
        
        maxWeAtMtpaTrackStart = 0
        if len(mtpaTrack) > 0:
            id_start = mtpaTrack[0, idIdx]
            iq_start = mtpaTrack[0, iqIdx]
            maxWeAtMtpaTrackStart = self._calc_We(id_start, iq_start, self.UPmax)[2]
        
        self.logger.debug(f"MTPA末端最大转速={maxWeAtMtpaTrackEnd:.2f}, 起点最大转速={maxWeAtMtpaTrackStart:.2f}")
        
        if We <= maxWeAtMtpaTrackEnd:
            self.logger.info("转速过低，返回空轨迹")
            return np.array([]), 0.0
        
        # 确定起点
        if We <= maxWeAtMtpaTrackStart:
            We_mtpa = mtpaTrack[:, weIdx] if mtpaTrack.shape[1] > 4 else np.array([mtpaTrack[i, 4] for i in range(len(mtpaTrack))])
            We_mtpa = We_mtpa[::-1]
            id_start = self._interp_sorted(We, We_mtpa, mtpaTrack[::-1, idIdx])
            iq_start = self._interp_sorted(We, We_mtpa, mtpaTrack[::-1, iqIdx])
            self.logger.debug(f"起点在MTPA内: id={id_start:.2f}, iq={iq_start:.2f}")
        else:
            success_R, msg_R, id_right = self._calc_id(iq=0, We=We, Us=self.UPmax, id_init=self.id_demag, root='R')
            
            if success_R:
                id_start = id_right
            else:
                id_start = 0
            
            iq_start = 0
            self.logger.debug(f"起点在MTPA外: id={id_start:.2f}, iq={iq_start:.2f}")
        
        # 确定终点
        maxWeAtFw1TrackStart = fw1Track[-1, weIdx]
        
        if We <= maxWeAtFw1TrackStart or mtpvTrack is None:
            We_fw1 = fw1Track[:, weIdx]
            id_end = self._interp_sorted(We, We_fw1, fw1Track[:, idIdx])
            iq_end = self._interp_sorted(We, We_fw1, fw1Track[:, iqIdx])
            self.logger.debug(f"终点在弱磁I区: id={id_end:.2f}, iq={iq_end:.2f}")
        else:
            We_mtpv = mtpvTrack[:, weIdx]
            id_end = self._interp_sorted(We, We_mtpv, mtpvTrack[:, idIdx])
            iq_end = self._interp_sorted(We, We_mtpv, mtpvTrack[:, iqIdx])
            self.logger.debug(f"终点在MTPV区: id={id_end:.2f}, iq={iq_end:.2f}")
        
        # 构建等电压椭圆轨迹
        success, msg, iq_max = self._calc_iq(id=self.id_demag, We=We, Us=self.UPmax, iq_init=0)
        
        if not success or iq_max is None or np.isnan(iq_max) or iq_max <= 0:
            iq_max = 0
        
        self.logger.debug(f"等电压圆最大iq={iq_max:.2f}")
        
        IsTrackInEquU = []
        iq_threshold = iq_max / splitRatio if splitRatio > 0 else 0
        
        # 第一段
        if iq_start <= iq_threshold:
            iq_points_1 = np.linspace(iq_start, iq_threshold, self.equUPoints // 2)
            
            id_init_value = id_start
            for iq in iq_points_1:
                success, msg, id = self._calc_id(iq=iq, We=We, Us=self.UPmax, id_init=id_init_value, root='R')
                if success and id is not None and not np.isnan(id):
                    IsTrackInEquU.append([iq, id, np.sqrt(iq**2 + id**2), self._calc_Tem(id, iq), We])
                    id_init_value = id
            
            id_split = IsTrackInEquU[-1][1] if IsTrackInEquU else id_start
            iq_split = IsTrackInEquU[-1][0] if IsTrackInEquU else iq_start
        else:
            id_split = id_start
            iq_split = iq_start
        
        # 第二段
        id_points_2 = np.linspace(id_split, id_end, self.equUPoints // 2)
        
        iq_init_value = iq_split
        for id in id_points_2:
            success, msg, iq = self._calc_iq(id=id, We=We, Us=self.UPmax, iq_init=iq_init_value)
            if success and iq is not None and not np.isnan(iq):
                IsTrackInEquU.append([iq, id, np.sqrt(iq**2 + id**2), self._calc_Tem(id, iq), We])
                iq_init_value = iq
        
        IsTrackInEquU = np.array(IsTrackInEquU)
        
        # 计算最大转矩
        Tem_max = 0.0
        if len(IsTrackInEquU) > 0:
            for i in range(len(IsTrackInEquU)):
                iq_i = IsTrackInEquU[i, 0]
                id_i = IsTrackInEquU[i, 1]
                Tem_i = self._calc_Tem(id_i, iq_i)
                if Tem_i > Tem_max:
                    Tem_max = Tem_i
        
        self.logger.info(f"等电压椭圆计算完成，轨迹点数={len(IsTrackInEquU)}, 最大转矩={Tem_max:.2f} N·m")
        
        return IsTrackInEquU, Tem_max
    
    def generate_non_fw_result(
        self,
        controlType: Literal['id0', 'mtpa'],
        temTarget: float,
        weTarget: float,
        loadType: Literal['constTem', 'constWe']
    ) -> Dict[str, np.ndarray]:
        """
        生成非弱磁控制策略结果
        
        Args:
            controlType: 控制策略，'id0'或'mtpa'
            temTarget: 目标转矩 [N·m]
            weTarget: 目标电角速度 [rad/s]
            loadType: 负载类型，'constTem'或'constWe'
        
        Returns:
            包含以下键值的字典：
            - 'IsRange': 电流可用范围 (N×2)
            - 'IsTrackAtMaxCapacity': 最大容量轨迹 (N×5)
            - 'equTemTrack': 恒转矩轨迹 (N×4)
            - 'equWeTrack': 恒转速轨迹 (N×4)
            - 'givenPoint': 给定工况点 (4,)
        """
        self.logger.info(f"开始生成非弱磁控制结果: 策略={controlType}, Tem={temTarget:.2f}, We={weTarget:.2f}, 负载={loadType}")
        
        # 列索引
        iqIdx, idIdx, IsIdx, temIdx, weIdx = 0, 1, 2, 3, 4
        
        # 生成非弱磁轨迹
        if controlType == 'id0':
            nonFwIsTrack = self.calc_id0_track()
        elif controlType == 'mtpa':
            nonFwIsTrack = self.calc_mtpa_track()
        else:
            raise ValueError(f"不支持的控制策略: {controlType}")
        
        # IsRange
        IsRange = self.calc_full_track(mtpaTrack=nonFwIsTrack, mtpvTrack=None, fw1Track=None)
        
        # IsTrackAtMaxCapacity
        nonFwIsTrack_reversed = np.flipud(nonFwIsTrack)
        pointWe0 = nonFwIsTrack_reversed[0].copy()
        pointWe0[weIdx] = 0
        IsTrackAtMaxCapacity = [pointWe0]
        
        weBase = nonFwIsTrack_reversed[0, weIdx]
        weInterpList = np.linspace(0, weBase, self.constTemPointsAtNonFW)
        
        for i, We_i in enumerate(weInterpList):
            if i == 0 or i == len(weInterpList) - 1:
                continue
            pointInterp = nonFwIsTrack_reversed[0].copy()
            pointInterp[weIdx] = We_i
            IsTrackAtMaxCapacity.append(pointInterp)
        
        IsTrackAtMaxCapacity.extend(nonFwIsTrack_reversed)
        IsTrackAtMaxCapacity = np.array(IsTrackAtMaxCapacity)
        
        # equTemTrack
        temMax = nonFwIsTrack[-1, temIdx]
        temTargetClamped = min(temTarget, temMax)
        
        pointAtEquTem = self._interp_sorted(
            temTargetClamped,
            nonFwIsTrack[:, temIdx],
            nonFwIsTrack
        )
        
        weMaxAtEquTem = pointAtEquTem[weIdx]
        weList = np.linspace(0, weMaxAtEquTem, self.constTemPointsAtNonFW)
        
        equTemTrack = []
        for weI in weList:
            point = pointAtEquTem.copy()
            point[weIdx] = weI
            equTemTrack.append(point[[iqIdx, idIdx, temIdx, weIdx]])
        equTemTrack = np.array(equTemTrack)
        
        # equWeTrack
        weMax = nonFwIsTrack[0, weIdx]
        weTargetClamped = min(weTarget, weMax)
        
        pointAtEquWe = self._interp_sorted(
            weTargetClamped,
            nonFwIsTrack[:, weIdx],
            nonFwIsTrack
        )
        temMaxAtEquWe = pointAtEquWe[temIdx]
        
        if controlType == 'mtpa':
            equWeTrack = self.calc_mtpa_track()
            # 限制电流到pointAtEquWe的电流
            mask = equWeTrack[:, IsIdx] <= pointAtEquWe[IsIdx]
            equWeTrack = equWeTrack[mask]
        else:
            # 重新计算id0轨迹，限制电流
            equWeTrack = []
            for i in range(self.constWePointsAtNonFW + 1):
                iq = pointAtEquWe[IsIdx] * i / self.constWePointsAtNonFW
                id = 0.0
                Is = np.sqrt(id**2 + iq**2)
                Tem = self._calc_Tem(id, iq)
                equWeTrack.append([iq, id, Is, Tem, weTargetClamped])
            equWeTrack = np.array(equWeTrack)
        
        equWeTrack[:, weIdx] = weTargetClamped
        equWeTrack = equWeTrack[:, [iqIdx, idIdx, temIdx, weIdx]]
        
        # givenPoint
        if loadType == 'constTem':
            if weTarget > weMaxAtEquTem:
                givenPoint = equTemTrack[-1].copy()
                self.logger.info("恒转矩负载，目标转速超出最大转速")
            else:
                givenPoint = self._interp_sorted(
                    weTarget,
                    equTemTrack[:, 3],
                    equTemTrack
                )
        elif loadType == 'constWe':
            if temTarget > temMaxAtEquWe:
                givenPoint = equWeTrack[-1].copy()
                self.logger.info("恒转速负载，目标转矩超出最大转矩")
            else:
                givenPoint = self._interp_sorted(
                    temTarget,
                    equWeTrack[:, 2],
                    equWeTrack
                )
        else:
            raise ValueError(f"不支持的负载类型: {loadType}")
        
        self.logger.info("非弱磁控制结果生成完成")
        
        return {
            'IsRange': IsRange,
            'IsTrackAtMaxCapacity': IsTrackAtMaxCapacity,
            'equTemTrack': equTemTrack,
            'equWeTrack': equWeTrack,
            'givenPoint': givenPoint
        }
    
    def generate_fw_result(
        self,
        controlType: Literal['id0', 'mtpa'],
        temTarget: float,
        weTarget: float,
        loadType: Literal['constTem', 'constWe']
    ) -> Dict[str, np.ndarray]:
        """
        生成弱磁控制策略结果
        
        本方法计算弱磁控制下的完整性能曲线，包括：
        - 弱磁I区轨迹（fw1Track）：总是计算
        - 弱磁II区轨迹（mtpvTrack）：仅在isDemag=True时计算
        
        Args:
            controlType: 控制策略，'id0'或'mtpa'
            temTarget: 目标转矩 [N·m]
            weTarget: 目标电角速度 [rad/s]
            loadType: 负载类型，'constTem'或'constWe'
        
        Returns:
            包含以下键值的字典：
            - 'IsRange': 电流可用范围 (N×2)
            - 'IsTrackAtMaxCapacity': 最大容量轨迹 (N×5)
            - 'equTemTrack': 恒转矩轨迹 (N×4)
            - 'equWeTrack': 恒转速轨迹 (N×4)
            - 'givenPoint': 给定工况点 (4,)
        """
        self.logger.info(f"开始生成弱磁控制结果: 策略={controlType}, Tem={temTarget:.2f}, We={weTarget:.2f}, 负载={loadType}")
        
        # 列索引
        iqIdx, idIdx, IsIdx, temIdx, weIdx = 0, 1, 2, 3, 4
        
        # 生成非弱磁轨迹
        if controlType == 'id0':
            nonFwIsTrack = self.calc_id0_track()
        elif controlType == 'mtpa':
            nonFwIsTrack = self.calc_mtpa_track()
        else:
            raise ValueError(f"不支持的控制策略: {controlType}")
        
        # 生成MTPV和弱磁I区轨迹
        # 注意：只有发生完全退磁时，MTPV轨迹才有意义
        if self.isDemag:
            mtpvTrack = self.calc_mtpv_track(nonFWTrack=nonFwIsTrack)
            self.logger.info(f"发生完全退磁，计算MTPV轨迹，点数={len(mtpvTrack)}")
        else:
            mtpvTrack = None
            self.logger.info("未发生完全退磁，跳过MTPV轨迹计算")
        
        fw1Track = self.calc_fw1_track(mtpaTrack=nonFwIsTrack, mtpvTrack=mtpvTrack)
        
        # IsRange
        IsRange = self.calc_full_track(
            mtpaTrack=nonFwIsTrack,
            mtpvTrack=mtpvTrack,
            fw1Track=fw1Track
        )
        
        # IsTrackAtMaxCapacity
        nonFwIsTrack_reversed = np.flipud(nonFwIsTrack)
        pointWe0 = nonFwIsTrack_reversed[0].copy()
        pointWe0[weIdx] = 0
        IsTrackAtMaxCapacity = [pointWe0]
        
        weBase = nonFwIsTrack_reversed[0, weIdx]
        weInterpList = np.linspace(0, weBase, self.constTemPointsAtNonFW)
        
        for i, We_i in enumerate(weInterpList):
            if i == 0 or i == len(weInterpList) - 1:
                continue
            pointInterp = nonFwIsTrack_reversed[0].copy()
            pointInterp[weIdx] = We_i
            IsTrackAtMaxCapacity.append(pointInterp)
        
        fwIsTrack = fw1Track
        if mtpvTrack is not None and len(mtpvTrack) > 0:
            fwIsTrack = np.concatenate((fwIsTrack, mtpvTrack), axis=0)
        IsTrackAtMaxCapacity.extend(fwIsTrack.tolist())
        IsTrackAtMaxCapacity = np.array(IsTrackAtMaxCapacity)
        
        # equTemTrack
        temMax = nonFwIsTrack[-1, temIdx]
        temTargetClamped = min(temTarget, temMax)
        
        fwBoundaryPointAtEquTem = self._interp_sorted(
            temTargetClamped,
            nonFwIsTrack[:, temIdx],
            nonFwIsTrack
        )
        
        fwBoundaryWe = fwBoundaryPointAtEquTem[weIdx]
        weList = np.linspace(0, fwBoundaryWe, self.constTemPointsAtNonFW)
        
        equTemTrack = []
        for weI in weList:
            point = fwBoundaryPointAtEquTem.copy()
            point[weIdx] = weI
            equTemTrack.append(point)
        
        # 计算弱磁段
        equTemTrack_fw = self.calc_equ_tem_track(
            Tem_target=temTargetClamped,
            mtpaTrack=nonFwIsTrack,
            fw1Track=fw1Track,
            mtpvTrack=mtpvTrack
        )
        
        equTemTrack.extend(equTemTrack_fw)
        equTemTrack = np.array([
            [point[iqIdx], point[idIdx], point[temIdx], point[weIdx]]
            for point in equTemTrack
        ])
        
        # equWeTrack
        weMax = IsTrackAtMaxCapacity[-1, weIdx]
        weTargetClamped = min(weTarget, weMax)
        
        equWeTrack = []
        fwBoundaryPointAtEquWe = None
        
        if weTargetClamped < nonFwIsTrack[0, weIdx]:
            fwBoundaryPointAtEquWe = self._interp_sorted(
                weTargetClamped,
                nonFwIsTrack[:, weIdx],
                nonFwIsTrack
            )
            
            IsAtFwBoundaryPoint = fwBoundaryPointAtEquWe[IsIdx]
            
            if controlType == 'mtpa':
                equWeTrack_nonFw = self.calc_mtpa_track()
                mask = equWeTrack_nonFw[:, IsIdx] <= IsAtFwBoundaryPoint
                equWeTrack_nonFw = equWeTrack_nonFw[mask]
            else:
                equWeTrack_nonFw = []
                for i in range(self.constWePointsAtNonFW + 1):
                    iq = IsAtFwBoundaryPoint * i / self.constWePointsAtNonFW
                    id = 0.0
                    Is = np.sqrt(id**2 + iq**2)
                    Tem = self._calc_Tem(id, iq)
                    equWeTrack_nonFw.append([iq, id, Is, Tem, weTargetClamped])
                equWeTrack_nonFw = np.array(equWeTrack_nonFw)
            
            for point in equWeTrack_nonFw:
                point[weIdx] = weTargetClamped
            
            equWeTrack = list(equWeTrack_nonFw)
        
        # 计算弱磁段
        equWeTrack_fw = self.calc_equ_u_track(
            We=weTargetClamped,
            mtpaTrack=nonFwIsTrack,
            fw1Track=fw1Track,
            mtpvTrack=mtpvTrack,
            splitRatio=5
        )[0]
        
        equWeTrack.extend(equWeTrack_fw)
        equWeTrack = np.array([
            [point[iqIdx], point[idIdx], point[temIdx], point[weIdx]]
            for point in equWeTrack
        ])
        
        # givenPoint
        if loadType == 'constTem':
            if weTarget > equTemTrack[-1, 3]:
                givenPoint = equTemTrack[-1].copy()
                self.logger.info("恒转矩负载，目标转速超出最大转速")
            else:
                givenPoint = self._interp_sorted(
                    weTarget,
                    equTemTrack[:, 3],
                    equTemTrack
                )
        elif loadType == 'constWe':
            if temTarget > equWeTrack[-1, 2]:
                givenPoint = equWeTrack[-1].copy()
                self.logger.info("恒转速负载，目标转矩超出最大转矩")
            else:
                givenPoint = self._interp_sorted(
                    temTarget,
                    equWeTrack[:, 2],
                    equWeTrack
                )
        else:
            raise ValueError(f"不支持的负载类型: {loadType}")
        
        self.logger.info("弱磁控制结果生成完成")
        
        return {
            'IsRange': IsRange,
            'IsTrackAtMaxCapacity': IsTrackAtMaxCapacity,
            'equTemTrack': equTemTrack,
            'equWeTrack': equWeTrack,
            'givenPoint': givenPoint
        }
    
    def calc_equ_torque_track(self, Tem_target: float, mtpaTrack: np.ndarray, 
                              fw1Track: np.ndarray, mtpvTrack: Optional[np.ndarray] = None,
                              equTemPoints: int = 50) -> np.ndarray:
        """
        计算给定转矩的等转矩线
        
        参数:
            Tem_target: 目标电磁转矩 (N·m)
            mtpaTrack: MTPA轨迹 (N×4)，格式：[[iq0, id0, Is0, Tem0], ...]
            fw1Track: 弱磁I区轨迹 (N×5)，格式：[[iq0, id0, Is0, Tem0, We0], ...]
            mtpvTrack: MTPV轨迹 (N×5)，可选，格式：[[iq0, id0, Is0, Tem0, We0], ...]
            equTemPoints: 等转矩点数
            
        返回:
            equTemTrack: 等转矩轨迹，格式：[[iq0, id0, Is0, Tem0, We0], ...]
        """
        self.logger.info(f"开始计算等转矩线，目标转矩={Tem_target:.2f} N·m")
        
        # 获得fw1Track上的最大和最小电磁转矩
        Tem_max = fw1Track[0, 3]
        Tem_splitFW = fw1Track[-1, 3]
        
        # 判断给定转矩是否超出最大电磁转矩
        if Tem_target > Tem_max:
            raise ValueError(f"目标转矩{Tem_target:.2f} N·m大于最大转矩{Tem_max:.2f} N·m")
        
        # 在MTPA上查找对应的id_mtpa和iq_mtpa
        id_mtpa = self._interp_sorted(Tem_target, mtpaTrack[:, 3], mtpaTrack[:, 1])
        iq_mtpa = self._interp_sorted(Tem_target, mtpaTrack[:, 3], mtpaTrack[:, 0])
        self.logger.debug(f"MTPA点: iq_mtpa={iq_mtpa:.2f} A, id_mtpa={id_mtpa:.2f} A")
        
        lastPoint = []
        
        # 判断等转矩线的左端点在弱磁I区/II区
        if Tem_target > Tem_splitFW or mtpvTrack is None:
            # 左端点在弱磁I区
            iq_fw1 = self._interp_sorted(Tem_target, fw1Track[:, 3], fw1Track[:, 0])
            id_fw1 = self._interp_sorted(Tem_target, fw1Track[:, 3], fw1Track[:, 1])
            Is_fw1 = self._interp_sorted(Tem_target, fw1Track[:, 3], fw1Track[:, 2])
            Tem_fw1 = Tem_target
            We_fw1 = self._interp_sorted(Tem_target, fw1Track[:, 3], fw1Track[:, 4])
            lastPoint = [iq_fw1, id_fw1, Is_fw1, Tem_fw1, We_fw1]
            self.logger.debug(f"左端点在弱磁I区: iq={iq_fw1:.2f} A, id={id_fw1:.2f} A")
            
            # 判断扫描变量
            id_diff = abs(id_mtpa - id_fw1)
            iq_diff = abs(iq_mtpa - iq_fw1)
            
            if id_diff >= iq_diff:
                scan_by_id = True
                id_step = (id_mtpa - id_fw1 * 0.95) / equTemPoints
                self.logger.debug(f"选择id作为扫描变量, step={id_step:.2f} A")
            else:
                scan_by_id = False
                iq_step = (iq_mtpa - iq_fw1 * 0.95) / equTemPoints
                self.logger.debug(f"选择iq作为扫描变量, step={iq_step:.2f} A")
        else:
            # 左端点在弱磁II区
            minTemAtMtpv = mtpvTrack[-1, 3]
            minidAtMtpv = mtpvTrack[-1, 1]
            
            if Tem_target < minTemAtMtpv:
                iq_mtpv = 0.0
                id_mtpv = minidAtMtpv
                self.logger.debug(f"左端点在弱磁II区(Tem<minTem): iq={iq_mtpv:.2f}, id={id_mtpv:.2f}")
            else:
                iq_mtpv = self._interp_sorted(Tem_target, mtpvTrack[:, 3], mtpvTrack[:, 0])
                id_mtpv = self._interp_sorted(Tem_target, mtpvTrack[:, 3], mtpvTrack[:, 1])
                Is_mtpv = self._interp_sorted(Tem_target, mtpvTrack[:, 3], mtpvTrack[:, 2])
                Tem_mtpv = Tem_target
                We_mtpv = self._interp_sorted(Tem_target, mtpvTrack[:, 3], mtpvTrack[:, 4])
                lastPoint = [iq_mtpv, id_mtpv, Is_mtpv, Tem_mtpv, We_mtpv]
                self.logger.debug(f"左端点在弱磁II区: iq={iq_mtpv:.2f}, id={id_mtpv:.2f}")
            
            # 判断扫描变量
            id_diff = abs(id_mtpa - id_mtpv)
            iq_diff = abs(iq_mtpa - iq_mtpv)
            
            if id_diff >= iq_diff:
                scan_by_id = True
                id_step = (id_mtpa - id_mtpv * 0.95) / equTemPoints
                self.logger.debug(f"选择id作为扫描变量, step={id_step:.2f} A")
            else:
                scan_by_id = False
                iq_step = (iq_mtpa - iq_mtpv * 0.95) / equTemPoints
                self.logger.debug(f"选择iq作为扫描变量, step={iq_step:.2f} A")
        
        # 初始化等转矩轨迹列表
        equTemTrack = []
        
        # 根据扫描变量进行迭代
        if scan_by_id:
            # id作为扫描变量
            iq_init = iq_mtpa * 1.05
            
            for i in range(equTemPoints + 1):
                id = id_mtpa - i * id_step
                iq = self._calc_iq_by_Tem_id(Tem_target, id, iq_init)
                
                if iq < 0.0:
                    self.logger.debug(f"iq<0，停止计算")
                    break
                
                Is = np.sqrt(id**2 + iq**2)
                if Is > self.IPmax:
                    self.logger.debug(f"电流超限: Is={Is:.2f} A")
                    break
                
                success, _, We = self._calc_We(id, iq, self.UPmax)
                if not success:
                    self.logger.debug(f"We无解，停止计算")
                    break
                
                equTemTrack.append([iq, id, Is, Tem_target, We])
                iq_init = iq
        else:
            # iq作为扫描变量
            id_init = id_mtpa * 1.05
            
            for i in range(equTemPoints + 1):
                iq = iq_mtpa - i * iq_step
                id = self._calc_id_by_Tem_iq(Tem_target, iq, id_init, -self.IPmax)
                
                if id < -self.IPmax:
                    self.logger.debug(f"id超限，停止计算")
                    break
                
                Is = np.sqrt(id**2 + iq**2)
                if Is > self.IPmax:
                    self.logger.debug(f"电流超限: Is={Is:.2f} A")
                    break
                
                success, _, We = self._calc_We(id, iq, self.UPmax)
                if not success:
                    self.logger.debug(f"We无解，停止计算")
                    break
                
                equTemTrack.append([iq, id, Is, Tem_target, We])
                id_init = id
        
        # 添加最后一个点
        if lastPoint:
            equTemTrack.append(lastPoint)
        
        result = np.array(equTemTrack)
        self.logger.info(f"等转矩线计算完成，轨迹点数={len(result)}")
        
        return result
    
    def calc_equ_voltage_track(self, We: float, mtpaTrack: np.ndarray, fw1Track: np.ndarray,
                               mtpvTrack: Optional[np.ndarray] = None,
                               equUPoints: int = 100, splitRatio: int = 5) -> tuple[np.ndarray, float]:
        """
        计算给定转速下的等电压椭圆轨迹
        
        参数:
            We: 电角速度 (rad/s)
            mtpaTrack: MTPA轨迹 (N×4)，格式：[[iq0, id0, Is0, Tem0], ...]
            fw1Track: 弱磁I区轨迹 (N×5)，格式：[[iq0, id0, Is0, Tem0, We0], ...]
            mtpvTrack: MTPV轨迹 (N×5)，可选
            equUPoints: 等电压椭圆扫描点数
            splitRatio: 分段比例
            
        返回:
            tuple: (IsTrackInEquU, Tem_max) - 等电压椭圆轨迹和最大转矩
        """
        self.logger.info(f"开始计算等电压椭圆，转速={We:.2f} rad/s")
        
        # 步骤1：转速范围判断
        maxWeAtMtpaTrackEnd = 0
        if len(mtpaTrack) > 0:
            id_end = mtpaTrack[-1, 1]
            iq_end = mtpaTrack[-1, 0]
            _, _, maxWeAtMtpaTrackEnd = self._calc_We(id_end, iq_end, self.UPmax)
        
        maxWeAtMtpaTrackStart = 0
        if len(mtpaTrack) > 0:
            id_start_mtpa = mtpaTrack[0, 1]
            iq_start_mtpa = mtpaTrack[0, 0]
            _, _, maxWeAtMtpaTrackStart = self._calc_We(id_start_mtpa, iq_start_mtpa, self.UPmax)
        
        self.logger.debug(f"MTPA末端最大转速={maxWeAtMtpaTrackEnd:.2f} rad/s")
        self.logger.debug(f"MTPA起点最大转速={maxWeAtMtpaTrackStart:.2f} rad/s")
        
        if We <= maxWeAtMtpaTrackEnd:
            self.logger.info("转速过低，等电压椭圆在电流控制范围外")
            return np.array([]), 0.0
        
        # 步骤2：确定轨迹起点
        if We <= maxWeAtMtpaTrackStart:
            # 等电压椭圆与MTPA轨迹相交
            self.logger.debug("起点在MTPA轨迹内")
            # 计算MTPA轨迹的We值
            We_mtpa = np.array([self._calc_We(mtpaTrack[i, 1], mtpaTrack[i, 0], self.UPmax)[2] 
                               for i in range(len(mtpaTrack))])
            # We_mtpa从大到小排序，需要反转
            We_mtpa_sorted = We_mtpa[::-1]
            mtpa_reversed = mtpaTrack[::-1]
            id_start = self._interp_sorted(We, We_mtpa_sorted, mtpa_reversed[:, 1])
            iq_start = self._interp_sorted(We, We_mtpa_sorted, mtpa_reversed[:, 0])
            self.logger.debug(f"插值得到起点: id={id_start:.2f}, iq={iq_start:.2f}")
        else:
            # 等电压椭圆在MTPA轨迹外侧
            self.logger.debug("起点在MTPA轨迹外侧")
            success_R, _, id_right = self._calc_id(iq=0, We=We, Us=self.UPmax, 
                                                   id_init=self.id_demag, root='R')
            if success_R:
                id_start = id_right
                self.logger.debug(f"计算得到id_start={id_start:.2f}")
            else:
                id_start = 0
                self.logger.debug("计算失败，使用默认值id_start=0")
            iq_start = 0
        
        # 步骤3：确定轨迹终点
        maxWeAtFw1TrackStart = fw1Track[-1, 4]
        self.logger.debug(f"弱磁I区终点最大转速={maxWeAtFw1TrackStart:.2f} rad/s")
        
        if We <= maxWeAtFw1TrackStart or mtpvTrack is None:
            # 终点在弱磁I区
            self.logger.debug("终点在弱磁I区")
            We_fw1 = fw1Track[:, 4]
            id_end = self._interp_sorted(We, We_fw1, fw1Track[:, 1])
            iq_end = self._interp_sorted(We, We_fw1, fw1Track[:, 0])
            self.logger.debug(f"插值得到终点: id={id_end:.2f}, iq={iq_end:.2f}")
        else:
            # 终点在MTPV区
            self.logger.debug("终点在MTPV区")
            We_mtpv = mtpvTrack[:, 4]
            id_end = self._interp_sorted(We, We_mtpv, mtpvTrack[:, 1])
            iq_end = self._interp_sorted(We, We_mtpv, mtpvTrack[:, 0])
            self.logger.debug(f"插值得到终点: id={id_end:.2f}, iq={iq_end:.2f}")
        
        # 步骤4：构建等电压椭圆轨迹
        success, _, iq_max = self._calc_iq(id=self.id_demag, We=We, Us=self.UPmax, iq_init=0)
        
        if not success or iq_max is None or np.isnan(iq_max) or iq_max <= 0:
            self.logger.debug("无法计算iq_max，设置为0")
            iq_max = 0
        else:
            self.logger.debug(f"等电压圆最大iq={iq_max:.2f}")
        
        IsTrackInEquU = []
        
        # 计算阈值
        iq_threshold = iq_max / splitRatio if splitRatio > 0 else 0
        self.logger.debug(f"iq扫描阈值={iq_threshold:.2f}")
        
        # 第一段：[iq_start, iq_threshold]
        if iq_start <= iq_threshold:
            self.logger.debug(f"执行第一段计算: iq从{iq_start:.2f}到{iq_threshold:.2f}")
            iq_points_1 = np.linspace(iq_start, iq_threshold, equUPoints // 2)
            
            id_init_value = id_start
            for iq in iq_points_1:
                success, _, id = self._calc_id(iq=iq, We=We, Us=self.UPmax, 
                                              id_init=id_init_value, root='R')
                if success and id is not None and not np.isnan(id):
                    Tem = self._calc_Tem(id, iq)
                    Is = np.sqrt(iq**2 + id**2)
                    IsTrackInEquU.append([iq, id, Is, Tem, We])
                    id_init_value = id
            
            if IsTrackInEquU:
                id_split = IsTrackInEquU[-1][1]
                iq_split = IsTrackInEquU[-1][0]
            else:
                id_split = id_start
                iq_split = iq_start
        else:
            self.logger.debug("跳过第一段")
            id_split = id_start
            iq_split = iq_start
        
        # 第二段：[id_split, id_end]
        self.logger.debug(f"执行第二段计算: id从{id_split:.2f}到{id_end:.2f}")
        id_points_2 = np.linspace(id_split, id_end, equUPoints // 2)
        
        iq_init_value = iq_split
        for id in id_points_2:
            success, _, iq = self._calc_iq(id=id, We=We, Us=self.UPmax, iq_init=iq_init_value)
            if success and iq is not None and not np.isnan(iq):
                Tem = self._calc_Tem(id, iq)
                Is = np.sqrt(iq**2 + id**2)
                IsTrackInEquU.append([iq, id, Is, Tem, We])
                iq_init_value = iq
        
        # 转换为numpy数组
        IsTrackInEquU = np.array(IsTrackInEquU)
        
        # 计算最大转矩
        Tem_max = 0.0
        if len(IsTrackInEquU) > 0:
            Tem_max = np.max(IsTrackInEquU[:, 3])
        
        self.logger.info(f"等电压椭圆计算完成，轨迹点数={len(IsTrackInEquU)}, 最大转矩={Tem_max:.2f} N·m")
        
        return IsTrackInEquU, Tem_max
    
    def plot_tracks_for_operating_point(self, Tem_target: float, We_target: float,
                                        mtpaTrack: np.ndarray, fw1Track: np.ndarray, 
                                        mtpvTrack: Optional[np.ndarray] = None,
                                        equUPoints: int = 100, equTemPoints: int = 50) -> tuple:
        """
        计算给定工况点的三条轨迹
        
        参数:
            Tem_target: 目标转矩 (N·m)
            We_target: 目标电角速度 (rad/s)
            mtpaTrack: MTPA轨迹
            fw1Track: 弱磁I区轨迹
            mtpvTrack: MTPV轨迹（可选）
            equUPoints: 等电压椭圆扫描点数
            equTemPoints: 等转矩线扫描点数
            
        返回:
            tuple: (IsTrack, equTemTrack, IsTrackInEquU)
        """
        self.logger.info(f"绘制给定工况点的三条轨迹: Tem={Tem_target:.2f} N·m, We={We_target:.2f} rad/s")
        
        # 步骤1：获取完整的电流容量边界 IsTrack
        self.logger.debug("步骤1: 获取完整电流容量边界")
        mtpa_part = mtpaTrack[:, :2]  # [iq, id]
        fw1_part = fw1Track[:, :2]
        if mtpvTrack is not None and len(mtpvTrack) > 0:
            mtpv_part = mtpvTrack[:, :2]
            IsTrack = np.vstack([mtpa_part, fw1_part, mtpv_part])
        else:
            IsTrack = np.vstack([mtpa_part, fw1_part])
        self.logger.debug(f"IsTrack点数: {len(IsTrack)}")
        
        # 步骤2：计算等转矩线
        self.logger.debug(f"步骤2: 计算等转矩线 (Tem={Tem_target:.2f} N·m)")
        equTemTrack = self.calc_equ_torque_track(
            Tem_target=Tem_target,
            mtpaTrack=mtpaTrack,
            fw1Track=fw1Track,
            mtpvTrack=mtpvTrack,
            equTemPoints=equTemPoints
        )
        self.logger.debug(f"等转矩线点数: {len(equTemTrack)}")
        
        # 步骤3：计算等电压椭圆
        self.logger.debug(f"步骤3: 计算等电压椭圆 (We={We_target:.2f} rad/s)")
        IsTrackInEquU, Tem_max_at_We = self.calc_equ_voltage_track(
            We=We_target,
            mtpaTrack=mtpaTrack,
            fw1Track=fw1Track,
            mtpvTrack=mtpvTrack,
            equUPoints=equUPoints
        )
        self.logger.debug(f"等电压椭圆点数: {len(IsTrackInEquU)}, 最大转矩: {Tem_max_at_We:.2f} N·m")
        
        self.logger.info("给定工况点的三条轨迹计算完成")
        
        return IsTrack, equTemTrack, IsTrackInEquU

