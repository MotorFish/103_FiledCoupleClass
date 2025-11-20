# PMSM场路耦合计算器

永磁同步电机（PMSM）场路耦合性能计算的Python实现。

## 功能特性

- ✅ 电压/电流限幅计算
- ✅ 电磁转矩计算
- ✅ 电角速度计算
- ✅ MTPA（最大转矩/电流比）控制轨迹计算
- ✅ MTPV（最大转矩/功率比）控制轨迹计算
- ✅ 弱磁I区轨迹计算
- ✅ 等转矩线计算
- ✅ 等电压椭圆计算
- ✅ T-n（转矩-转速）曲线计算
- ✅ id=0控制策略
- ✅ 完整的日志记录支持

## 文件结构

```
Refactor/
├── pmsm_fcc_calculator.py  # 主计算类（1600+行）
├── test_pmsm_fcc.py        # 测试和可视化（700+行）
└── README.md               # 本文档
```

## 安装依赖

```bash
pip install numpy scipy matplotlib
```

## 快速开始

### 1. 基本使用

```python
import numpy as np
from pmsm_fcc_calculator import PMSMFieldCoupledCalculator

# 准备电机参数
id_range = np.linspace(-300, 100, 61)
iq_range = np.linspace(-300, 300, 71)

# 磁链-id特性
PsiR_vs_id = np.column_stack([
    id_range,
    1.0 - 2.0e-4 * id_range - 2.5e-7 * id_range**2
])

# d轴电感-id特性
Lad_vs_id = np.column_stack([
    id_range,
    0.006 + (-1.5e-8) * (id_range + 133.33)**2
])

# q轴电感-iq特性
Laq_vs_iq = np.column_stack([
    iq_range,
    9e-3 - 8.0e-9 * iq_range**2
])

# 创建计算器实例
calculator = PMSMFieldCoupledCalculator(
    PsiR_vs_id=PsiR_vs_id,
    Lad_vs_id=Lad_vs_id,
    Laq_vs_iq=Laq_vs_iq,
    Udc=540.0,          # 母线电压 [V]
    Imax=300.0,         # 限幅电流 [A]
    connectType='Y',    # 绕组连接方式
    modulationType='SVPWM',  # 调制方式
    polePairs=4,        # 极对数
    Lsigma=0.015e-3,    # 定子漏抗 [H]
    Rs=0.02,            # 相电阻 [Ω]
    We_base=100.0       # 基速 [rad/s]
)
```

### 2. 计算MTPA轨迹

```python
# 计算MTPA控制轨迹
mtpa_track = calculator.calc_mtpa_track()

# 轨迹格式: [iq, id, Is, Tem, maxWe]
print(f"轨迹点数: {len(mtpa_track)}")
print(f"最大转矩: {mtpa_track[-1, 3]:.2f} N·m")
```

### 3. 计算id=0控制轨迹

```python
# 计算id=0控制轨迹
id0_track = calculator.calc_id0_track()

# 轨迹格式: [iq, id, Is, Tem, maxWe]
print(f"最大转矩: {id0_track[-1, 3]:.2f} N·m")
```

### 4. 生成非弱磁控制结果

```python
# 生成MTPA非弱磁控制结果
result = calculator.generate_non_fw_result(
    controlType='mtpa',      # 控制策略: 'id0' 或 'mtpa'
    temTarget=500.0,         # 目标转矩 [N·m]
    weTarget=200.0,          # 目标转速 [rad/s]
    loadType='constTem'      # 负载类型: 'constTem' 或 'constWe'
)

# 结果包含:
# - IsRange: 电流可用范围 (N×2)
# - IsTrackAtMaxCapacity: 最大容量轨迹 (N×5)
# - equTemTrack: 恒转矩轨迹 (N×4)
# - equWeTrack: 恒转速轨迹 (N×4)
# - givenPoint: 给定工况点 (4,)

print(f"工况点: iq={result['givenPoint'][0]:.2f} A, "
      f"id={result['givenPoint'][1]:.2f} A, "
      f"Tem={result['givenPoint'][2]:.2f} N·m, "
      f"We={result['givenPoint'][3]:.2f} rad/s")
```

### 5. 生成弱磁控制结果

```python
# 检查是否发生退磁
if calculator.isDemag:
    # 生成弱磁MTPA控制结果
    fw_result = calculator.generate_fw_result(
        controlType='mtpa',
        temTarget=500.0,
        weTarget=500.0,
        loadType='constWe'
    )
    
    print(f"弱磁控制工况点: "
          f"Tem={fw_result['givenPoint'][2]:.2f} N·m, "
          f"We={fw_result['givenPoint'][3]:.2f} rad/s")
else:
    print("未发生完全退磁，不需要弱磁控制")
```

## 运行测试

```bash
# 进入Refactor目录
cd Refactor

# 运行完整测试套件
python test_pmsm_fcc.py
```

测试将生成以下图表：
- `motor_characteristics.png` - 电机特性曲线
- `mtpa_track.png` - MTPA轨迹
- `current_range.png` - 电流控制范围
- `equ_lines_and_circles.png` - 等转矩线和等电压椭圆
- `非弱磁MTPA控制策略.png` - 非弱磁控制结果
- `弱磁MTPA控制策略.png` - 弱磁控制结果（如果适用）

## 高级用法

### 自定义日志记录

```python
import logging

# 创建自定义logger
logger = logging.getLogger('my_pmsm_calc')
logger.setLevel(logging.DEBUG)

# 创建计算器时传入logger
calculator = PMSMFieldCoupledCalculator(
    ...,  # 其他参数
    logger=logger
)
```

### 调整计算精度

```python
calculator = PMSMFieldCoupledCalculator(
    ...,  # 电机参数
    mtpaPoints=500,          # MTPA扫描点数（默认200）
    equTemPoints=200,        # 等转矩线点数（默认100）
    equUPoints=3000,         # 等电压椭圆点数（默认2000）
    IsSmoothInterp=True      # 使用三次样条插值（默认True）
)
```

### 计算等转矩线

```python
# 首先计算必要的轨迹
mtpa_track = calculator.calc_mtpa_track()
fw1_track = calculator.calc_fw1_track(mtpaTrack=mtpa_track)

# 计算指定转矩的等转矩线
equ_tem_track = calculator.calc_equ_tem_track(
    Tem_target=500.0,        # 目标转矩
    mtpaTrack=mtpa_track,
    fw1Track=fw1_track
)

# 轨迹格式: [iq, id, Is, Tem, We]
print(f"等转矩线点数: {len(equ_tem_track)}")
```

### 计算等电压椭圆

```python
# 计算指定转速的等电压椭圆
equ_u_track, Tem_max = calculator.calc_equ_u_track(
    We=300.0,                # 目标转速
    mtpaTrack=mtpa_track,
    fw1Track=fw1_track
)

print(f"等电压椭圆点数: {len(equ_u_track)}")
print(f"最大转矩: {Tem_max:.2f} N·m")
```

## API参考

### 主要公开方法

#### calc_id0_track()
计算id=0控制策略的电流轨迹。

**返回**: numpy数组 (N×5): [iq, id, Is, Tem, maxWe]

#### calc_mtpa_track()
计算MTPA控制策略的电流轨迹。

**返回**: numpy数组 (N×5): [iq, id, Is, Tem, maxWe]

#### calc_mtpv_track(We_base)
计算MTPV控制策略的电流轨迹。

**参数**:
- `We_base`: 基速 [rad/s]

**返回**: numpy数组 (N×5): [iq, id, Is, Tem, We]

#### calc_fw1_track(mtpaTrack, mtpvTrack=None)
计算弱磁I区的电流轨迹。

**参数**:
- `mtpaTrack`: MTPA轨迹
- `mtpvTrack`: MTPV轨迹（可选）

**返回**: numpy数组 (N×5): [iq, id, Is, Tem, We]

#### calc_full_track(mtpaTrack, mtpvTrack=None, fw1Track=None)
拼接完整的电流控制范围。

**返回**: numpy数组 (N×2): [iq, id]

#### calc_equ_tem_track(Tem_target, mtpaTrack, fw1Track, mtpvTrack=None)
计算等转矩线。

**参数**:
- `Tem_target`: 目标转矩 [N·m]
- `mtpaTrack`: MTPA轨迹
- `fw1Track`: 弱磁I区轨迹
- `mtpvTrack`: MTPV轨迹（可选）

**返回**: numpy数组 (N×5): [iq, id, Is, Tem, We]

#### calc_equ_u_track(We, mtpaTrack, fw1Track, mtpvTrack=None, splitRatio=5)
计算等电压椭圆。

**参数**:
- `We`: 目标转速 [rad/s]
- `mtpaTrack`: MTPA轨迹
- `fw1Track`: 弱磁I区轨迹
- `mtpvTrack`: MTPV轨迹（可选）
- `splitRatio`: 分段比例

**返回**: (IsTrackInEquU, Tem_max)

#### generate_non_fw_result(controlType, temTarget, weTarget, loadType)
生成非弱磁控制策略的完整结果。

**参数**:
- `controlType`: 'id0' 或 'mtpa'
- `temTarget`: 目标转矩 [N·m]
- `weTarget`: 目标转速 [rad/s]
- `loadType`: 'constTem' 或 'constWe'

**返回**: 包含以下键的字典
- `IsRange`: 电流可用范围 (N×2)
- `IsTrackAtMaxCapacity`: 最大容量轨迹 (N×5)
- `equTemTrack`: 恒转矩轨迹 (N×4): [iq, id, Tem, We]
- `equWeTrack`: 恒转速轨迹 (N×4): [iq, id, Tem, We]
- `givenPoint`: 给定工况点 (4,): [iq, id, Tem, We]

#### generate_fw_result(controlType, temTarget, weTarget, loadType)
生成弱磁控制策略的完整结果。

**参数**: 同 `generate_non_fw_result()`

**返回**: 同 `generate_non_fw_result()`

## 常见问题

### 1. 如何选择控制策略？

- **id=0**: 适用于表贴式PMSM，实现简单但效率较低
- **MTPA**: 适用于内置式PMSM，最大化转矩/电流比，效率最高
- **弱磁控制**: 当需要高速运行且电压受限时使用

### 2. 什么时候需要弱磁控制？

当电机需要在基速以上运行时，需要采用弱磁控制。计算器会自动检测是否需要弱磁（通过`isDemag`属性）。

### 3. 如何解释计算结果？

- **IsRange**: 显示在电压和电流约束下可用的电流矢量范围
- **IsTrackAtMaxCapacity**: 显示每个转速下的最大转矩输出能力
- **equTemTrack**: 显示恒定转矩下，转速从0到最大的控制轨迹
- **equWeTrack**: 显示恒定转速下，转矩从0到最大的控制轨迹
- **givenPoint**: 显示特定工况（目标转矩+转速）下的最优控制点

### 4. 为什么有些计算会失败？

常见原因：
- 目标转矩超出电机能力
- 目标转速超出电压限制
- 电机参数不合理（如负电感）
- 数值计算未收敛

检查日志输出可以获取详细的错误信息。

## 技术细节

### 算法实现

- **插值方法**: 支持线性插值和三次样条插值
- **搜索算法**: 自适应步长单侧搜索，快速收敛
- **数值稳定性**: 采用阻尼系数提高迭代稳定性
- **边界处理**: 完善的边界检查和异常处理

### 计算复杂度

- MTPA轨迹: O(n × m)，n为扫描点数，m为搜索迭代次数
- 等转矩线/等电压椭圆: O(k × m)，k为轨迹点数
- 完整控制结果: O(n × m + k × m)

典型计算时间（Intel i7处理器）：
- MTPA轨迹（200点）: ~1秒
- 完整弱磁控制结果: ~5-10秒

## 参考资料

本实现基于以下理论：
1. PMSM场路耦合模型
2. dq轴坐标变换
3. 最大转矩/电流比（MTPA）控制理论
4. 弱磁控制理论

## 许可证

本项目基于原始Jupyter Notebook重构而来，用于电机控制算法的研究和开发。

## 更新日志

### v1.0.0 (2024)
- 初始版本
- 完整的PMSM场路耦合计算功能
- 支持id=0、MTPA、弱磁控制策略
- 完善的日志记录和错误处理
- 包含测试套件和可视化功能

## 联系方式

如有问题或建议，请通过项目仓库提交issue。

