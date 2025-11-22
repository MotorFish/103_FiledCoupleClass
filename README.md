# PMSM场路耦合性能计算器

## 项目简介

本项目提供了永磁同步电机（PMSM）场路耦合性能计算的完整解决方案，包括电机特性分析、控制策略计算和可视化功能。

### 主要功能

- ⚡ **电机基础计算**
  - 电压/电流限幅计算
  - 电磁转矩计算
  - 电角速度计算
  - 定子电流圆计算

- 🎯 **控制策略轨迹**
  - MTPA（最大转矩/电流比）控制轨迹
  - MTPV（最大转矩/功率比）控制轨迹
  - id=0 控制策略
  - 弱磁控制区域分析

- 📊 **性能分析**
  - 等转矩线计算与可视化
  - 等电压椭圆计算
  - T-n（转矩-转速）特性曲线
  - 电机运行工况点分析

- 🔄 **弱磁控制**
  - 自动识别弱磁控制区域
  - 支持恒转矩和恒转速负载类型
  - 弱磁I区和弱磁II区分析

## 项目结构

```
103_FiledCoupleClass/
├── Refactor/                          # 主代码目录
│   ├── pmsm_fcc_calculator.py        # 核心计算器类（数据接口层）
│   ├── test_pmsm_fcc.py              # 测试套件和可视化函数
│   ├── example_usage.py              # CSV数据加载和测试示例
│   └── test_results/                 # 测试结果输出目录
│       ├── *.png                     # 生成的图表
│       └── test_log.txt              # 测试日志
│
├── ExternalValidation/               # 外部验证案例
│   ├── example1/                     # 案例1
│   │   ├── baseParams.csv           # 基本参数
│   │   ├── psid.csv                 # d轴磁链-id曲线
│   │   ├── ld.csv                   # d轴电感-id曲线
│   │   ├── lq.csv                   # q轴电感-iq曲线
│   │   └── test_results/            # 案例1测试结果
│   └── example2/                     # 案例2
│       └── ...                       # 结构同案例1
│
└── verificationOfAlgorithm/          # 算法验证
    └── PMSM_FCC_Verify.ipynb        # Jupyter Notebook验证文档
```

### 核心文件说明

#### 1. `pmsm_fcc_calculator.py` - 核心计算器类（数据接口层）

这是项目的核心计算引擎，**只提供计算方法，不包含任何可视化功能**。主要特点：

- ✅ 纯计算类，可直接被外部程序调用
- ✅ 提供完整的PMSM性能计算API
- ✅ 适合集成到其他系统（仿真软件、控制器开发工具等）
- ✅ 输入输出均为数值数据，便于数据耦合

**适用场景**：
- 需要将PMSM计算功能集成到现有系统
- 批量计算和参数扫描
- 作为后端服务提供计算接口

#### 2. `test_pmsm_fcc.py` - 测试套件和可视化函数

提供完整的测试框架和可视化功能，**不直接运行，而是被其他脚本调用**。主要功能：

- 🧪 单元测试函数（测试各个控制轨迹计算）
- 🧪 集成测试函数（测试完整控制策略）
- 📊 可视化绘图函数（生成各类性能曲线图）
- ⚙️ 测试配置类（`TestConfig`）

**适用场景**：
- 作为测试库被其他脚本导入
- 提供标准化的可视化接口
- 支持自定义测试配置

#### 3. `example_usage.py` - CSV数据加载和测试示例

**完整的端到端示例**，演示如何从CSV文件加载参数并进行测试与可视化：

- 📁 从CSV文件读取电机参数
- 🔧 配置测试工况
- 🚀 调用测试套件执行计算
- 📊 生成完整的测试报告和图表

**适用场景**：
- 快速验证电机参数
- 外部数据源导入测试
- 批量测试多个电机案例

**使用流程**：
```
CSV参数文件 → example_usage.py → 调用pmsm_fcc_calculator.py计算 
                                → 调用test_pmsm_fcc.py可视化
                                → 输出测试结果
```

## 快速开始

### 环境要求

- Python 3.7+
- NumPy
- SciPy
- Matplotlib（仅测试和可视化需要）

### 安装依赖

```bash
# 完整安装（包含可视化）
pip install numpy scipy matplotlib

# 最小安装（仅计算功能）
pip install numpy scipy
```

### 快速决策：我该从哪里开始？

```
您的需求是什么？
│
├─ 🔧 集成到现有系统
│   └─ 使用方法1：直接导入 pmsm_fcc_calculator.py
│
├─ 📁 有CSV格式的电机参数需要测试
│   └─ 使用方法2：运行 example_usage.py
│
└─ 🧪 想快速了解项目功能
    └─ 使用方法3：运行 test_pmsm_fcc.py
```

### 使用方法

根据不同的应用场景，本项目提供三种使用方式：

---

#### 方法1：集成到外部程序（推荐用于系统集成）

**使用 `pmsm_fcc_calculator.py`**，适合将计算功能集成到您的程序中：

```python
import numpy as np
from pmsm_fcc_calculator import PMSMFieldCoupledCalculator

# 准备电机参数
id_range = np.linspace(-300, 100, 61)
iq_range = np.linspace(-300, 300, 71)

# 永磁磁链-id曲线
PsiR_vs_id = np.column_stack([
    id_range, 
    1.0 - 2.0e-4 * id_range - 2.5e-7 * id_range**2
])

# d轴激磁电感-id曲线
Lad_vs_id = np.column_stack([
    id_range, 
    0.006 + (-1.5e-8) * (id_range + 133.33)**2
])

# q轴激磁电感-iq曲线
Laq_vs_iq = np.column_stack([
    iq_range, 
    9e-3 - 8.0e-9 * iq_range**2
])

# 创建计算器实例
calculator = PMSMFieldCoupledCalculator(
    Lad_vs_id=Lad_vs_id,
    Laq_vs_iq=Laq_vs_iq,
    Udc=540.0,              # 母线电压 [V]
    Imax=300.0,             # 最大电流 [A]
    connectType='Y',        # 绕组连接方式
    modulationType='SVPWM', # 调制方式
    polePairs=4,            # 极对数
    Lsigma=0.015e-3,        # 定子漏抗 [H]
    Rs=0.02,                # 定子电阻 [Ω]
    PsiR_vs_id=PsiR_vs_id   # 永磁磁链曲线
)

# 计算MTPA轨迹
mtpa_track = calculator.calc_mtpa_track()

# 生成弱磁控制结果
result = calculator.generate_fw_result(
    controlType='mtpa',     # 控制策略
    temTarget=100.0,        # 目标转矩 [N·m]
    weTarget=500.0,         # 目标电角速度 [rad/s]
    loadType='constTem'     # 负载类型
)
```

> **说明**：此方法仅进行计算，不生成图表。如需可视化，请结合方法2或方法3使用。

---

#### 方法2：从CSV文件快速测试（推荐用于电机参数验证）

**使用 `example_usage.py`**，从CSV文件加载参数并自动完成测试与可视化：

**1. 准备CSV文件**

在测试文件夹中创建以下文件：

- `baseParams.csv` - 基本参数
```csv
Udc,540
Imax,300
connectType,'Y'
modulationType,'SVPWM'
polePairs,4
Lsigma,1.50E-05
Rs,0.02
Tem_target,500
rspeed_target,1000
controlType,'mtpa'
loadType,'constTem'
```

- `psid.csv` - d轴总磁链-id曲线
- `ld.csv` - d轴电感-id曲线
- `lq.csv` - q轴电感-iq曲线

**2. 运行测试**

```bash
cd Refactor

# 使用默认测试文件夹（../ExternalValidation/example1）
python example_usage.py

# 指定测试文件夹
python example_usage.py --folder ../ExternalValidation/example2
```

> **说明**：此方法会自动调用 `pmsm_fcc_calculator.py` 进行计算，并调用 `test_pmsm_fcc.py` 进行可视化。

---

#### 方法3：开发和调试（推荐用于功能开发）

**直接运行 `test_pmsm_fcc.py`**，使用内置的测试参数进行开发和调试：

```bash
cd Refactor
python test_pmsm_fcc.py
```

测试结果将保存在 `test_results/` 目录下，包括：
- 各种性能曲线图（PNG格式）
- 详细的测试日志（test_log.txt）

> **说明**：此方法使用代码中预定义的电机参数，适合算法开发和功能验证。如需修改测试参数，编辑 `test_pmsm_fcc.py` 中的 `TestConfig` 类或 `prepare_test_motor_parameters()` 函数。

## 核心API说明

### PMSMFieldCoupledCalculator 类

#### 初始化参数

| 参数 | 类型 | 说明 |
|------|------|------|
| `Lad_vs_id` | np.ndarray | d轴激磁电感-id曲线 (N×2) |
| `Laq_vs_iq` | np.ndarray | q轴激磁电感-iq曲线 (N×2) |
| `Udc` | float | 变频器母线电压 [V] |
| `Imax` | float | 变频器限幅电流 [A] |
| `connectType` | str | 绕组连接方式 ('Y' 或 'D') |
| `modulationType` | str | 调制方式 ('SVPWM' 或 'SPWM') |
| `polePairs` | int | 极对数 |
| `Lsigma` | float | 定子漏抗 [H] |
| `Rs` | float | 定子相电阻 [Ω] |
| `PsiR_vs_id` | np.ndarray | 永磁磁链-id曲线 (N×2)，可选 |
| `PsiD_vs_id` | np.ndarray | d轴总磁链-id曲线 (N×2)，可选 |

**注意**：`PsiR_vs_id` 和 `PsiD_vs_id` 二选一提供即可。

#### 主要方法

##### 基础计算方法

- `calc_PsiD(id, iq)` - 计算d轴总磁链
- `calc_PsiQ(id, iq)` - 计算q轴总磁链
- `calc_Tem(id, iq)` - 计算电磁转矩
- `calc_Us(id, iq, We)` - 计算定子相电压幅值
- `calc_We(id, iq, Us)` - 计算电角速度

##### 控制轨迹计算

- `calc_id0_track()` - 计算id=0控制轨迹
- `calc_mtpa_track()` - 计算MTPA控制轨迹
- `calc_mtpv_track()` - 计算MTPV控制轨迹
- `calc_I_circle()` - 计算定子电流圆

##### 等值线计算

- `calc_equ_Tem_line(Tem_target)` - 计算等转矩线
- `calc_equ_U_ellipse(We)` - 计算等电压椭圆

##### 综合分析方法

- `generate_nonfw_result()` - 生成非弱磁控制结果
- `generate_fw_result()` - 生成弱磁控制结果
- `calc_motor_characteristics()` - 计算电机特性曲线

## 输出结果说明

### 生成的图表

运行测试后，会在输出目录生成以下图表：

1. **motor_characteristics.png** - 电机特性曲线
   - 转矩-转速曲线
   - 功率-转速曲线
   - 电流-转速曲线
   - 电压-转速曲线

2. **equ_lines_and_circles.png** - 等值线和控制轨迹
   - 电流圆
   - 等电压椭圆
   - 等转矩线
   - MTPA轨迹
   - MTPV轨迹

3. **operating_point_tracks.png** - 运行工况点轨迹
   - 给定工况点在id-iq平面的位置
   - 工况点转速扫描轨迹

4. **nonFw_track.png** - 非弱磁控制轨迹
   - 恒转矩/恒转速负载下的电流轨迹

5. **非弱磁控制策略.png** / **弱磁控制策略.png**
   - 详细的控制策略可视化
   - 包含电流、转矩、电压、转速等多维度信息

6. **current_range.png** - 电流范围分析
   - id和iq的有效工作范围

### 日志文件

`test_log.txt` 包含详细的计算过程和结果，包括：
- 电机参数
- 控制轨迹关键点
- 弱磁区域分析
- 性能指标计算结果

## 算法验证

项目提供了详细的算法验证文档（Jupyter Notebook格式），位于：
```
verificationOfAlgorithm/PMSM_FCC_Verify.ipynb
```

该文档包含：
- 算法原理说明
- 数学公式推导
- 计算步骤验证
- 结果对比分析

## 配置测试参数

测试参数可以通过修改 `test_pmsm_fcc.py` 中的 `TestConfig` 类进行配置：

```python
class TestConfig:
    # 控制策略类型
    CONTROL_TYPE = 'mtpa'        # 'id0' 或 'mtpa'
    
    # 非弱磁控制测试工况
    NON_FW_TEM_TARGET = 500.0    # 目标转矩 [N·m]
    NON_FW_WE_TARGET = 200.0     # 目标电角速度 [rad/s]
    NON_FW_N_TARGET = None       # 目标机械转速 [rpm]
    NON_FW_LOAD_TYPE = 'constTem'  # 'constTem' 或 'constWe'
    
    # 弱磁控制测试工况
    FW_TEM_TARGET = 500.0        # 目标转矩 [N·m]
    FW_WE_TARGET = 500.0         # 目标电角速度 [rad/s]
    FW_N_TARGET = None           # 目标机械转速 [rpm]
    FW_LOAD_TYPE = 'constWe'     # 'constTem' 或 'constWe'
```

## 架构设计

本项目采用分层架构设计，将**计算层**、**测试层**和**应用层**解耦：

```
┌─────────────────────────────────────────────────────────┐
│                   Application Layer                     │
│                                                         │
│  example_usage.py - CSV loader & test orchestration     │
│  ├─ Read CSV parameter files                            │
│  ├─ Configure test scenarios                            │
│  └─ Orchestrate calculation & visualization             │
└──────────────────┬──────────────────────────────────────┘
                   │
        ┌──────────┴──────────┐
        │                     │
        ▼                     ▼
┌──────────────────┐   ┌──────────────────────────┐
│   Test Layer     │   │    Compute Layer         │
│                  │   │                          │
│ test_pmsm_fcc.py │◄──│ pmsm_fcc_calculator.py   │
│                  │   │                          │
│ ├─ Unit tests    │   │ ├─ Motor performance     │
│ ├─ Integration   │   │ ├─ Control trajectories  │
│ └─ Visualization │   │ └─ Iso-lines calculation │
└──────────────────┘   └──────────────────────────┘
```

### 各层职责

| 层次 | 文件 | 职责 | 对外接口 |
|------|------|------|----------|
| **计算层** | `pmsm_fcc_calculator.py` | 纯计算，无可视化 | ✅ 可被外部程序直接调用 |
| **测试层** | `test_pmsm_fcc.py` | 测试和可视化函数库 | ⚠️ 作为库被其他脚本导入 |
| **应用层** | `example_usage.py` | 端到端应用示例 | ✅ 可直接运行 |

### 使用建议

- 🔧 **系统集成**：仅使用 `pmsm_fcc_calculator.py`，在您的程序中直接调用计算方法
- 🧪 **参数验证**：使用 `example_usage.py`，通过CSV文件批量测试不同电机参数
- 🛠️ **功能开发**：修改并运行 `test_pmsm_fcc.py`，快速验证新功能
- 📊 **自定义可视化**：导入 `test_pmsm_fcc.py` 中的绘图函数，组合使用

## 常见问题

**Q: 我应该使用哪个文件/方法？**

A: 根据您的需求选择：
- **外部程序集成**：直接导入 `pmsm_fcc_calculator.py`，调用其计算方法
- **CSV批量测试**：运行 `example_usage.py --folder <your_folder>`
- **快速验证**：运行 `test_pmsm_fcc.py`（使用内置参数）
- **自定义测试**：导入 `test_pmsm_fcc.py` 的函数，编写自己的测试脚本

**Q: 如何选择 PsiR_vs_id 还是 PsiD_vs_id？**

A: EM单机版“交直轴电感计算”生成的是 PsiD（d轴总磁链）查表，程序会自动转换为 PsiR（永磁磁链）查表。如果已知 PsiR，可直接提供。

**Q: 如何批量测试多个工况？**

A: 使用 `example_usage.py` 脚本，通过修改CSV文件中的参数即可快速切换工况。

**Q: 如何在我的程序中集成PMSM计算功能？**

A: 仅需导入 `pmsm_fcc_calculator.py`：
```python
from pmsm_fcc_calculator import PMSMFieldCoupledCalculator

# 创建计算器实例（传入您的电机参数）
calculator = PMSMFieldCoupledCalculator(...)

# 调用计算方法获取结果
mtpa_track = calculator.calc_mtpa_track()
result = calculator.generate_fw_result(...)
```
计算器类不包含任何可视化代码，输入输出都是NumPy数组，便于数据对接。

**Q: 图表中文显示乱码？**

A: 确保系统安装了 SimHei 字体。Windows系统通常已内置，Linux系统可能需要手动安装。

## 贡献指南

欢迎提交问题报告和改进建议！

## 许可证

本项目为 Easimotor 内部软件功能设计项目。

---

**最后更新**：2025年11月22日