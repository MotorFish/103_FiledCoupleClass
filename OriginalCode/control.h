#ifndef CONTROL_H
#define CONTROL_H

#include <QList>
#include <QPointF>
#include <QString>
#include <math.h>

//#define PI 3.1415926

// 控制算法输入参数
struct ControlInput
{
    bool showProgress;
    // 从磁路法能够直接获取的参数
    double f ;      // 频率
    int m ;         // 相数
    int ers ;       // 磁极类型 0为表贴式  1为内埋式
    double Xd ;     // 直轴同步电抗
    QList<QPointF> Iq_vs_Xq ;  // 交轴磁化曲线，内埋式时需要
    double Ld;
    QList<QPointF> LqIq,LdId;   // 交直轴电感曲线
    double Uiv ;    // 感应电压有效值
    int p ;         // 极对数
    double R ;      // 电阻
    double kh ;     // 铁耗系数
    double kc ;
    double ke ;
    double Bt ;     // 齿部磁密
    double Bj ;     // 轭部磁密
    double mt ;     // 齿部质量
    double mj ;     // 轭部质量
    double D1 ;     // 定子外径
    //double Pfw ;    // 风摩损耗
    // 通风损耗, 如果未输入,系统内部自动计算
    double P_wind;
    // 轴承摩擦损耗, 如果未输入,系统内部自动计算
    double P_friction;
    // 参考转速
    double n_reference;

    //2017-3-31新增
    // 杂散损耗系数, 输出功率的倍数
    double K_loss_stray;
    // 铁耗修正系数,（一般填1.0；可根据系列统计规律填大于1或小于1的数据来调整铁耗值）
    double K_loss_Fe_adjust;


    double X1 ;     // 定子漏抗
    // 设置参数
    double Udc ;    // 变频器母线电压
    double Imax ;   // 变频器限幅电流
    int e_control ; // 0为SPWM  1为SVPWM
    static const int SPWM = 0;
    static const int SVPWM = 1;
    // 选择工作点
    int e_work ;    // 工作方式 0为恒转矩负载  1为恒转速负载
    double T_rated ;  // 负载转矩
    double n_rated ;  // 运行转速

    // 计算中间参数
    double phif;

    QString toString() const;

    double calcLd(double Id) const;
    double calcLq(double Lq) const;
};

// 恒转速负载时输出曲线
struct ConstSpeedResult
{
    double maxTm;                   // 恒转速负载时最大转矩
    QList<QPointF> Tem_vs_Tm ;      // 电磁转矩-转矩曲线
    QList<QPointF> Tm_vs_Tm ;       // 输出转矩-转矩曲线
    QList<QPointF> P1_vs_Tm ;       // 输入功率-转矩曲线
    QList<QPointF> Pem_vs_Tm ;      // 电磁功率-转矩曲线
    QList<QPointF> P2_vs_Tm ;       // 输出功率-转矩曲线
    QList<QPointF> Pfe_vs_Tm ;      // 定子铁耗-转矩曲线
    QList<QPointF> Pcu_vs_Tm ;      // 定子铜耗-转矩曲线
    QList<QPointF> Pfw_vs_Tm ;      // 风摩损耗-转矩曲线
    QList<QPointF> Ps_vs_Tm ;       // 附加损耗-转矩曲线
    QList<QPointF> Id_vs_Tm ;       // d轴电流-转矩曲线
    QList<QPointF> Iq_vs_Tm ;       // q轴电流-转矩曲线
    QList<QPointF> ead_vs_Tm ;      // 效率-转矩曲线
    QList<QPointF> cosphi_vs_Tm ;   // 功率因数-转矩曲线
    QList<QPointF> Us_vs_Tm ;       // 相电压-转矩曲线
    QList<QPointF> Is_vs_Tm ;       // 相电流-转矩曲线
    QList<QPointF> theta_vs_Tm ;    // 功角-转矩曲线
    double Tem ;       // 恒转速负载时电磁转矩
    double Tm ;        // 恒转速负载时负载转矩
    double P1 ;        // 恒转速负载时输入功率
    double Pem ;       // 恒转速负载时电磁功率
    double P2 ;        // 恒转速负载时输出功率
    double Pfe ;       // 恒转速负载时铁耗
    double Pfet ;
    double Pfej ;
    double Pcu ;       // 恒转速负载时铜耗
    double Pfw ;       // 恒转速负载时风摩损耗
    double Ps ;        // 恒转速负载时附加损耗
    double Id ;        // 恒转速负载时直轴电流
    double Iq ;        // 恒转速负载时交轴电流
    double ead ;       // 恒转速负载时效率
    double cosphi ;    // 恒转速负载时功率因数
    double Us ;        // 恒转速负载时相电压
    double Is ;        // 恒转速负载时相电流
    double theta ;     // 恒转速负载时功角
    double wm;         // 恒转速负载时转速

    void clear();

};

// 恒转矩负载时输出曲线
struct ConstTorqueResult
{
    double maxWm;                  // 恒转矩负载时最大转速
    QList<QPointF> Tem_vs_n ;      // 电磁转矩-转速曲线
    QList<QPointF> Tm_vs_n ;       // 输出转矩-转速曲线
    QList<QPointF> P1_vs_n ;       // 输入功率-转速曲线
    QList<QPointF> Pem_vs_n ;      // 电磁功率-转速曲线
    QList<QPointF> P2_vs_n ;       // 输出功率-转速曲线
    QList<QPointF> Pfe_vs_n ;      // 定子铁耗-转速曲线
    QList<QPointF> Pcu_vs_n ;      // 定子铜耗-转速曲线
    QList<QPointF> Pfw_vs_n ;      // 风摩损耗-转速曲线
    QList<QPointF> Ps_vs_n ;       // 附加损耗-转速曲线
    QList<QPointF> Id_vs_n ;       // d轴电流-转速曲线
    QList<QPointF> Iq_vs_n ;       // q轴电流-转速曲线
    QList<QPointF> ead_vs_n ;      // 效率-转速曲线
    QList<QPointF> cosphi_vs_n ;   // 功率因数-转速曲线
    QList<QPointF> Us_vs_n ;       // 相电压-转速曲线
    QList<QPointF> Is_vs_n ;       // 相电流-转速曲线
    QList<QPointF> theta_vs_n ;    // 功角-转速曲线
    double Tem ;       // 恒转矩负载时电磁转矩
    double Tm ;        // 恒转矩负载时负载转矩
    double P1 ;        // 恒转矩负载时输入功率
    double Pem ;       // 恒转矩负载时电磁功率
    double P2 ;        // 恒转矩负载时输出功率
    double Pfe ;       // 恒转矩负载时铁耗
    double Pfet ;
    double Pfej ;
    double Pcu ;       // 恒转矩负载时铜耗
    double Pfw ;       // 恒转矩负载时风摩损耗
    double Ps ;        // 恒转矩负载时附加损耗
    double Id ;        // 恒转矩负载时直轴电流
    double Iq ;        // 恒转矩负载时交轴电流
    double ead ;       // 恒转矩负载时效率
    double cosphi ;    // 恒转矩负载时功率因数
    double Us ;        // 恒转矩负载时相电压
    double Is ;        // 恒转矩负载时相电流
    double theta ;     // 恒转矩负载时功角
    double wm;         // 恒转矩负载时转速

    void clear();
};

// 控制算法输出参数
struct ControlOutput
{
    // 输出曲线
    QList<QPointF> Id_vs_Iq ;      // d轴电流-q轴电流曲线
    QList<QPointF> Is_vs_Tem ;     // 电流转矩曲线
    QList<QPointF> Id_vs_Tem ;     // d轴电流转矩曲线
    QList<QPointF> Iq_vs_Tem ;     // q轴电流转矩曲线
    QList<QPointF> Tem_vs_n ;      // 最大电磁转矩-转速曲线
    QList<QPointF> Tm_vs_n ;       // 最大输出转矩-转速曲线
    QList<QPointF> P1_vs_n ;       // 输入功率-转速曲线
    QList<QPointF> Pem_vs_n ;      // 电磁功率-转速曲线
    QList<QPointF> P2_vs_n ;       // 输出功率-转速曲线
    QList<QPointF> Pfe_vs_n ;      // 定子铁耗-转速曲线
    QList<QPointF> Pcu_vs_n ;      // 定子铜耗-转速曲线
    QList<QPointF> Pfw_vs_n ;      // 风摩损耗-转速曲线
    QList<QPointF> Ps_vs_n ;       // 附加损耗-转速曲线
    QList<QPointF> Id_vs_n ;       // d轴电流-转速曲线
    QList<QPointF> Iq_vs_n ;       // q轴电流-转速曲线
    QList<QPointF> ead_vs_n ;      // 效率-转速曲线
    QList<QPointF> cosphi_vs_n ;   // 功率因数-转速曲线
    QList<QPointF> Us_vs_n ;       // 相电压-转速曲线
    QList<QPointF> Is_vs_n ;       // 相电流-转速曲线
    QList<QPointF> theta_vs_n ;    // 功角-转速曲线
    double wmmax ;                 // 最大输出功率时转速
    // 恒转速负载时输出曲线
    ConstSpeedResult n_rated;
    // 恒转矩负载时输出曲线
    ConstTorqueResult T_rated;

    void clear();
};


// 工作点
struct WorkPoint {
    double Iq;
    double Id;
    double wm;
    double f;
    double Pfe, Ptfe, Pjfe;
    double Pcu;
    double Pfw;
    double Ps;
    double ead;
    double cosphi;
    double Us;
    double P1;
    double P2;
    double Tem;
    double Tm;
    double Pem;
    double Is;
    double theta;

    WorkPoint() {
        Iq = 0.0;
        Id = 0.0;
        wm = 0.0;
        f = 0.0;
        Pfe = 0.0;
        Ptfe = 0.0;
        Pjfe = 0.0;
        Pcu = 0.0;
        Pfw = 0.0;
        Ps = 0.0;
        ead = 0.0;
        cosphi = 0.0;
        Us = 0.0;
        P1 = 0.0;
        P2 = 0.0;
        Tem = 0.0;
        Tm = 0.0;
        Pem = 0.0;
        Is = 0.0;
        theta = 0.0;
    }

    // 计算风摩损耗
    static double calcPfw(double wm, double f, const ControlInput &in);
    // 计算铁耗
    double calcPfe(const ControlInput &in);

    // 根据输入的电流等参数计算性能
    void calcWithIdq(double vId, double vIq, double we, const ControlInput &in);
    void calcWithTorque(double Tm, double we, const ControlInput &in);

    // 根据输出转矩、转速计算电磁转矩/功率
    void calcTemWithTorque(double torq, double we, const ControlInput &in);
};

// Id=0永磁同步电机控制算法输入参数
typedef ControlInput IdVector_in;

// Id=0永磁同步电机控制算法输出参数
typedef ControlOutput IdVector_out ;

bool valid_IdVector(const IdVector_in , QString&) ;
bool IdVector(IdVector_in & , IdVector_out& , QString&) ;
bool IdVectorNew(IdVector_in & , IdVector_out& , QString&) ;

// 永磁同步电机最大转矩电流比控制算法输入参数
typedef ControlInput MaxTorque_in;
// 永磁同步电机最大转矩电流比控制算法输出参数
typedef ControlOutput MaxTorque_out;

bool valid_MaxTorque(const MaxTorque_in , QString&) ;
bool MaxTorque(MaxTorque_in &, MaxTorque_out& ,QString&) ;
bool MaxTorqueNew(MaxTorque_in &, MaxTorque_out& ,QString&) ;

typedef struct
{
    double J ;              // 转动惯量
    double mur ;            // 摩擦系数
    double T_end ;          // 结束时间
    //double T_begin ;
    double dltT ;           // 步长
    double TL ;             // 负载转矩
    int p ;                 // 极对数
    double wm0 ;            // 初始转速
    double Uiv ;            // 感应电压有效值
    double f ;              // 频率
    double Xd ;             // d轴电抗
    int ers ;               // 磁极类型 0为表贴式  1为内埋式
    QList<QPointF> Iq_vs_Xq ;  // 交轴磁化曲线，内埋式时需要
    double X1 ;             // 定子漏抗
    double R ;              // 电阻
    double kh ;     // 铁耗系数
    double kc ;
    double ke ;
    double Bt ;     // 齿部磁密
    double Bj ;     // 轭部磁密
    double mt ;     // 齿部质量
    double mj ;     // 轭部质量
    double D1 ;     // 定子外径
    //double Pfw ;    // 风摩损耗
    // 通风损耗, 如果未输入,系统内部自动计算
    double P_wind;
    // 轴承摩擦损耗, 如果未输入,系统内部自动计算
    double P_friction;
    // 参考转速
    double n_reference;

    //2017-3-31新增
    // 杂散损耗系数, 输出功率的倍数
    double K_loss_stray;
    // 铁耗修正系数,（一般填1.0；可根据系列统计规律填大于1或小于1的数据来调整铁耗值）
    double K_loss_Fe_adjust;


    int m ;         // 相数
    QString external_algorithm_name ;  // 外部控制算法名称
    
} Control_in ;

typedef struct
{
    QList<QPointF> Tem_vs_t ;      // 电磁转矩-时间曲线
    QList<QPointF> Tm_vs_t ;       // 输出转矩-时间曲线
    QList<QPointF> P1_vs_t ;       // 输入功率-时间曲线
    QList<QPointF> Pem_vs_t ;      // 电磁功率-时间曲线
    QList<QPointF> P2_vs_t ;       // 输出功率-时间曲线
    QList<QPointF> Pfe_vs_t ;      // 定子铁耗-时间曲线
    QList<QPointF> Pcu_vs_t ;      // 定子铜耗-时间曲线
    QList<QPointF> Pfw_vs_t ;      // 风摩损耗-时间曲线
    QList<QPointF> Ps_vs_t ;       // 附加损耗-时间曲线
    QList<QPointF> Id_vs_t ;       // d轴电流-时间曲线
    QList<QPointF> Iq_vs_t ;       // q轴电流-时间曲线
    QList<QPointF> ead_vs_t ;      // 效率-时间曲线
    QList<QPointF> cosphi_vs_t ;   // 功率因数-时间曲线
    QList<QPointF> theta_vs_t ;    // 功角-时间曲线
    QList<QPointF> Ia_vs_t ;       // a相电流-时间曲线
    QList<QPointF> Ib_vs_t ;       // b相电流-时间曲线
    QList<QPointF> Ic_vs_t ;       // c相电流-时间曲线
    QList<QPointF> Ua_vs_t ;       // a相电压-时间曲线
    QList<QPointF> Ub_vs_t ;       // b相电压-时间曲线
    QList<QPointF> Uc_vs_t ;       // c相电压-时间曲线
    QList<QPointF> Ud_vs_t ;       // d轴电压-时间曲线
    QList<QPointF> Uq_vs_t ;       // q轴电压-时间曲线
    QList<QPointF> phid_vs_t ;     // d轴磁链-时间曲线
    QList<QPointF> phiq_vs_t ;     // q轴磁链-时间曲线
    QList<QPointF> phia_vs_t ;     // a相磁链-时间曲线
    QList<QPointF> phib_vs_t ;     // b相磁链-时间曲线
    QList<QPointF> phic_vs_t ;     // c相磁链-时间曲线
    QList<QPointF> position_vs_t ; // 位置-时间曲线
    QList<QPointF> wm_vs_t ;       // 转速-时间曲线
} Control_out ;

bool valid_Control(const Control_in  , QString &) ;
bool Control(const Control_in , Control_out& , QString& ) ;

// 弱磁控制算法输入输出参数
typedef ControlInput FluxWeaken_in;
typedef ControlOutput FluxWeaken_out;
bool valid_FluxWeaken(const FluxWeaken_in  , QString &) ;
bool FluxWeaken(FluxWeaken_in &, FluxWeaken_out& , QString & ) ;
bool FluxWeakenNew(FluxWeaken_in &, FluxWeaken_out& , QString & ) ;


#endif
