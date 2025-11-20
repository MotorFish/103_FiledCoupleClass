

#include "control.h"
#include "easilogger.h"
#include "findcurvevalue.h"
#include "emmath.h"
#include <QLibrary>
#include <QCoreApplication>
#include <QByteArray>
#include <QTextStream>


#include "log4qt.h"
LOG4QT_DECLARE_STATIC_LOGGER(logger, "TsytOutControl")
#define LOGGER_OUTPUT_VAR(X) if (in.showProgress) logger()->debug() << #X << "=" << X
#define LOGGER_OUTPUT_CONTROLINPUT(X)  if (in.showProgress) logger()->debug() << #X << "=  \n" << X.toString()

#define Cnt1 100
#define Cnt2 100
#define Cnt 1000

//const double PI = 3.1415926535897932;
const double sqrt_2 = 1.4142135623730950488016887242097;
const double sqrt_3 = 1.7320508075688772935274463415059;

void ConstSpeedResult::clear()
{
    Tem_vs_Tm.clear();      // 电磁转矩-转矩曲线
    Tm_vs_Tm.clear();       // 输出转矩-转矩曲线
    P1_vs_Tm.clear();       // 输入功率-转矩曲线
    Pem_vs_Tm.clear();      // 电磁功率-转矩曲线
    P2_vs_Tm.clear();       // 输出功率-转矩曲线
    Pfe_vs_Tm.clear();      // 定子铁耗-转矩曲线
    Pcu_vs_Tm.clear();      // 定子铜耗-转矩曲线
    Pfw_vs_Tm.clear();      // 风摩损耗-转矩曲线
    Ps_vs_Tm.clear();       // 附加损耗-转矩曲线
    Id_vs_Tm.clear();       // d轴电流-转矩曲线
    Iq_vs_Tm.clear();       // q轴电流-转矩曲线
    ead_vs_Tm.clear();      // 效率-转矩曲线
    cosphi_vs_Tm.clear();   // 功率因数-转矩曲线
    Us_vs_Tm.clear();       // 相电压-转矩曲线
    Is_vs_Tm.clear();       // 相电流-转矩曲线
    theta_vs_Tm.clear();    // 功角-转矩曲线
}

void ConstTorqueResult::clear()
{
    Tem_vs_n.clear();      // 电磁转矩-转速曲线
    Tm_vs_n.clear();       // 输出转矩-转速曲线
    P1_vs_n.clear();       // 输入功率-转速曲线
    Pem_vs_n.clear();      // 电磁功率-转速曲线
    P2_vs_n.clear();       // 输出功率-转速曲线
    Pfe_vs_n.clear();      // 定子铁耗-转速曲线
    Pcu_vs_n.clear();      // 定子铜耗-转速曲线
    Pfw_vs_n.clear();      // 风摩损耗-转速曲线
    Ps_vs_n.clear();       // 附加损耗-转速曲线
    Id_vs_n.clear();       // d轴电流-转速曲线
    Iq_vs_n.clear();       // q轴电流-转速曲线
    ead_vs_n.clear();      // 效率-转速曲线
    cosphi_vs_n.clear();   // 功率因数-转速曲线
    Us_vs_n.clear();       // 相电压-转速曲线
    Is_vs_n.clear();       // 相电流-转速曲线
    theta_vs_n.clear();    // 功角-转速曲线
}

void ControlOutput::clear()
{
    Id_vs_Iq.clear();      // d轴电流-q轴电流曲线
    Is_vs_Tem.clear();     // 电流转矩曲线
    Id_vs_Tem.clear();     // d轴电流转矩曲线
    Iq_vs_Tem.clear();     // q轴电流转矩曲线
    Tem_vs_n.clear();      // 最大电磁转矩-转速曲线
    Tm_vs_n.clear();       // 最大输出转矩-转速曲线
    P1_vs_n.clear();       // 输入功率-转速曲线
    Pem_vs_n.clear();      // 电磁功率-转速曲线
    P2_vs_n.clear();       // 输出功率-转速曲线
    Pfe_vs_n.clear();      // 定子铁耗-转速曲线
    Pcu_vs_n.clear();      // 定子铜耗-转速曲线
    Pfw_vs_n.clear();      // 风摩损耗-转速曲线
    Ps_vs_n.clear();       // 附加损耗-转速曲线
    Id_vs_n.clear();       // d轴电流-转速曲线
    Iq_vs_n.clear();       // q轴电流-转速曲线
    ead_vs_n.clear();      // 效率-转速曲线
    cosphi_vs_n.clear();   // 功率因数-转速曲线
    Us_vs_n.clear();       // 相电压-转速曲线
    Is_vs_n.clear();       // 相电流-转速曲线
    theta_vs_n.clear();    // 功角-转速曲线

    n_rated.clear();
    T_rated.clear();
}

// 计算单位铁耗函数
static double calcUnitLossFe(double kh, double kc, double ke, double f, double B)
{
    double ff = f;
    //if (f<=0.0) ff = 50.0;
    double tv = ff * B;
    tv = kh * tv * B + kc * pow(tv, 2.0) + ke * pow(tv, 1.5);
    return tv;
}


double ControlInput::calcLd(double Id) const
{
#if 0
    // 原有的方法:Ld常数值
    return Ld;
#else
    if (LdId.isEmpty()) {
        return Ld;
    }
    else {
        return getYValueByX(Id, LdId);
    }
#endif
}

double ControlInput::calcLq(double Iq) const
{
#if 0
    // 原有的方法: 按表贴式和其他区分
    double Lq;
    if ( ers == 0 )
        Lq = Ld ;
    else
        Lq = (getYValueByX(Iq, Iq_vs_Xq) + X1)/( 2*PI*f ) ;
    return Lq;
#else
    if (!LqIq.isEmpty()) {
        return getYValueByX(Iq, LqIq);
    }
    else if (!Iq_vs_Xq.isEmpty()){
        return (getYValueByX(Iq, Iq_vs_Xq) + X1)/( 2*PI*f ) ;
    }
    else {
        return Ld;
    }
#endif
}

// 计算风摩损耗
double WorkPoint::calcPfw(double wm, double f, const ControlInput& in)
{
    double Pfw;
    if (in.P_wind <= 0.0 && in.P_friction <= 0.0)
    {
        if ( 50*in.p/f < 1.5 )
            Pfw = 1.3*(1-in.D1/100)*pow(3*f/50/in.p,2.0)*pow(in.D1/10,4.0);
        else
            Pfw = pow(3*f/50/in.p,2.0)*pow(in.D1/10,4.0);
    }
    else
    {
        Pfw = in.P_wind  * pow(wm/in.n_reference, 3.0)
        + in.P_friction * pow(wm/in.n_reference, 1.0);
        Pfw *= 1000.0;  // P_wind和P_friction输入单位是kw
    }
    return Pfw;
}

// 计算铁耗
double WorkPoint::calcPfe(const ControlInput& in)
{
    double pfet = calcUnitLossFe(in.kh,in.kc,in.ke,f,in.Bt) ;
    double pfej = calcUnitLossFe(in.kh,in.kc,in.ke,f,in.Bj) ;
    Ptfe = 2.5 * pfet * in.mt;//齿部铁耗
    Pjfe = 2.0 * pfej * in.mj;//轭部铁耗
    if (in.K_loss_Fe_adjust > 0) {
        Ptfe *= in.K_loss_Fe_adjust;
        Pjfe *= in.K_loss_Fe_adjust;
    }
    Pfe = Ptfe + Pjfe ;
    return Pfe;
}

// 工况性能计算
// Id,Iq - 直轴/交轴电流
// we - 电角速度
// in - 输入数据
void WorkPoint::calcWithIdq(double vId, double vIq, double we,
                     const ControlInput& in)
{
    Id = vId;
    Iq = vIq ;
    double It = sqrt(Id*Id+Iq*Iq)/sqrt_2;

    double Ld = in.calcLd(Id);
    double Lq = in.calcLq(Iq);
    double Ud = in.R*Id - we*Lq*Iq ;
    double Uq = in.R*Iq + we*Ld*Id + we*in.phif ;
    Us = sqrt(Ud*Ud+Uq*Uq)/sqrt_2 ;

    wm = 9.55 * we / in.p ;
    f = we / 2 / PI ;

    // 铁耗
    Pfe = calcPfe(in);

    // 铁耗电流（假设与Us同相位）
    double Ife = Pfe / 3 / Us;

    // 转矩电流矢量：I_t = Id + jIq
    // Us矢量：Ud + jUq
    // It_abs为转矩电流模长，Us_abs为电压模长
    double It_abs = sqrt(Id*Id + Iq*Iq);
    double Us_abs = sqrt(Ud*Ud + Uq*Uq);

    // 转矩电流相对于Us的夹角phi_t，余弦值cos_phi_t
    double cos_phi_t = 0.0;
    if(It_abs > 1e-8 && Us_abs > 1e-8) {
        // dot表示点积
        double Us_It_dot = Ud*Id + Uq*Iq;
        cos_phi_t = Us_It_dot / (Us_abs * It_abs);
        // 限定cos_phi_t在[-1,1]
        if (cos_phi_t > 1.0) cos_phi_t = 1.0;
        if (cos_phi_t < -1.0) cos_phi_t = -1.0;
    }

    // 合成电流Is（Ife与Us同相位，即夹角为0，It与Us夹角为phi_t）
    // 合成电流矢量大小
    Is = sqrt(It_abs * It_abs + Ife * Ife + 2 * It_abs * Ife * cos_phi_t);

    // 合成电流与Us的余弦值cosphi（Ife与Us同相，It与Us的夹角为phi_t）
    // Is矢量 = It_abs * exp(j * phi_t) + Ife * exp(j * 0)
    // cosphi = (Is在Us方向上的分量) / Is
    //         = (It_abs * cos(phi_t) + Ife) / Is
    double cosphi = 0.0;
    if(Is > 1e-8) {
        cosphi = (It_abs * cos_phi_t + Ife) / Is;
    }


    // 铜耗
    Pcu = Is * Is * in.R * in.m ;

    //当输入的风损和轴承摩擦损耗都为0时，使用内置算法计算，此时不用对参考转速进行判断
    Pfw = calcPfw(wm, f, in);

    // 杂散损耗
    Ps = ( in.K_loss_stray/(1.0+in.K_loss_stray )) * ( Pem - Pfw );

    Pem = 1.5 * in.p * (in.phif * Iq + (Ld-Lq)*Id*Iq ) * wm / 9.55 ;

    P2 = Pem - Pfw - Ps ;
    P1 = Pem + Pcu + Pfe ;
    ead = P2 / P1 * 100 ;

    Tem = Pem/wm * 9.55 ;
    Tm = P2 * 9.55 / wm ;
    theta = atan(Lq*Iq/in.phif) ;
}
// Id=0下，对指定转矩的dq轴电流计算
void WorkPoint::calcWithTorque(double torq, double we, const ControlInput &in)
{
    calcTemWithTorque(torq, we, in);
    Iq = ( Pem ) /wm * 9.55 / (1.5*in.p*in.phif) ;
    Id = 0 ;
    calcWithIdq(Id, Iq, we, in);
}

// 根据输出转矩、转速计算电磁转矩/功率
void WorkPoint::calcTemWithTorque(double torq, double we, const ControlInput &in)
{
    wm = 9.55 * we / in.p ;
    f = we / 2 / PI ;

    Tm = torq ;          // 给定负载
    P2 = Tm * wm / 9.55 ;

    Ps = in.K_loss_stray * P2 ;
    //当输入的风损和轴承摩擦损耗都为0时，使用内置算法计算，此时不用对参考转速进行判断
    Pfw = calcPfw(wm, f, in);

    Pem = P2 + Ps + Pfw ;

    Pfe = calcPfe(in);

    Tem = Pem/wm * 9.55 ;
}

// 根据电压公式计算给定电压和Id,Iq下的转速(电角速度) {2025.10.21}
// 方程  [L^2_q I^2_q + (L_d I_d + \lambda_f)^2]\omega ^2
//    + [-2R_sI_dL_qI_q+2R_sI_q(L_d I_d + \lambda_f)]\omega
//    + [R^2_sI^2_d + R^2_sI^2_q - U^2] = 0
static double calcOmegaE(double U, double Id, double Iq,
                         const ControlInput& in)
{
    double Ld = in.calcLd(Ld);
    double Lq = in.calcLq(Iq);
    double a = easimath::sqr(Lq*Iq) +  easimath::sqr(Ld*Id + in.phif);
    double b = -2*in.R*Id*Lq*Iq+2*in.R*Iq*(Ld*Id + in.phif);
    double c = easimath::sqr(in.R*Id) + easimath::sqr(in.R*Iq) - U*U;
    double dlt = b*b-4*a*c;
    if (dlt<0) {
        return qSNaN();
    }
    return (sqrt(dlt) - b)/(2*a) ;
}

// // ID=0模式下，当Lq为常数的情况，直接计算给定电压和转速(电角速度)时的Iq电流{2025.10.21}
// // 方程: (L^2_q\omega^2+ R^2_s)I^2_q  + 2R_s\lambda_f\omega I_q +  \lambda_f^2\omega ^2 - U^2 = 0
// static double calcIqWithId0(double U, double we, double Lq, const ControlInput& in)
// {
//     double A = pow(we*Lq,2.0)+pow(in.R,2.0) ;
//     double B = 2 * we * in.phif * in.R ;
//     double C = pow(we*in.phif,2.0) - pow(U,2.0) ;
//     double dlt = B * B - 4 * A * C ;
//     if ( dlt < 0 )
//     {
//         // errmsg = QT_TRANSLATE_NOOP("TsytControl","fail to solve the equation") ;
//         // return false ;
//         return qSNaN();
//     }
//     else
//         return (sqrt(dlt) - B) / (2*A) ;
// }

// ID=0模式下，当Lq-Iq为曲线的情况，迭代计算给定电压和转速(电角速度)时的Iq电流{2025.10.21}
// 方程: L^2_q I^2_q\omega ^2 + (\lambda_f\omega  + R_sI_q)^2 - U^2_{max} = 0
static double findIqWithId0(double U, double we, const ControlInput& in)
{
    // 1. 定义一个 lambda（“内嵌函数”）
    auto func = [we, U, in](double Lq, double Iq) -> double
    {
        return pow(we*Lq*Iq,2.0)+pow(in.R*Iq+we*in.phif,2.0)-pow(U,2.0);
    };

    double Iqa = 0 ;
    double Iqb = in.Imax ;
    if ( func(0, 0) > 0 )
    {
        return qSNaN();
    }
    double Lq = in.calcLq(Iqb) ;
    if ( func(Lq, Iqb) < 0 )
    {
        return qSNaN();
    }
    double Iqc ;
    int Iq_Loop = 0 ;
    do
    {
        Iq_Loop++ ;
        Iqc = 0.5 * ( Iqa + Iqb ) ;
        Lq = in.calcLq(Iqc);
        if ( func(Lq, Iqc) > 0 )
            Iqb = Iqc ;
        else
            Iqa = Iqc ;
        if ( fabs(Iqa-Iqb)/Iqc < 0.005 )
            break ;
    }while(Iq_Loop<50) ;

    if ( Iq_Loop >= 50 )
        return qSNaN();
    return Iqc;
}

// 计算MTPA的轨迹
static void calcMTPATrace(const MaxTorque_in &in, MaxTorque_out& out)
{
    double Itemp , Id , Iq , Tem , Idmax , Iqmax ,Tmax ;
    double Ld , Lq ;
    QList<QPointF> Id_vs_Iq ;

    for ( int i = 0 ; i <= Cnt ; i++ )
    {
        Tmax = 0.0 ;
        Itemp = in.Imax * i / Cnt ;
        for ( int j = -Cnt ; j <= 0 ; j++ )
        {
            Id = Itemp * j / Cnt ;
            Iq = sqrt(pow(Itemp,2.0) - pow(Id,2.0)) ;

            Ld = in.calcLd(Id);
            Lq = in.calcLq(Iq);
            Tem = 1.5*in.p*(in.phif*Iq+(Ld-Lq)*Id*Iq) ;
            if ( Tem >= Tmax )
            {
                Tmax = Tem ;
                Idmax = Id ;
                Iqmax = Iq ;
            }
        }
        out.Is_vs_Tem.append(QPointF(Tmax,Itemp));
        out.Id_vs_Tem.append(QPointF(Tmax,Idmax));
        out.Iq_vs_Tem.append(QPointF(Tmax,Iqmax));
        Id_vs_Iq.append(QPointF(Idmax,Iqmax));
    }

    // // 去除两个相同值的连续点?? 对表贴式这里会丢掉数据，计算失败
    for ( int i = 0 ; i < Id_vs_Iq.count() ; i++)
    {
        //double Id = Id_vs_Iq.at(Id_vs_Iq.count()-i-1).x() ;
        //double Iq = Id_vs_Iq.at(Id_vs_Iq.count()-i-1).y() ;
        if ( i<Id_vs_Iq.count()-1 )
        {
            if (Id_vs_Iq.at(Id_vs_Iq.count()-i-1).x()==Id_vs_Iq.at(Id_vs_Iq.count()-i-2).x() )
            {
                continue ;
            }
        }
        out.Id_vs_Iq.append(Id_vs_Iq.at(Id_vs_Iq.count()-i-1)) ;
    }

}

// 查找弱磁模式下给定Umax和Imax、we时的Iq
static double findIqWithFluxWeaken(double U, double we,
                                   const ControlInput & in )
{
    // 1. 定义一个 lambda（“内嵌函数”）
    auto func = [we, U, in](double Ld, double Id, double Lq, double Iq) -> double
    {
        return pow(in.R*Id - we*Lq*Iq,2.0)+pow(in.R*Iq+we*(Ld*Id+in.phif),2.0)-pow(U,2.0);
    };

    double Iqa = 0 , Iqb = in.Imax ;
    double Ida = -in.Imax, Idb = 0;

    double Ld = in.calcLd(Ida);
    double Lq = in.calcLq(Iqb);

    double fa = func(Ld, Ida, 0, Iqa) ;
    //double fb = func(0, Idb, Lq, Iqb);
    double Iqc , fc ;
    int IqLoop = 0 ;
    do
    {
        IqLoop ++ ;
        Iqc = 0.5 * ( Iqa + Iqb ) ;
        Lq  = in.calcLq(Iqc);
        double Id = -sqrt(in.Imax*in.Imax-Iqc*Iqc) ;
        Ld = in.calcLd(Id);
        fc = func(Ld, Id, Lq, Iqc) ;
        if ( fabs(fc) < 0.005 )
            break ;
        if ( fc * fa > 0 )
        {
            Iqa = Iqc ;
        }
        else
        {
            Iqb = Iqc ;
        }
    }while(IqLoop<50) ;

    //修改迭代失败，曲线异常翘尾问题 20250625
    if(IqLoop >=50){
        return qSNaN();
    }

    return Iqc ;
}

bool IdVectorNew(IdVector_in & in , IdVector_out& out , QString &errmsg)
{
    // 打印输入参数
    logger()->debug() << __FUNCTION__;
    LOGGER_OUTPUT_CONTROLINPUT(in);
    // 参数校验
    if ( !(valid_IdVector(in,errmsg)) )
        return false ;

    // 等效最大电压
    double Umax ;
    if ( in.e_control == in.SPWM )
        Umax = in.Udc/2.0 ;
    else
        Umax = in.Udc/sqrt_3 ;
    LOGGER_OUTPUT_VAR(Umax);

    // 空载时的永磁体磁链
    in.phif = in.Uiv * sqrt_2 / ( 2*PI*in.f ) ;
    LOGGER_OUTPUT_VAR(in.Uiv);
    LOGGER_OUTPUT_VAR(in.f);
    LOGGER_OUTPUT_VAR(in.phif);

    // 第一点, 转速=0
    double we1 = 0.0;

    // 第二点，转折点，电压达到最大电压，计算转折转速
    double Iq2 = in.Imax ;
    double we2 = calcOmegaE(Umax, 0., Iq2, in);
    double wm2 = we2 * 9.55 / in.p ;


    // 计算第三个点
    //double wmax = Umax / in.phif * 9.55 / in.p ;
    double we3 = Umax / in.phif;

    // 开始扫描
    errmsg.clear();
    out.clear() ;

    double Iq , Lq , wm, we ;
    double P2max = 0.0 ;
    for ( int i = 1 ; i < Cnt1+Cnt2 ; i++ )
    {
        // 从第一个点到第二个点扫描
        if ( i <= Cnt1 )
        {
            Iq = in.Imax ;
            we = we1 + (we2-we1) * i / Cnt1 ;
        }
        // 从第二个点到第三个点扫描
        else
        {
            we = we2 + (we3 - we2) * (i-Cnt1)/Cnt2 ;
            // if ( in.ers == 0 )
            // {
            //     Lq = in.Xd / (2*PI*in.f) ;
            //     Iq = calcIqWithId0(Umax, we, Lq, in);
            //     if ( qIsNaN(Iq) )
            //     {
            //         errmsg = QT_TRANSLATE_NOOP("TsytControl","fail to solve the equation") ;
            //         return false ;
            //     }
            // }
            // else
            // {
            Iq = findIqWithId0(Umax, we, in);
            if ( qIsNaN(Iq) )
            {
                errmsg = QT_TRANSLATE_NOOP("TsytControl","fail to solve the equation") ;
                return false ;
            }
            // }
        }

        WorkPoint wp;
        wp.calcWithIdq(0, Iq, we, in);
        // 记录最大功率
        if ( wp.P2 > P2max )
        {
            out.wmmax = wp.wm ;
            P2max = wp.P2 ;
        }
        if ( wp.P2 > 0 )
        {
            out.cosphi_vs_n.append(QPointF(wp.wm, wp.cosphi)) ;
            out.ead_vs_n.append(QPointF(wp.wm, wp.ead));
            out.Id_vs_n.append(QPointF(wp.wm, wp.Id));
            out.Iq_vs_n.append(QPointF(wp.wm, wp.Iq));
            out.Is_vs_n.append(QPointF(wp.wm, wp.Is));
            out.P1_vs_n.append(QPointF(wp.wm, wp.P1*1e-3));
            out.P2_vs_n.append(QPointF(wp.wm, wp.P2*1e-3));
            out.Pcu_vs_n.append(QPointF(wp.wm, wp.Pcu*1e-3));
            out.Pem_vs_n.append(QPointF(wp.wm, wp.Pem*1e-3));
            out.Pfe_vs_n.append(QPointF(wp.wm, wp.Pfe*1e-3));
            out.Pfw_vs_n.append(QPointF(wp.wm, wp.Pfw*1e-3));
            out.Ps_vs_n.append(QPointF(wp.wm, wp.Ps*1e-3));
            out.Tem_vs_n.append(QPointF(wp.wm, wp.Tem));
            out.Tm_vs_n.append(QPointF(wp.wm, wp.Tm));
            out.Us_vs_n.append(QPointF(wp.wm, wp.Us));
            out.theta_vs_n.append(QPointF(wp.wm, wp.theta*180/PI));
        }
    }

    if (out.Tm_vs_n.size()==0) {
        errmsg = QT_TRANSLATE_NOOP("TsytControl","Cannot get the rated point in characteristic curve.") ;
        return false;
    }

    // 恒转矩负载
    if ( in.e_work == 0 )
    {
        // 当Tm=Trated时寻找相应的wm
        bool flag = false ;
        double wTm = getRootValueByY( in.T_rated , out.Tm_vs_n , &flag ) ;
        if ( !flag ) {
            if ( in.T_rated > out.Tm_vs_n.at(0).y() )
            {
                // 额定转矩大于电机可输出的最大转矩，计算结果中将输出转折转速时的性能曲线
                errmsg = QT_TRANSLATE_NOOP("TsytControl","The rated torque is greater than the maximum torque the motor can output, and the performance curve at the inflection point speed will be output in the calculation results.");
                wTm = wm2 ;
                in.T_rated = getYValueByX(wm2,out.Tm_vs_n);
            }
            else if ( in.T_rated < out.Tm_vs_n.last().y() )
            {
                double T1 , T2 , w1 , w2 ;
                int count = out.Tm_vs_n.count() ;
                T1 = out.Tm_vs_n.last().y() ;
                w1 = out.Tm_vs_n.last().x() ;
                T2 = out.Tm_vs_n.at(count-2).y() ;
                w2 = out.Tm_vs_n.at(count-2).x() ;
                wTm = w2 + (w2-w1)/(T2-T1)*(in.T_rated-T2) ;
            }
        }
        out.T_rated.maxWm = wTm;

        // 从0-wTm对wm进行扫描
        for ( int i = 1 ; i <= Cnt1 ; i++ )
        {
            wm = wTm * i / Cnt1 ;
            we = wm * in.p / 9.55 ;

            WorkPoint wp;
            wp.calcWithTorque(in.T_rated, we, in);

            out.T_rated.cosphi_vs_n.append(QPointF(wp.wm, wp.cosphi)) ;
            out.T_rated.ead_vs_n.append(QPointF(wp.wm, wp.ead));
            out.T_rated.Id_vs_n.append(QPointF(wp.wm, wp.Id));
            out.T_rated.Iq_vs_n.append(QPointF(wp.wm, wp.Iq));
            out.T_rated.Is_vs_n.append(QPointF(wp.wm, wp.Is));
            out.T_rated.P1_vs_n.append(QPointF(wp.wm, wp.P1*1e-3));
            out.T_rated.P2_vs_n.append(QPointF(wp.wm, wp.P2*1e-3));
            out.T_rated.Pcu_vs_n.append(QPointF(wp.wm, wp.Pcu*1e-3));
            out.T_rated.Pem_vs_n.append(QPointF(wp.wm, wp.Pem*1e-3));
            out.T_rated.Pfe_vs_n.append(QPointF(wp.wm, wp.Pfe*1e-3));
            out.T_rated.Pfw_vs_n.append(QPointF(wp.wm, wp.Pfw*1e-3));
            out.T_rated.Ps_vs_n.append(QPointF(wp.wm, wp.Ps*1e-3));
            out.T_rated.Tem_vs_n.append(QPointF(wp.wm, wp.Tem));
            out.T_rated.Tm_vs_n.append(QPointF(wp.wm, wp.Tm));
            out.T_rated.Us_vs_n.append(QPointF(wp.wm, wp.Us));
            out.T_rated.theta_vs_n.append(QPointF(wp.wm, wp.theta*180/PI));
        }


        // 当wm为运行转速时，计算各工作点特性
        ///////////////////////////////////////////////////////////////////////////
        wm = in.n_rated ;
        if (in.n_rated > out.T_rated.maxWm) {
            // 在当前负载转矩下，电机无法达到额定转速，输出的额定工况为最大转速下的电机性能。
            errmsg += QT_TRANSLATE_NOOP("TsytControl","Under the current load torque, the motor cannot reach the rated speed, and the rated operating condition is the motor performance at the maximum speed.") ;
            wm = out.T_rated.maxWm;
        }
        we = wm * in.p / 9.55 ;
        WorkPoint wp;
        wp.calcWithTorque(in.T_rated, we, in);

        out.T_rated.cosphi = wp.cosphi ;
        out.T_rated.ead = wp.ead ;
        out.T_rated.Id = wp.Id ;
        out.T_rated.Iq = wp.Iq ;
        out.T_rated.Is = wp.Is ;
        out.T_rated.P1 = wp.P1 ;
        out.T_rated.P2 = wp.P2 ;
        out.T_rated.Pcu = wp.Pcu ;
        out.T_rated.Pem = wp.Pem ;
        out.T_rated.Pfe = wp.Pfe ;
        out.T_rated.Pfw = wp.Pfw ;
        out.T_rated.Ps = wp.Ps ;
        out.T_rated.Tem = wp.Tem ;
        out.T_rated.Tm = wp.Tm ;
        out.T_rated.Us = wp.Us ;
        out.T_rated.Pfet = wp.Ptfe;
        out.T_rated.Pfej = wp.Pjfe ;
        out.T_rated.theta = wp.theta ;
        out.T_rated.wm = wp.wm;
        ///////////////////////////////////////////////////////////////////////////////////
        // 工作特性计算结束
    }
    else
    {
        // 恒转速负载
        // Tm_vs_n曲线的最大转速
        double Nmax = out.Tm_vs_n.last().x();
        double Nrated = in.n_rated;
        if (Nrated>Nmax) {
            // 额定转速大于电机可达到的最大转速，计算结果中将输出转折转速时的性能曲线
            errmsg = QT_TRANSLATE_NOOP("TsytControl","The rated speed is greater than the maximum speed the motor can achieve, and the performance curve at the inflection point speed will be output in the calculation results.");
            Nrated = wm2;
        }

        // 根据Tm_vs_n曲线，找到额定转速下的Tmax
        double Tmax = getYValueByX( Nrated , out.Tm_vs_n ) ;
        out.n_rated.maxTm = Tmax;
        // 对Tm进行0-Tmax之间的扫描
        wm = Nrated ;          // 恒转速
        we = wm * in.p / 9.55 ;
        for ( int i = 0 ; i <= Cnt1 ; i++ )
        {
            //Tm = Tmax * i / Cnt1 ;

            WorkPoint wp;
            wp.calcWithTorque(Tmax * i / Cnt1, we, in);

            out.n_rated.cosphi_vs_Tm.append(QPointF(wp.Tm, wp.cosphi)) ;
            out.n_rated.ead_vs_Tm.append(QPointF(wp.Tm, wp.ead));
            out.n_rated.Id_vs_Tm.append(QPointF(wp.Tm, wp.Id)) ;
            out.n_rated.Iq_vs_Tm.append(QPointF(wp.Tm, wp.Iq));
            out.n_rated.Is_vs_Tm.append(QPointF(wp.Tm, wp.Is));
            out.n_rated.P1_vs_Tm.append(QPointF(wp.Tm, wp.P1*1e-3));
            out.n_rated.P2_vs_Tm.append(QPointF(wp.Tm, wp.P2*1e-3));
            out.n_rated.Pcu_vs_Tm.append(QPointF(wp.Tm, wp.Pcu*1e-3));
            out.n_rated.Pem_vs_Tm.append(QPointF(wp.Tm, wp.Pem*1e-3));
            out.n_rated.Pfe_vs_Tm.append(QPointF(wp.Tm, wp.Pfe*1e-3));
            out.n_rated.Pfw_vs_Tm.append(QPointF(wp.Tm, wp.Pfw*1e-3));
            out.n_rated.Ps_vs_Tm.append(QPointF(wp.Tm, wp.Ps*1e-3));
            out.n_rated.Tem_vs_Tm.append(QPointF(wp.Tm, wp.Tem));
            out.n_rated.Tm_vs_Tm.append(QPointF(wp.Tm, wp.Tm));
            out.n_rated.Us_vs_Tm.append(QPointF(wp.Tm, wp.Us)) ;
            out.n_rated.theta_vs_Tm.append(QPointF(wp.Tm, wp.theta*180/PI));
        }

        // 计算恒转速负载时额定转矩下的各工作特性
        ///////////////////////////////////////////////////////////////////
        double Tm = in.T_rated ;          // 额定转矩
        if (Tm>Tmax) {
            // 在当前额定转速下，电机无法输出额定的负载转矩，输出的额定工况为该转速输出最大转矩时的电机性能。
            errmsg += QT_TRANSLATE_NOOP("TsytControl","At the current rated speed, the motor cannot output the rated load torque, and the rated operating condition is the motor performance when the maximum torque is output at this speed.");
            Tm = Tmax;
        }
        WorkPoint wp;
        wp.calcWithTorque(Tm, we, in);

        out.n_rated.cosphi = wp.cosphi ;
        out.n_rated.ead = wp.ead ;
        out.n_rated.Id = wp.Id ;
        out.n_rated.Iq = wp.Iq ;
        out.n_rated.Is = wp.Is ;
        out.n_rated.P1 = wp.P1 ;
        out.n_rated.P2 = wp.P2 ;
        out.n_rated.Pcu = wp.Pcu ;
        out.n_rated.Pem = wp.Pem ;
        out.n_rated.Pfe = wp.Pfe ;
        out.n_rated.Pfw = wp.Pfw ;
        out.n_rated.Ps = wp.Ps ;
        out.n_rated.Tem = wp.Tem ;
        out.n_rated.Tm = wp.Tm ;
        out.n_rated.Us = wp.Us ;
        out.n_rated.Pfet = wp.Ptfe ;
        out.n_rated.Pfej = wp.Pjfe ;
        out.n_rated.theta = wp.theta ;
        out.n_rated.wm = wp.wm;
        ///////////////////////////////////////////////////////////////////
        // 恒转速负载时额定转矩下的各工作特性计算结束
    }

    return true ;
}

bool MaxTorqueNew(MaxTorque_in &in , MaxTorque_out& out ,QString& errmsg )
{
    logger()->debug() << __FUNCTION__;
    LOGGER_OUTPUT_CONTROLINPUT(in);

    if ( !valid_MaxTorque(in,errmsg) )
        return false ;

    // 等效最大电压
    double Umax ;
    if ( in.e_control == in.SPWM )
        Umax = in.Udc/2 ;
    else
        Umax = in.Udc/sqrt_3 ;
    LOGGER_OUTPUT_VAR(Umax);

    // 空载时的永磁体磁链
    in.phif = in.Uiv * sqrt_2 / ( 2*PI*in.f ) ;

    // 扫描得到MTPA的轨迹
    double Id , Iq , Tem ;
    //double Ld , Lq ;
    out.clear() ;
    calcMTPATrace(in, out);

    // 计算第一个点
    double wm1, wm2, wm3;

    wm1 = 0;

    {
        // 最大电流对应的最大转矩
        double Tem1 = getXValueByY(in.Imax,out.Is_vs_Tem,true);
        double Id1 = getYValueByX(Tem1,out.Id_vs_Tem,true);
        double Iq1 = getYValueByX(Tem1,out.Iq_vs_Tem,true);

        // 计算第二个点

        double Iq2 , Id2 ;
        //double Pfe2 , Pcu2 , Pfw2 , Ps2 , ead2 ,  P12 , P22 , Tem2 , Tm2 , Pem2  ;
        Iq2 = Iq1 ;
        Id2 = Id1 ;

        // 求解一元二次方程求解we2
        double we2 = calcOmegaE(Umax, Id2, Iq2, in) ;
        wm2 = 9.55 * we2 / in.p ;
    }


    {
        // 计算第三个点
        double wmax = Umax / in.phif * 9.55 / in.p ;
        wm3 = wmax ;
    }
    // 开始扫描外特性曲线

    double wm , we , Tm;
    double P2max = 0.0 ;
    // 第一段恒转矩部分(Tem和电流不变)
    Tem = getXValueByY(in.Imax,out.Is_vs_Tem) ;
    Id = getYValueByX(Tem,out.Id_vs_Tem) ;
    Iq = getYValueByX(Tem,out.Iq_vs_Tem) ;
    for ( int i = 1 ; i < Cnt1+Cnt2 ; i++ )
    {
        // 从第一个点到第二个点扫描
        if ( i <= Cnt1 )
        {
            wm = wm1 + (wm2-wm1) * i / Cnt1 ;
            we = wm * in.p / 9.55 ;
        }
        // 从第二个点到第三个点扫描
        else
        {
            wm = wm2 + (wm3 - wm2) * (i-Cnt1)/Cnt2 ;
            we = wm * in.p / 9.55 ;

            // 达到最大电压，扫描Id,在MTPA轨迹上查找Iq,要求匹配最大转速等于we
            double we_temp ;
            Id = 0 ;
            double fb = 1 ;
            do
            {
                Iq = getYValueByX(Id,out.Id_vs_Iq) ;
                we_temp = calcOmegaE(Umax, Id, Iq, in) ;
                if ( we < we_temp )
                {
                    if ( fabs(we-we_temp)/we < 0.005 )
                    {
                        break ;
                    }
                    else
                    {
                        Id -= in.Imax/Cnt * fb ;
                    }
                }
                else
                {
                    if ( fabs(we-we_temp)/we < 0.005 )
                    {
                        break ;
                    }
                    else
                    {
                        Id += in.Imax/Cnt * fb ;
                        fb *= 0.1 ;
                        Id -= in.Imax/Cnt * fb ;
                    }
                }

            }while(fb>0.0001) ;
        }

        // 计算性能
        WorkPoint wp;
        wp.calcWithIdq(Id, Iq, we, in);

        if ( wp.P2 > P2max )
        {
            out.wmmax = wp.wm ;
            P2max = wp.P2 ;
        }

        if ( wp.P2 > 0 )
        {
            out.cosphi_vs_n.append(QPointF(wp.wm, wp.cosphi)) ;
            out.ead_vs_n.append(QPointF(wp.wm, wp.ead));
            out.Id_vs_n.append(QPointF(wp.wm, wp.Id));
            out.Iq_vs_n.append(QPointF(wp.wm, wp.Iq));
            out.Is_vs_n.append(QPointF(wp.wm, wp.Is));
            out.P1_vs_n.append(QPointF(wp.wm, wp.P1*1e-3));
            out.P2_vs_n.append(QPointF(wp.wm, wp.P2*1e-3));
            out.Pcu_vs_n.append(QPointF(wp.wm, wp.Pcu*1e-3));
            out.Pem_vs_n.append(QPointF(wp.wm, wp.Pem*1e-3));
            out.Pfe_vs_n.append(QPointF(wp.wm, wp.Pfe*1e-3));
            out.Pfw_vs_n.append(QPointF(wp.wm, wp.Pfw*1e-3));
            out.Ps_vs_n.append(QPointF(wp.wm, wp.Ps*1e-3));
            out.Tem_vs_n.append(QPointF(wp.wm, wp.Tem));
            out.Tm_vs_n.append(QPointF(wp.wm, wp.Tm));
            out.Us_vs_n.append(QPointF(wp.wm, wp.Us));
            out.theta_vs_n.append(QPointF(wp.wm, wp.theta*180/PI));
        }
    }

    if (out.Tm_vs_n.size()==0) {
        errmsg = QT_TRANSLATE_NOOP("TsytControl","Cannot get the rated point in characteristic curve.") ;
        return false;
    }

    // 恒转矩负载
    if ( in.e_work == 0 )
    {
        // 当Tm=Trated时寻找相应的wm
        bool flag = false;
        double wTm = getRootValueByY( in.T_rated , out.Tm_vs_n , &flag ) ;
        if ( !flag ) {
            if ( in.T_rated > out.Tm_vs_n.at(0).y() )
            {
                // 额定转矩大于电机可输出的最大转矩，计算结果中将输出转折转速时的性能曲线
                errmsg = QT_TRANSLATE_NOOP("TsytControl","The rated torque is greater than the maximum torque the motor can output, and the performance curve at the inflection point speed will be output in the calculation results.");
                wTm = wm2 ;
                in.T_rated = getYValueByX(wm2,out.Tm_vs_n);
            }
            else if ( in.T_rated < out.Tm_vs_n.last().y() )
            {
                double T1 , T2 , w1 , w2 ;
                int count = out.Tm_vs_n.count() ;
                T1 = out.Tm_vs_n.last().y() ;
                w1 = out.Tm_vs_n.last().x() ;
                T2 = out.Tm_vs_n.at(count-2).y() ;
                w2 = out.Tm_vs_n.at(count-2).x() ;
                wTm = w2 + (w2-w1)/(T2-T1)*(in.T_rated-T2) ;
            }
        }
        out.T_rated.maxWm = wTm;

        // 从0-wTm对wm进行扫描
        for ( int i = 1 ; i <= Cnt1 ; i++ )
        {
            wm = wTm * i / Cnt1 ;
            we = wm * in.p / 9.55 ;
            //Tm = in.T_rated ;          // 恒转矩负载

            WorkPoint wp;
            wp.calcTemWithTorque(in.T_rated, we, in);
            // 根据MTPA算法确定IdIq
            double Iq = getYValueByX(wp.Tem,out.Iq_vs_Tem);
            double Id = getYValueByX(wp.Tem,out.Id_vs_Tem);
            wp.calcWithIdq(Id, Iq, we, in);

            out.T_rated.cosphi_vs_n.append(QPointF(wp.wm, wp.cosphi)) ;
            out.T_rated.ead_vs_n.append(QPointF(wp.wm, wp.ead));
            out.T_rated.Id_vs_n.append(QPointF(wp.wm, wp.Id));
            out.T_rated.Iq_vs_n.append(QPointF(wp.wm, wp.Iq));
            out.T_rated.Is_vs_n.append(QPointF(wp.wm, wp.Is));
            out.T_rated.P1_vs_n.append(QPointF(wp.wm, wp.P1*1e-3));
            out.T_rated.P2_vs_n.append(QPointF(wp.wm, wp.P2*1e-3));
            out.T_rated.Pcu_vs_n.append(QPointF(wp.wm, wp.Pcu*1e-3));
            out.T_rated.Pem_vs_n.append(QPointF(wp.wm, wp.Pem*1e-3));
            out.T_rated.Pfe_vs_n.append(QPointF(wp.wm, wp.Pfe*1e-3));
            out.T_rated.Pfw_vs_n.append(QPointF(wp.wm, wp.Pfw*1e-3));
            out.T_rated.Ps_vs_n.append(QPointF(wp.wm, wp.Ps*1e-3));
            out.T_rated.Tem_vs_n.append(QPointF(wp.wm, wp.Tem));
            out.T_rated.Tm_vs_n.append(QPointF(wp.wm, wp.Tm));
            out.T_rated.Us_vs_n.append(QPointF(wp.wm, wp.Us));
            out.T_rated.theta_vs_n.append(QPointF(wp.wm, wp.theta*180/PI));
        }

        ///////////////////////////////////////////////////////////////////////////
        // 当wm为运行转速时，计算各工作点特性
        wm = in.n_rated ;
        if (in.n_rated > out.T_rated.maxWm) {
            // 在当前负载转矩下，电机无法达到额定转速，输出的额定工况为最大转速下的电机性能。
            errmsg += QT_TRANSLATE_NOOP("TsytControl","Under the current load torque, the motor cannot reach the rated speed, and the rated operating condition is the motor performance at the maximum speed.") ;
            wm = out.T_rated.maxWm;
        }

        we = wm * in.p / 9.55 ;

        WorkPoint wp;
        wp.calcTemWithTorque(in.T_rated, we, in);
        // 根据MTPA算法确定IdIq
        double Iq = getYValueByX(wp.Tem,out.Iq_vs_Tem);
        double Id = getYValueByX(wp.Tem,out.Id_vs_Tem);
        wp.calcWithIdq(Id, Iq, we, in);

        out.T_rated.cosphi = wp.cosphi ;
        out.T_rated.ead = wp.ead ;
        out.T_rated.Id = wp.Id ;
        out.T_rated.Iq = wp.Iq ;
        out.T_rated.Is = wp.Is ;
        out.T_rated.P1 = wp.P1 ;
        out.T_rated.P2 = wp.P2 ;
        out.T_rated.Pcu = wp.Pcu ;
        out.T_rated.Pem = wp.Pem ;
        out.T_rated.Pfe = wp.Pfe ;
        out.T_rated.Pfw = wp.Pfw  ;
        out.T_rated.Ps = wp.Ps ;
        out.T_rated.Tem = wp.Tem ;
        out.T_rated.Tm = wp.Tm ;
        out.T_rated.Us = wp.Us ;
        out.T_rated.Pfet = wp.Ptfe ;
        out.T_rated.Pfej = wp.Pjfe ;
        out.T_rated.theta = wp.theta ;
        out.T_rated.wm = wp.wm;
        ///////////////////////////////////////////////////////////////////////////////////
        // 工作特性计算结束
    }
    else
    {
        // 恒转速负载
        // Tm_vs_n曲线的最大转速
        double Nmax = out.Tm_vs_n.last().x();
        double Nrated = in.n_rated;
        if (Nrated>Nmax) {
            // 额定转速大于电机可达到的最大转速，计算结果中将输出转折转速时的性能曲线
            errmsg = QT_TRANSLATE_NOOP("TsytControl","The rated speed is greater than the maximum speed the motor can achieve, and the performance curve at the inflection point speed will be output in the calculation results.");
            Nrated = wm2;
        }

        // 根据Tm_vs_n曲线，找到额定转速下的Tmax
        double Tmax = getYValueByX( in.n_rated , out.Tm_vs_n ) ;
        out.n_rated.maxTm = Tmax;

        // 对Tm进行0-Tmax之间的扫描
        wm = in.n_rated ;          // 恒转速
        we = wm * in.p / 9.55 ;
        for ( int i = 1 ; i <= Cnt1 ; i++ )
        {
            Tm = Tmax * i / Cnt1 ;
            WorkPoint wp;
            wp.calcTemWithTorque(Tm, we, in);
            // 根据MTPA算法确定IdIq
            double Iq = getYValueByX(wp.Tem,out.Iq_vs_Tem);
            double Id = getYValueByX(wp.Tem,out.Id_vs_Tem);
            wp.calcWithIdq(Id, Iq, we, in);

            out.n_rated.cosphi_vs_Tm.append(QPointF(wp.Tm, wp.cosphi)) ;
            out.n_rated.ead_vs_Tm.append(QPointF(wp.Tm, wp.ead));
            out.n_rated.Id_vs_Tm.append(QPointF(wp.Tm, wp.Id)) ;
            out.n_rated.Iq_vs_Tm.append(QPointF(wp.Tm, wp.Iq));
            out.n_rated.Is_vs_Tm.append(QPointF(wp.Tm, wp.Is));
            out.n_rated.P1_vs_Tm.append(QPointF(wp.Tm, wp.P1*1e-3));
            out.n_rated.P2_vs_Tm.append(QPointF(wp.Tm, wp.P2*1e-3));
            out.n_rated.Pcu_vs_Tm.append(QPointF(wp.Tm, wp.Pcu*1e-3));
            out.n_rated.Pem_vs_Tm.append(QPointF(wp.Tm, wp.Pem*1e-3));
            out.n_rated.Pfe_vs_Tm.append(QPointF(wp.Tm, wp.Pfe*1e-3));
            out.n_rated.Pfw_vs_Tm.append(QPointF(wp.Tm, wp.Pfw*1e-3));
            out.n_rated.Ps_vs_Tm.append(QPointF(wp.Tm, wp.Ps*1e-3));
            out.n_rated.Tem_vs_Tm.append(QPointF(wp.Tm, wp.Tem));
            out.n_rated.Tm_vs_Tm.append(QPointF(wp.Tm, wp.Tm));
            out.n_rated.Us_vs_Tm.append(QPointF(wp.Tm, wp.Us)) ;
            out.n_rated.theta_vs_Tm.append(QPointF(wp.Tm, wp.theta*180/PI));

        }

        // 计算恒转速负载时额定转矩下的各工作特性
        ///////////////////////////////////////////////////////////////////
        Tm = in.T_rated ;          // 额定转矩
        if (Tm>Tmax) {
            // 在当前额定转速下，电机无法输出额定的负载转矩，输出的额定工况为该转速输出最大转矩时的电机性能。
            errmsg += QT_TRANSLATE_NOOP("TsytControl","At the current rated speed, the motor cannot output the rated load torque, and the rated operating condition is the motor performance when the maximum torque is output at this speed.");
            Tm = Tmax;
        }
        wm = in.n_rated ;          // 恒转速
        we = wm * in.p / 9.55 ;

        WorkPoint wp;
        wp.calcTemWithTorque(Tm, we, in);
        // 根据MTPA算法确定IdIq
        double Iq = getYValueByX(wp.Tem,out.Iq_vs_Tem);
        double Id = getYValueByX(wp.Tem,out.Id_vs_Tem);
        wp.calcWithIdq(Id, Iq, we, in);

        out.n_rated.cosphi = wp.cosphi ;
        out.n_rated.ead = wp.ead ;
        out.n_rated.Id = wp.Id ;
        out.n_rated.Iq = wp.Iq ;
        out.n_rated.Is = wp.Is ;
        out.n_rated.P1 = wp.P1 ;
        out.n_rated.P2 = wp.P2 ;
        out.n_rated.Pcu = wp.Pcu ;
        out.n_rated.Pem = wp.Pem ;
        out.n_rated.Pfe = wp.Pfe ;
        out.n_rated.Pfw = wp.Pfw ;
        out.n_rated.Ps = wp.Ps ;
        out.n_rated.Tem = wp.Tem ;
        out.n_rated.Tm = wp.Tm ;
        out.n_rated.Us = wp.Us ;
        out.n_rated.Pfet = wp.Ptfe ;
        out.n_rated.Pfej = wp.Pjfe ;
        out.n_rated.theta = wp.theta ;
        out.n_rated.wm = wp.wm;
        ///////////////////////////////////////////////////////////////////
        // 恒转速负载时额定转矩下的各工作特性计算结束
    }

    return true ;
}

/*******************************************************************/
//   下面定义一个二元非线性的NR迭代函数

#include <cmath>
#include <iostream>
#include <iomanip>
#include <functional>

// 简易二维向量
struct Vec2
{
    double x{}, y{};
    Vec2() = default;
    Vec2(double x_, double y_) : x(x_), y(y_) {}
    Vec2 operator+(const Vec2& v) const { return {x + v.x, y + v.y}; }
    Vec2 operator*(double s) const { return {x * s, y * s}; }
    double norm() const { return std::hypot(x, y); }
};

/* ------------- 求解器：用 std::function 接收任意可调用对象 ------------- */
using Func  = std::function<double(double, double)>;
using DFunc = std::function<double(double, double)>;

bool solve2x2(const double J[2][2], const Vec2& F, Vec2& delta)
{
    double det = J[0][0] * J[1][1] - J[0][1] * J[1][0];
    if (std::fabs(det) < 1e-14) return false;
    delta.x = -(J[1][1] * F.x - J[0][1] * F.y) / det;
    delta.y = -(-J[1][0] * F.x + J[0][0] * F.y) / det;
    return true;
}

/********************************************************
 * 2D Newton–Raphson  接受任意可调用对象（函数指针、lambda…）
 * g++ nr2d_lambda.cpp -std=c++17 -O2 -o nr2d_lambda
 *******************************************************/
Vec2 newton2D(Func f, Func g,
              DFunc dfdx, DFunc dfdy,
              DFunc dgdx, DFunc dgdy,
              Vec2 guess,
              double tol = 1e-6,
              int maxIter = 50)
{
    Vec2 X = guess;
    for (int k = 0; k < maxIter; ++k)
    {
        double F1 = f(X.x, X.y);
        double F2 = g(X.x, X.y);
        Vec2 F(F1, F2);

        if (F.norm() < tol)
        {
            std::cout << "Converged in " << k << " steps\n";
            return X;
        }

        double J[2][2] = {
            {dfdx(X.x, X.y), dfdy(X.x, X.y)},
            {dgdx(X.x, X.y), dgdy(X.x, X.y)}
        };

        Vec2 delta;
        if (!solve2x2(J, F, delta))
        {
            std::cerr << "Jacobian singular at step " << k << "\n";
            break;
        }
        X = X + delta;

        std::cout << std::setw(2) << k << "  (x,y)= "
                  << std::fixed << std::setprecision(10)
                  << X.x << "  " << X.y
                  << "  |F|= " << F.norm() << '\n';
    }
    std::cerr << "Newton did not converge!\n";
    return X;
}

/* -------------------- 主函数：lambda 写法 -------------------- */
// int main()
// {
//     // 例：圆 x^2+y^2=4 与双曲线 xy=1
//     auto f  = [](double x, double y) { return x * x + y * y - 4; };
//     auto g  = [](double x, double y) { return x * y - 1; };
//     auto fx = [](double x, double y) { return 2 * x; };
//     auto fy = [](double x, double y) { return 2 * y; };
//     auto gx = [](double x, double y) { return y; };
//     auto gy = [](double x, double y) { return x; };

//     Vec2 guess(1.0, 1.5);
//     std::cout << "Initial guess: (" << guess.x << ", " << guess.y << ")\n";

//     Vec2 sol = newton2D(f, g, fx, fy, gx, gy, guess);

//     std::cout << "Solution: x= " << sol.x << ", y= " << sol.y << '\n';
//     std::cout << "Check f= " << f(sol.x, sol.y)
//               << "  g= " << g(sol.x, sol.y) << '\n';
//     return 0;
// }
/*******************************************************************/

bool findIdqWithFluxWeaken(double Umax, double Tem, double  we,
                           double &Id, double &Iq, const ControlInput &in)
{
    // T_{em} = \frac{3}{2} p \left[ \psi_f I_q + (L_d - L_q) I_d I_q \right]
    auto f  = [Tem, in](double Id, double Iq) {
        double Ld = in.calcLd(Id);
        double Lq = in.calcLq(Iq);
        return 1.5*in.p*(in.phif*Iq+(Ld-Lq)*Id*Iq) - Tem ;
    };
    //   U_d = R_s I_d - \omega L_q I_q
    //   U_q = R_s I_q + \omega (L_d I_d + \lambda_f) \]
    //   U^2_{max} = U^2_d + U^2_q
    auto g  = [Umax, we, in](double Id, double Iq) {
        double Ld = in.calcLd(Id);
        double Lq = in.calcLq(Iq);
        double Ud = in.R*Id - we*Lq*Iq;
        double Uq = in.R*Iq + we*(Ld*Id+in.phif);
        return Ud*Ud+Uq*Uq-Umax*Umax;
    };

    auto fx = [f](double Id, double Iq) {
        double Idp = Id*1.001;
        double Idn = Id*0.999;
        double fp = f(Idp, Iq) ;
        double fn = f(Idn, Iq) ;
        return (fp-fn)/(Idp-Idn);
    };

    auto fy = [f](double Id, double Iq) {
        double Iqp = Iq*1.001;
        double Iqn = Iq*0.999;
        double fp = f(Id, Iqp) ;
        double fn = f(Id, Iqn) ;
        return (fp-fn)/(Iqp-Iqn);
    };
    auto gx = [g](double Id, double Iq) {
        double Idp = Id*1.001;
        double Idn = Id*0.999;
        return (g(Idp, Iq)-g(Idn, Iq))/(Idp-Idn);
    };
    auto gy = [g](double Id, double Iq) {
        double Iqp = Iq*1.001;
        double Iqn = Iq*0.999;
        return (g(Id, Iqp)-g(Id, Iqn))/(Iqp-Iqn);
    };

    Vec2 guess(Id, Iq);
    std::cout << "Initial guess: (" << guess.x << ", " << guess.y << ")\n";
    Vec2 sol = newton2D(f, g, fx, fy, gx, gy, guess);
    std::cout << "Solution: Id= " << sol.x << ", Iq= " << sol.y << '\n';
    std::cout << "Check f= " << f(sol.x, sol.y)
              << "  g= " << g(sol.x, sol.y) << '\n';
    Id = sol.x; Iq = sol.y;
    return true;
}

bool FluxWeakenNew(FluxWeaken_in &in, FluxWeaken_out& out, QString &errmsg)
{
    logger()->debug() << __FUNCTION__;
    LOGGER_OUTPUT_CONTROLINPUT(in);

    if ( !valid_FluxWeaken(in,errmsg) )
        return false ;

    double Umax ;
    if ( in.e_control == 0 )
        Umax = in.Udc/2 ;
    else
        Umax = in.Udc/sqrt(3.0) ;
    in.phif = in.Uiv * sqrt_2 / ( 2*PI*in.f ) ;

    double Id , Iq ;
    //double Ld , Lq ;
    out.clear();

    calcMTPATrace(in, out);

    // 计算第一个点
    double wm1 = 0;

    // 计算第二个点
    double wm2;


    // 最大电流对应的最大转矩
    double Tem1 = getXValueByY(in.Imax,out.Is_vs_Tem,true);
    double Id2 = getYValueByX(Tem1,out.Id_vs_Tem,true);
    double Iq2 = getYValueByX(Tem1,out.Iq_vs_Tem,true);

    // 求解一元二次方程求解we2
    double we2 = calcOmegaE(Umax, Id2, Iq2, in) ;
    wm2 = 9.55 * we2 / in.p ;

    // 计算第三个点
    double Ld = in.calcLd(-in.Imax);
    double wm3 = Umax / (in.phif-Ld*in.Imax) * 9.55/in.p ;
    LOGGER_OUTPUT_VAR(Ld);
    LOGGER_OUTPUT_VAR(wm3);
    if ( wm3 < 0 )
        wm3 = 4 * Umax / in.phif * 9.55/in.p ;
    LOGGER_OUTPUT_VAR(wm3);

    // 开始扫描
    double wm , f , we , pfet , pfej ;
    double Pfe , Pcu , Pfw , Ps , ead , cosphi , Us , P1 , Tm , Pem , Is , theta , Ud , Uq ;
    double P2max = 0.0 ;

    // 第一段恒转矩部分(Tem和电流不变)
    double Tem = getXValueByY(in.Imax,out.Is_vs_Tem) ;
    Id = getYValueByX(Tem,out.Id_vs_Tem) ;
    Iq = getYValueByX(Tem,out.Iq_vs_Tem) ;
    for ( int i = 1 ; i < Cnt1+Cnt2 ; i++ )
    {
        // 从第一个点到第二个点扫描
        if ( i <= Cnt1 )
        {
            wm = wm1 + (wm2-wm1) * i / Cnt1 ;
            we = wm * in.p / 9.55 ;
        }
        // 从第二个点到第三个点扫描
        else
        {
            wm = wm2 + (wm3 - wm2) * (i-Cnt1)/Cnt2 ;
            we = wm * in.p / 9.55 ;
            Iq = findIqWithFluxWeaken(Umax, we, in);
            if (qIsNaN(Iq)) {
                break;
            }
            Id = -sqrt(in.Imax*in.Imax-Iq*Iq) ;
        }

        WorkPoint wp;
        wp.calcWithIdq(Id, Iq, we, in);
        if ( wp.P2 > P2max )
        {
            out.wmmax = wp.wm ;
            P2max = wp.P2 ;
        }

        if ( wp.P2 > 0 )
        {
            out.cosphi_vs_n.append(QPointF(wp.wm, wp.cosphi)) ;
            out.ead_vs_n.append(QPointF(wp.wm, wp.ead));
            out.Id_vs_n.append(QPointF(wp.wm, wp.Id));
            out.Iq_vs_n.append(QPointF(wp.wm, wp.Iq));
            out.Is_vs_n.append(QPointF(wp.wm, wp.Is));
            out.P1_vs_n.append(QPointF(wp.wm, wp.P1*1e-3));
            out.P2_vs_n.append(QPointF(wp.wm, wp.P2*1e-3));
            out.Pcu_vs_n.append(QPointF(wp.wm, wp.Pcu*1e-3));
            out.Pem_vs_n.append(QPointF(wp.wm, wp.Pem*1e-3));
            out.Pfe_vs_n.append(QPointF(wp.wm, wp.Pfe*1e-3));
            out.Pfw_vs_n.append(QPointF(wp.wm, wp.Pfw*1e-3));
            out.Ps_vs_n.append(QPointF(wp.wm, wp.Ps*1e-3));
            out.Tem_vs_n.append(QPointF(wp.wm, wp.Tem));
            out.Tm_vs_n.append(QPointF(wp.wm, wp.Tm));
            out.Us_vs_n.append(QPointF(wp.wm, wp.Us));
            out.theta_vs_n.append(QPointF(wp.wm, wp.theta*180/PI));
        }
    }

    if (out.Tm_vs_n.size()==0) {
        errmsg = QT_TRANSLATE_NOOP("TsytControl","Cannot get the rated point in characteristic curve.") ;
        return false;
    }

    // 恒转矩负载
    if ( in.e_work == 0 )
    {
        // 当前转矩的转折转速
        double wm_temp = -1;

        // 当Tm=Trated时寻找相应的wm
        bool flag = false ;
        double wTm = getRootValueByY( in.T_rated , out.Tm_vs_n , &flag ) ;
        if ( !flag ) {
            if ( in.T_rated > out.Tm_vs_n.at(0).y() )
            {
                // 额定转矩大于电机可输出的最大转矩，计算结果中将输出转折转速时的性能曲线
                errmsg = QT_TRANSLATE_NOOP("TsytControl","The rated torque is greater than the maximum torque the motor can output, and the performance curve at the inflection point speed will be output in the calculation results.");
                wTm = wm2 ;
                in.T_rated = getYValueByX(wm2,out.Tm_vs_n);
                wm_temp = wm2;  // 转折转速等于wm2
            }
            else if ( in.T_rated < out.Tm_vs_n.last().y() )
            {
                double T1 , T2 , w1 , w2 ;
                int count = out.Tm_vs_n.count() ;
                T1 = out.Tm_vs_n.last().y() ;
                w1 = out.Tm_vs_n.last().x() ;
                T2 = out.Tm_vs_n.at(count-2).y() ;
                w2 = out.Tm_vs_n.at(count-2).x() ;
                wTm = w2 + (w2-w1)/(T2-T1)*(in.T_rated-T2) ;
            }
        }
        out.T_rated.maxWm = wTm;

        if (wm_temp<0) {
            // 当前转矩存在弱磁段，需要计算弱磁分界点
            WorkPoint wp2;
            double wm1 = 0, wm2 = wTm;

            do {
                double assume_wm = 0.5 * (wm1+wm2);
                wp2.calcTemWithTorque(in.T_rated, assume_wm, in);
                Id = getYValueByX(wp2.Tem,out.Id_vs_Tem) ;
                Iq = getYValueByX(wp2.Tem,out.Iq_vs_Tem) ;
                wm_temp = calcOmegaE(Umax, Id, Iq, in);
                wm_temp = 9.55 * wm_temp / in.p ;
                if (fabs(wm_temp/assume_wm-1.0)<0.005) {
                    break;
                }
                else {
                    if (wm_temp > assume_wm) {
                        wm1 = assume_wm;
                    }
                    else {
                        wm2 = assume_wm;
                    }
                }
            } while (true);
        }

        // 从0-wTm对wm进行扫描
        double Iq0, Ld, Lq;
        for ( int i = 1 ; i <= Cnt1+Cnt2 ; i++ )
        {
            WorkPoint wp;
            if ( i <= Cnt1 )
            {
                wm = wm_temp * i / Cnt1 ;
                we = wm * in.p / 9.55 ;

                wp.calcTemWithTorque(in.T_rated, we, in);
                Iq = getYValueByX(wp.Tem,out.Iq_vs_Tem);
                Id = getYValueByX(wp.Tem,out.Id_vs_Tem);
                wp.calcWithIdq(Id, Iq, we, in);
            }
            else if (wm_temp<wTm)
            {
                wm = wm_temp + (wTm-wm_temp) * (i-Cnt1) / Cnt2 ;
                we = wm * in.p / 9.55 ;
                wp.calcTemWithTorque(in.T_rated, we, in);
                Tem = wp.Tem;
                // 弱磁区域迭代，已知Tem和Umax，求解满足条件的Id,Iq
                findIdqWithFluxWeaken(Umax, Tem, we, Id, Iq, in);
                // 计算性能
                wp.calcWithIdq(Id, Iq, we, in);
            }

            out.T_rated.cosphi_vs_n.append(QPointF(wp.wm, wp.cosphi));
            out.T_rated.ead_vs_n.append(QPointF(wp.wm, wp.ead));
            out.T_rated.Id_vs_n.append(QPointF(wp.wm, wp.Id));
            out.T_rated.Iq_vs_n.append(QPointF(wp.wm, wp.Iq));
            out.T_rated.Is_vs_n.append(QPointF(wp.wm, wp.Is));
            out.T_rated.P1_vs_n.append(QPointF(wp.wm, wp.P1*1e-3));
            out.T_rated.P2_vs_n.append(QPointF(wp.wm, wp.P2*1e-3));
            out.T_rated.Pcu_vs_n.append(QPointF(wp.wm, wp.Pcu*1e-3));
            out.T_rated.Pem_vs_n.append(QPointF(wp.wm, wp.Pem*1e-3));
            out.T_rated.Pfe_vs_n.append(QPointF(wp.wm, wp.Pfe*1e-3));
            out.T_rated.Pfw_vs_n.append(QPointF(wp.wm, wp.Pfw*1e-3));
            out.T_rated.Ps_vs_n.append(QPointF(wp.wm, wp.Ps*1e-3));
            out.T_rated.Tem_vs_n.append(QPointF(wp.wm, wp.Tem));
            out.T_rated.Tm_vs_n.append(QPointF(wp.wm, wp.Tm));
            out.T_rated.Us_vs_n.append(QPointF(wp.wm, wp.Us));
            out.T_rated.theta_vs_n.append(QPointF(wp.wm, wp.theta*180/PI));
        }

        // 当wm为运行转速时，计算各工作点特性
        ///////////////////////////////////////////////////////////////////////////
        wm = in.n_rated ;
        if (in.n_rated > out.T_rated.maxWm) {
            // 在当前负载转矩下，电机无法达到额定转速，输出的额定工况为最大转速下的电机性能。
            errmsg += QT_TRANSLATE_NOOP("TsytControl", "Under the current load torque, the motor cannot reach the rated speed, and the rated operating condition is the motor performance at the maximum speed.") ;
            wm = out.T_rated.maxWm;
        }

        WorkPoint wp;
        if (wm<wm_temp) {
            we = wm * in.p / 9.55 ;

            wp.calcTemWithTorque(in.T_rated, we, in);
            Iq = getYValueByX(wp.Tem,out.Iq_vs_Tem);
            Id = getYValueByX(wp.Tem,out.Id_vs_Tem);
            wp.calcWithIdq(Id, Iq, we, in);

        }
        else {
            we = wm * in.p / 9.55 ;
            f = we / 2 / PI ;
            wp.calcTemWithTorque(in.T_rated, we, in);
            Tem = wp.Tem;
            // 弱磁区域迭代，已知Tem和Umax，求解满足条件的Id,Iq
            findIdqWithFluxWeaken(Umax, Tem, we, Id, Iq, in);

            wp.calcWithIdq(Id, Iq, we, in);
        }

        out.T_rated.cosphi = wp.cosphi ;
        out.T_rated.ead = wp.ead ;
        out.T_rated.Id = wp.Id ;
        out.T_rated.Iq = wp.Iq ;
        out.T_rated.Is = wp.Is ;
        out.T_rated.P1 = wp.P1 ;
        out.T_rated.P2 = wp.P2 ;
        out.T_rated.Pcu = wp.Pcu ;
        out.T_rated.Pem = wp.Pem ;
        out.T_rated.Pfe = wp.Pfe ;
        out.T_rated.Pfw = wp.Pfw ;
        out.T_rated.Ps = wp.Ps ;
        out.T_rated.Tem = wp.Tem ;
        out.T_rated.Tm = wp.Tm ;
        out.T_rated.Us = wp.Us ;
        out.T_rated.Pfet = wp.Ptfe ;
        out.T_rated.Pfej = wp.Pjfe ;
        out.T_rated.theta = wp.theta ;
        out.T_rated.wm = wp.wm;
        ///////////////////////////////////////////////////////////////////////////////////
        // 工作特性计算结束
    }
    else {
        errmsg = QT_TRANSLATE_NOOP("TsytControl","Constant speed operation is not supported.") ;
        return false;
    }
    return true ;
}
