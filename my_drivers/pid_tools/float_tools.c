#include "../include/float_tools.h"
#include "arm_math.h"
float U_alpha_beta_L=0;
float I_1_alpha_beta_L=0;
float I_2_alpha_beta_L=0;
float DQ_alpha_beta_L=0;
//初始化PID参数，PID_CONTROLER_F为结构体
void PID_init_F(PID_CONTROLER_F *pic,float out0,float kp, float ki, float kd, float upper, float lower)
{
    pic->kp = kp;
    pic->ki = ki;
    pic->kd = kd;
    pic->out_last=out0;
    pic->B0 = ki + kp + kd;
    pic->B1 = kp + 2 * kd;
    pic->B2 = kd;
    pic->upper = upper;
    pic->lower = lower;
    pic->err_pre1 = 0;
    pic->err_pre2 = 0;
    pic->out = out0;
}
void PID_step_F(PID_CONTROLER_F *pic, float err)//err为当前误差值
{
	if(err<0.001&&err>-0.001)err=0;
    pic->out = pic->out_last + pic->B0 * err- pic->B1 * pic->err_pre1;//+ pic->B2 * pic->err_pre2
    pic->out = (pic->out > pic->upper) ? pic->upper : ((pic->out < pic->lower) ? pic->lower : pic->out);
   // pic->err_pre2 = pic->err_pre1;
    pic->err_pre1 = err;
    pic->out_last = pic->out;
}
//将三相电流值转换为两相静止坐标系下的 α 和 β 值
void abc2alpha_beta(ABC *abc, Alpha_Beta *alpha_beta)
{
    // alpha_beta->alpha = fix_mpy(fix_IQ(0.816497),abc->a-_IQdiv2(abc->b+abc->c));
    // alpha_beta->beta = fix_mpy(fix_IQ(0.707107),abc->b-abc->c);
    alpha_beta->alpha = 0.816497 * (abc->a - 0.5 * (abc->b + abc->c));
    alpha_beta->beta = 0.707107 * (abc->b - abc->c);
}
//两相静止坐标系下的 α（阿尔法）和 β（贝塔）分量转换为两相旋转坐标系下的 d（直轴）和 q（交轴）分量
void alpha_beta2dq(Alpha_Beta *I_alpha_beta, Alpha_Beta *U_alpha_beta, Theta *theta, D_Q *dq)
{
    dq->q = -I_alpha_beta->alpha * theta->sin_theta + I_alpha_beta->beta * theta->cos_theta;
    dq->d = I_alpha_beta->alpha * theta->cos_theta + I_alpha_beta->beta * theta->sin_theta;
}
void dq2alpha_beta(D_Q *dq, Theta *theta, Alpha_Beta *alpha_beta)
{
    // alpha_beta->alpha = -fix_mpy(dq->d,theta->sin_theta) - fix_mpy(dq->q,theta->cos_theta);
    // alpha_beta->beta = fix_mpy(dq->d,theta->cos_theta) - fix_mpy(dq->q,theta->sin_theta);
    //DQ_alpha_beta_L = sqrt(dq->d * dq->d + dq->q * dq->q);
    //arm_sqrt_f32(dq->d * dq->d + dq->q * dq->q,&DQ_alpha_beta_L);
    alpha_beta->beta = dq->d * theta->sin_theta + dq->q * theta->cos_theta;
    alpha_beta->alpha = dq->d * theta->cos_theta - dq->q * theta->sin_theta;
}
//将两相静止坐标系下的 α（阿尔法）和 β（贝塔）分量转换为三相电压波形
void alpha_beta2wave(float Udc, Alpha_Beta *alpha_beta, Wave *wave)
{
    // int k = fix_div(fix_IQ(512),Udc);
    // wave->w1 = fix_mpy(k,Udc - fix_mpy(fix_IQ(2.0),alpha_beta->alpha));
    // wave->w2 = fix_mpy(k,Udc + alpha_beta->alpha - fix_mpy(fix_IQ(1.732051),alpha_beta->beta));
    // wave->w3 = fix_mpy(k,Udc + alpha_beta->alpha + fix_mpy(fix_IQ(1.732051),alpha_beta->beta));
    float k = 512.0 / Udc;//HSH浠ｇ爜 淇敼鐨勫湪涓夌浉鏁存祦閲岄潰 鏀逛负iq瀹氱偣鏁颁簡
    wave->w1 = k * (Udc - 2.0 * alpha_beta->alpha);
    wave->w2 = k * (Udc + alpha_beta->alpha - 1.732051 * alpha_beta->beta);
    wave->w3 = k * (Udc + alpha_beta->alpha + 1.732051 * alpha_beta->beta);
}

void float_ave_init(float_ave * fa)
{
    fa->sum=0;
    for(int i=0;i<AVE_COUNT;i++)
    {
        fa->data[i]=0;
    }
    fa->pointer=0;
}
float float_ave_step(float_ave * fa,float datain)
{
    fa->sum=fa->sum-fa->data[fa->pointer]+datain;
    fa->data[fa->pointer]=datain;
    fa->pointer=(fa->pointer+1)<AVE_COUNT?(fa->pointer+1):0;
    return fa->sum/AVE_COUNT;
}
//动态修改PID参数
void PID_CHANGE_PARAM_F(PID_CONTROLER_F* pic,float kp,float ki,float kd)
{
    pic->kp = kp;
    pic->ki = ki;
    pic->kd = kd;
    pic->B0 = ki + kp + kd;
    pic->B1 = kp + 2 * kd;
    pic->B2 = kd;
}

void one_filter_init(one_filter * of,float a)
{
    of->a=a;
    of->b=1-a;
    of->y_out=0;
}
float one_filter_step(one_filter * of,float datain)
{
    of->y_out=of->b*of->y_out+of->a*datain;
    return of->y_out;
}
void float_to_string(float floa,char *str, int len1,int len2)
{
	    unsigned char i=0,j=0;
	    unsigned int zhengshu = floa;
	    unsigned int xiaoshu = (floa-zhengshu)*pow(10,len2);
	    while(len1--)
	    {
	        str[len1] = zhengshu%10+0x30;
	        zhengshu = zhengshu/10;
	        i++;
	    }
	    str[i] = '.';
	    while(len2--)
	    {
	        str[i+len2+1] = xiaoshu%10+0x30;
	        xiaoshu = xiaoshu/10;
	        j++;
	    }
	    str[i+j+1] = '\0';
}

void queue_push(queue* q,float x)
{
    *(q->data+q->tail)=x;
    q->tail=(q->tail+1)%QUEUE_SIZE;
}
float queue_pop(queue* q)
{
    float x=*(q->data+q->head);
    q->head=(q->head+1)%QUEUE_SIZE;
    return x;
}
void queue_clear(queue* q)
{
    q->head=0;
    q->tail=0;
    for(int i=0;i<QUEUE_SIZE;i++)
    {
        *(q->data+i)=0;
    }
}
void filter_init(filter * filter,float a)
{
    filter->a = a;
    filter->b = 1-a;
    filter->y_out = 0;
}
float filter_step(filter * filter,float x)
{
    // filter->y_out = fix_mpy(filter->b,filter->y_out)+fix_mpy(filter->a,x);
    filter->y_out = filter->b*filter->y_out+filter->a*x;
    return filter->y_out;
}
void sogi_init(sogi * sogi,float w0,float T_ADC,float k_sogi)
{
    sogi->w0_sogi = w0;
    sogi->T_ADC = T_ADC;
    sogi->k_sogi = k_sogi;
    sogi->integral_2 = 0;
    sogi->integral_3 = 0;
    sogi->x_a = 0;
}
void sogi_step(sogi * sogi,float ualfa_0,float* alfa,float* beta)
{
   sogi->x_a = (ualfa_0 - sogi->integral_2)*sogi->k_sogi;
   sogi->integral_2 = sogi->integral_2+sogi->w0_sogi*(sogi->x_a-sogi->integral_3)*sogi->T_ADC;
   //sogi->ualfa_1 = sogi->integral_2;
   *alfa=sogi->integral_2;
   sogi->integral_3 = sogi->integral_3 + sogi->w0_sogi*sogi->integral_2*sogi->T_ADC;
   sogi->ualfa_q = sogi->integral_3;
   *beta=sogi->ualfa_q;
//return sogi->ualfa_q;



    ////    v0_s = ((v_s1-v1_s)*K-v2_s)*W;//v0 x_a v1 i2  v2 i3 v3 i3
    //    v1_s += v0_s*Ts;//寰楀埌V1(s)(伪)
    //    v3_s += v1_s*Ts;
    //    v2_s = v3_s*W;//寰楀埌V2(s)(尾)


    // sogi->x_a = ((ualfa_0 - sogi->integral_2)*sogi->k_sogi-sogi->integral_3)*sogi->w0_sogi;
    // sogi->integral_2 = sogi->integral_2+sogi->x_a*sogi->T_ADC;
}
void tustin_sogi_init(tustin_sogi * tustin_sogi,float w0,float T_s,float k)
{
    tustin_sogi->w0 = w0;
    tustin_sogi->T_s = T_s;
    tustin_sogi->k = k;//杈撳叆鍙傛暟
    // 涓棿鍙傛暟璁＄畻
    tustin_sogi->x =2*k*w0*T_s;
    tustin_sogi->y =w0*T_s*w0*T_s;
    tustin_sogi->lamda=0.5*w0*T_s;

    tustin_sogi->b0=(tustin_sogi->x)/(tustin_sogi->y+tustin_sogi->x+4);

    tustin_sogi->a1=(8-2*tustin_sogi->y)/(tustin_sogi->y+tustin_sogi->x+4);
    tustin_sogi->a2=(tustin_sogi->x-tustin_sogi->y-4)/(tustin_sogi->y+tustin_sogi->x+4);

    tustin_sogi->qb0=tustin_sogi->b0*tustin_sogi->lamda;
    tustin_sogi->qb1=tustin_sogi->qb0*2;
    tustin_sogi->qb2=tustin_sogi->qb0;

    tustin_sogi->ui0=0;
    tustin_sogi->ui1=0;
    tustin_sogi->ui2=0;

    tustin_sogi->uq0=0;
    tustin_sogi->uq1=0;
    tustin_sogi->uq2=0;

    tustin_sogi->ud0=0;
    tustin_sogi->ud1=0;
    tustin_sogi->ud2=0;

}
void tustin_sogi_step(tustin_sogi * cal,float ualfa_0,float* alfa,float* beta)
{
    //%%璁＄畻杩囩▼锛屽搴斿紡11绂绘暎搴忓垪
// u0            = ui;
// osg_u0        = (b0*(u0-u2)) + (a1*osg_u1) + (a2*osg_u2);
// osg_u2        = osg_u1;
// osg_u1        = osg_u0;
// %%瀵瑰簲寮�12绂绘暎搴忓垪
// osg_qu0       = (qb0*u0) + (qb1*u1) + (qb2*u2) + (a1*osg_qu1) + (a2*osg_qu2);
// osg_qu2       = osg_qu1;
// osg_qu1       = osg_qu0;
// %%鏇存柊搴忓垪鍊�
// u2            = u1;
// u1            = u0;
// %%杈撳嚭
// uo            =osg_u0;
// quo           =osg_qu0;

    cal->ui0=ualfa_0;
    cal->ud0=(cal->b0*(cal->ui0-cal->ui2))+(cal->a1*cal->ud1)+(cal->a2*cal->ud2);
    cal->ud2=cal->ud1;
    cal->ud1=cal->ud0;

    cal->uq0=(cal->qb0*cal->ui0)+(cal->qb1*cal->ui1)+(cal->qb2*cal->ui2)+(cal->a1*cal->uq1)+(cal->a2*cal->uq2);
    cal->uq2=cal->uq1;
    cal->uq1=cal->uq0;

    cal->ui2=cal->ui1;
    cal->ui1=cal->ui0;

    *alfa=cal->ud0;
    *beta=cal->uq0;
}
void pi_pll_init(PI_PLL * pll,float kp,float ki,float max,float min)
{
    pll->m_f32Kp=kp;
    pll->m_f32Ki=ki;
    pll->m_f32MaxOutput=max;
    pll->m_f32MinOutput=min;
    pll->m_f32Err=0;
    pll->m_f32ErrPre=0;
    pll->m_f32LoopOut=2 * PI*50;
    pll->m_f32Ref=0;
    pll->Theta_out=0;

}
void pi_pll_step(PI_PLL * pll,Theta* theta,float fed)
{
    pll->m_f32Fed = fed;//t_spll->g_f32VoltQ;
    pll->m_f32Err = pll->m_f32Fed - pll->m_f32Ref;
//    t_f32TempValue = pll->m_f32Ki * pll->m_f32Err;                              /* 绉垎璁＄畻  */
//    t_f32TempValue += pll->m_f32Kp * (pll->m_f32Err - pll->m_f32ErrPre);        /* 姣斾緥璁＄畻  */
    pll->m_f32LoopOut += (pll->m_f32Ki * pll->m_f32Err+pll->m_f32Kp * (pll->m_f32Err - pll->m_f32ErrPre));/* 澧為噺杩愮畻  */
    // pll->m_f32LoopOut = 2 * M_PI*50+t_f32TempValue;

    pll->m_f32ErrPre = pll->m_f32Err;

    if (pll->m_f32LoopOut > pll->m_f32MaxOutput)                                  /* 杈撳嚭闄愬箙   */
    {
        pll->m_f32LoopOut = pll->m_f32MaxOutput;
    }

    if (pll->m_f32LoopOut < pll->m_f32MinOutput)
    {
        pll->m_f32LoopOut = pll->m_f32MinOutput;
    }
    pll->Theta_out += (pll->m_f32LoopOut /10000.0);//鐜矾杈撳嚭鍗充负w锛岄渶瑕乼绉垎//ts 1/10k

    if(pll->Theta_out > (2 * PI))
    {
        pll->Theta_out = pll->Theta_out - (2 * PI);
    }
    if(pll->Theta_out < 0)
    {
        pll->Theta_out = pll->Theta_out + (2 * PI);
    }
	// t_spll->g_f32SinWT = sinf(t_spll->g_f32Theta);
    // t_spll->g_f32CosWT = cosf(t_spll->g_f32Theta);
    theta->cos_theta=arm_cos_f32(pll->Theta_out);
    theta->sin_theta=arm_sin_f32(pll->Theta_out);
		// if (t_spll->g_f32Theta<(0.5f*M_PI)||t_spll->g_f32Theta>(1.5f*M_PI))//PB2鎸囩ず閿佺浉缁撴灉
		// 		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_SET);
		// else
		// 	  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_RESET);
}
