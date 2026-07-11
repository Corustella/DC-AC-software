#ifndef FLOAT_TOOLS_H_
#define TFLOAT_TOOLS_H_

#define PAREMETER_COUNT 12
#define PAREMETER_GROUP_COUNT 4
#define PAREMETER_CHOOSE_COUNT 3
#define AVE_COUNT 500
#define QUEUE_SIZE 50

extern float U_alpha_beta_L;
extern float I_1_alpha_beta_L;
extern float I_2_alpha_beta_L;
extern float DQ_alpha_beta_L;
typedef struct {
    float data[AVE_COUNT];
    float sum;
    int pointer;
}float_ave;
typedef struct
{
    float data[QUEUE_SIZE];
    int head;
    int tail;
}queue;

typedef struct
{
    float y_out;
    float a;//a=2*pi*fc/fs fc鎴棰戠巼 fs 閲囨牱棰戠巼
    float b;
}one_filter;

typedef struct {
    float kp,ki,kd;
    float B0;//KI+KP+KD
    float B1;//KP+2KD
    float B2;//KD
    float err_pre1;
    float err_pre2;
    float out;//澧為噺鍚巓ut
    float out_last;//璁板綍
    float upper;
    float lower;
}PID_CONTROLER_F;

typedef struct {
    float B0;
    float B1;
    float ylf_last;
    float ylf;
    float err_last;
    float upper;
    float lower;
}PI_CONTROLER_F;

typedef struct{
    float a;
    float b;
    float c;
}ABC;

typedef struct{
    float alpha;
    float beta;
}Alpha_Beta;

typedef struct{
    float d;
    float q;
}D_Q;
typedef struct
{
    float sin_theta;
    float cos_theta;
}Theta;

typedef struct
{
    float w1;
    float w2;
    float w3;
}Wave;
typedef struct
{
    float y_out;
    float a;//a=2*pi*fc/fs fc鎴棰戠巼 fs 閲囨牱棰戠巼
    float b;
}filter;
typedef struct
{
    float x_a;
    float T_ADC;
    float k_sogi;
    float integral_2;
    float integral_3;
    float w0_sogi;
    float ualfa_q;
}sogi;
typedef struct
{
    float w0;float T_s;float k;
    float x;float y;float lamda;

    float a1,a2;
    float b0;
    float qb0,qb1,qb2;

    float ui0,ui1,ui2;
    float ud0,ud1,ud2;
    float uq0,uq1,uq2;
}tustin_sogi;
typedef struct
{
    // pll->m_f32Ref = 0;							/* 缁欏畾                       */
    // pll->m_f32Fed = 0;               			/* 鍙嶉                       */
    // pll->m_f32Err = 0;               			/* 璇樊                       */
    // pll->m_f32ErrPre = 0;            			/* 鍓嶆媿璇樊                */
    // pll->m_f32integral = 0;
    // pll->m_f32Kp = 5;                		/* 姣斾緥鍙傛暟  5               */
    // pll->m_f32Ki = 0.2;                		/* 绉垎鍙傛暟    0.2             */
    // pll->m_f32Acc = 0;               			/* 绉垎绱姞               */
    // pll->m_f32LoopOut = 0;           			/* 杈撳嚭                       */
    // pll->m_f32KiMaxACC = 800;          	/* 绉垎绱姞鏈�澶ч檺骞�  */
    // pll->m_f32KiMinACC = -800;          	/* 绉垎绱姞鏈�灏忛檺骞�  */
    // pll->m_f32MaxOutput = 400;         	/* 杈撳嚭鏈�澶ч檺骞�           */
    // pll->m_f32MinOutput = 0;         	/* 杈撳嚭鏈�灏忛檺骞�           */
    float m_f32Ref;
    float m_f32Fed;
    float m_f32Err;
    float m_f32ErrPre;
    float m_f32Kp;
    float m_f32Ki;
    float m_f32LoopOut;
    float m_f32MaxOutput;
    float m_f32MinOutput;//澧為噺寮�
    float Theta_out;
}PI_PLL;

/*x_a=(ualfa_0 - SOGI_ualfa.integral_2)*k_sogi;
SOGI_ualfa.integral_2=SOGI_ualfa.integral_2+w0_sogi*(x_a-SOGI_ualfa.integral_3)*T_ADC;
ualfa_1 = SOGI_ualfa.integral_2;
SOGI_ualfa.integral_3 = SOGI_ualfa.integral_3 + w0_sogi*SOGI_ualfa.integral_2*T_ADC;
ualfa_q = SOGI_ualfa.integral_3;*/
void init();
void PID_init_F(PID_CONTROLER_F* pic,float out0,float kp,float ki,float kd,float upper,float lower);
void PID_step_F(PID_CONTROLER_F* pic,float err);
void PID_CHANGE_PARAM_F(PID_CONTROLER_F* pic,float kp,float ki,float kd);

void float_ave_init(float_ave * fa);
float float_ave_step(float_ave * fa,float datain);

void one_filter_init(one_filter * of,float a);
float one_filter_step(one_filter * of,float datain);

void float_to_string(float floa,char *str, int len1,int len2);

void abc2alpha_beta(ABC * abc,Alpha_Beta * alpha_beta);
void alpha_beta2dq(Alpha_Beta * I_alpha_beta,Alpha_Beta * U_alpha_beta,Theta * theta,D_Q * dq);
void dq2alpha_beta(D_Q *dq,Theta * theta,Alpha_Beta * alpha_beta);
void alpha_beta2wave(float Udc,Alpha_Beta * alpha_beta,Wave * wave);

void queue_push(queue* q,float x);
float queue_pop(queue * q);
void queue_clear(queue * q);

void filter_init(filter * filter,float a);
float filter_step(filter * filter,float x);

void sogi_init(sogi * sogi,float w0,float T_ADC,float k_sogi);
void sogi_step(sogi * sogi,float ualfa_0,float* alfa,float* beta);
void tustin_sogi_init(tustin_sogi * tustin_sogi,float w0,float T_s,float k);
void tustin_sogi_step(tustin_sogi * tustin_sogi,float ualfa_0,float* alfa,float* beta);
void pi_pll_init(PI_PLL * pi_pll,float kp,float ki,float max,float min);
void pi_pll_step(PI_PLL * pi_pll,Theta* theta,float fed);
#endif
