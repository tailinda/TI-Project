#include "DSP28x_Project.h"     // Device Headerfile and Examples Include File
#include <string.h>
#include <stdint.h>

void GPIO_select();
void avg();
void InitEPwmTimer(void);
void PIcontrol(void);
void swADC(void);
void swPWM(void);
interrupt void cpu_timer0_isr(void);

extern Uint16 RamfuncsLoadStart;
extern Uint16 RamfuncsLoadSize;
extern Uint16 RamfuncsRunStart;
///PI control///
int	Expectation = 48;
int AVGa[256],ana[256],err[256],p[256],kii[256],pi[256];
int analogy=0;
long int kp=0,verr=0,verrR=0,ki=0,kiR=0, pierr=0,AVGan=0;
int ai=0,bi=0;
int a[16]={0,0,0,0,
           0,0,0,0,
           0,0,0,0,
           0,0,0,0};
///pwm///
int  swpwm=0;
int  pro=0;
///adc///
int  swadc=0;
int  i=0;
int  j=0;

//主程式
void main(void)
{
   InitSysCtrl();
   memcpy((uint16_t *)&RamfuncsRunStart,(uint16_t *)&RamfuncsLoadStart, (unsigned long)&RamfuncsLoadSize);
   InitFlash();

   DINT;
   InitPieCtrl();
   InitPieVectTable();
   IER = 0x0000;	// Disable CPU interrupts and clear all CPU interrupt flags:
   IFR = 0x0000;

   InitCpuTimers();

   EALLOW;
   PieVectTable.TINT0 = &cpu_timer0_isr;
   EDIS;

   InitGpio();
   GPIO_select();

   InitAdc();         // init the ADC
	EALLOW;
	AdcRegs.ADCCTL1.bit.INTPULSEPOS	= 1;		//ADCINT1 trips after AdcResults latch
	AdcRegs.INTSEL1N2.bit.INT1CONT  = 0;			//Disable ADCINT1 Continuous mode
	AdcRegs.ADCSOC0CTL.bit.CHSEL = 0;       		//設置 ADCINA 0 轉換
	AdcRegs.ADCSOC0CTL.bit.TRIGSEL = 1;    			//ADCTRIG1 ,CPU Timer0 ,TINT0n
	AdcRegs.ADCSOC0CTL.bit.ACQPS = 6;       		//設置 SOC0 ACQPS至7 ADCCLK
	AdcRegs.ADCCTL1.bit.ADCENABLE = 1;      // Enable ADC
	EDIS;

	EALLOW;
	SysCtrlRegs.PCLKCR0.bit.TBCLKSYNC =0;
	EDIS;
	InitEPwmTimer();
	EALLOW;
	SysCtrlRegs.PCLKCR0.bit.TBCLKSYNC =1;
	EDIS;

	 ConfigCpuTimer(&CpuTimer0, 1, 3906);    //3906 = 60M /( 60*256 )
	 CpuTimer0Regs.TCR.all = 0x4001;         // TIE cpu-timer enable, TSS=0 CPU-timer running
	  // Enable ADCINT1 in PIE
	  PieCtrlRegs.PIECTRL.bit.ENPIE = 1;
	  PieCtrlRegs.PIEIER1.bit.INTx7 = 1;      // Enable CPUTIMER 1.7 in the PIE
	  IER |= M_INT1;                          // Enable CPU Interrupt 1
	  EINT;          						 // Enable Global interrupt INTM
	  ERTM;

	  for(;;){
		  swADC ();
		  swPWM ();
	  }
}

//選擇GPIO的腳位狀態
void GPIO_select(void)
{
	EALLOW;
	 GpioCtrlRegs.GPADIR.bit.GPIO1 = 1;      	//  GPIO0 as output
	 GpioCtrlRegs.GPAMUX1.bit.GPIO1= 1;      	//  EPWM1B

	 GpioCtrlRegs.GPADIR.bit.GPIO6 = 0;     	//  GPIO6 as input
	 GpioCtrlRegs.GPAMUX1.bit.GPIO6= 0;    		//  GPIO6  pwm switch
	 GpioCtrlRegs.GPAPUD.bit.GPIO6 = 0;
	 GpioCtrlRegs.GPADIR.bit.GPIO4 = 0;     	//  GPIO4 as input
	 GpioCtrlRegs.GPAMUX1.bit.GPIO4= 0;      	//  GPIO4  swADC switch
	 GpioCtrlRegs.GPAPUD.bit.GPIO4 = 0;

	 GpioCtrlRegs.GPADIR.bit.GPIO9 = 1;     	//  GPIO9 as output
	 GpioCtrlRegs.GPAMUX1.bit.GPIO9= 0;     	//  GPIO9  warn LED
	 GpioCtrlRegs.GPADIR.bit.GPIO7 = 1;     	//  GPIO7 as output
	 GpioCtrlRegs.GPAMUX1.bit.GPIO7= 0;     	//  GPIO7  swpwm LED
	 GpioCtrlRegs.GPADIR.bit.GPIO5 = 1;     	//  GPIO5 as output
	 GpioCtrlRegs.GPAMUX1.bit.GPIO5= 0;     	//  GPIO5  swADC LED

	 EDIS;
}

//初始化pwm//
void InitEPwmTimer(void){
	EPwm1Regs.TBPRD = 600; // Period = 1500 TBCLK counts
	                             // TBCLK = 1/60M  T=50us
	                             // Tpwm=2*TBPRD*TBCLK = 1/20k = 2*TBPRD*(1/60M)   = 1500
	 EPwm1Regs.TBPHS.half.TBPHS = 0; // Set Phase register to zero
   //  EPwm1Regs.CMPA.half.CMPA = pierr; // Compare A = pierr TBCLK counts
	 EPwm1Regs.TBCTL.bit.CTRMODE = 10;  //UPDOWN // Symmetrical mode
	 EPwm1Regs.TBCTL.bit.PHSEN = TB_DISABLE; // Master module
	 EPwm1Regs.TBCTL.bit.PRDLD = TB_SHADOW;
	 EPwm1Regs.TBCTL.bit.SYNCOSEL = TB_CTR_ZERO; // Sync down-stream module
	 EPwm1Regs.TBCTL.bit.HSPCLKDIV = TB_DIV1;        // TBCLK=SYSCLKOUT
	 EPwm1Regs.TBCTL.bit.CLKDIV = TB_DIV1;
	 EPwm1Regs.CMPCTL.bit.SHDWAMODE = CC_SHADOW;
	 EPwm1Regs.CMPCTL.bit.SHDWBMODE = CC_SHADOW;
	 EPwm1Regs.CMPCTL.bit.LOADAMODE = CC_CTR_ZERO; // load on CTR=Zero
	 EPwm1Regs.CMPCTL.bit.LOADBMODE = CC_CTR_ZERO; // load on CTR=Zero
	 EPwm1Regs.DBCTL.bit.OUT_MODE = 00;
}

interrupt void cpu_timer0_isr(void)
{
	PIcontrol ();
   	PieCtrlRegs.PIEACK.all = 0xFFFF;
}

////////  PI control  ////////
void PIcontrol(void){
		 analogy = AdcResult.ADCRESULT0; // 歸零校正
		 analogy = analogy - 2048 ;

	     avg();

	     if(AVGan >= 500){ 			//V=50V
	             EPwm1Regs.CMPA.half.CMPA = 0;
	             EPwm1Regs.AQCTLB.bit.CAU = AQ_CLEAR; 		// set actions for EPWM1B
	             EPwm1Regs.AQCTLB.bit.CAD = AQ_CLEAR;
	             GpioDataRegs.GPASET.bit.GPIO6 = 1;        	// GPIO6  warn LED
	                     }
	     verr= ((Expectation * 20.48) - AVGan);
	     kp = (verr * 0.01);

	     ki = (verrR + verr) * 5;
	     kiR = (kiR + (ki >> 14));

	 	 if(kiR >= 500){kiR = 500;}
	     if(kiR <= -500){kiR = -500;}

	     verrR = verr;

	     pierr= kp + kiR;

	     if(pierr >= 500){
		 	pierr = 500;}
	     if(pierr <= 0){
		 	pierr = 0;}  ㄛ

	     swADC();
}

void swADC(void)
{
     swadc = GpioDataRegs.GPADAT.bit.GPIO4;

     if(swadc == 1 ){ //pro 保護
      j++;
     	if(j > 10){
           GpioDataRegs.GPASET.bit.GPIO5 = 1;
           if(i < 256){
	               AVGa[i]= AVGan;  	//0x00009440
	                ana[i]= analogy; 	//0x00009040
                    err[i]= verr;
                      p[i]= kp;  		//0x00009540
                    kii[i]= kiR;  		//0x00009240
                     pi[i]= pierr; 		//0x00009340
	              i++;
                     }
           else if(i >= 256){i = 266;}
				  }
		else if(j >= 20){i = 20;}
                     }
      else
      {  i--;
   		  if(i < 255){
               i=0;
   			   j=0;
                    }
         GpioDataRegs.GPACLEAR.bit.GPIO5 = 1;
      }
}

void swPWM (void){
	swpwm = GpioDataRegs.GPADAT.bit.GPIO6;        		//設置pwm開關
	  if( swpwm==1 && GpioDataRegs.GPADAT.bit.GPIO9 == 0)  // 取樣 110v  1126 (55)
		{
		 EPwm1Regs.CMPA.half.CMPA = pierr;	// Compare A = pierr TBCLK counts
		 EPwm1Regs.AQCTLB.bit.CAU = AQ_CLEAR;
		 EPwm1Regs.AQCTLB.bit.CAD = AQ_SET;
		 GpioDataRegs.GPASET.bit.GPIO7 = 1;
		}
	   else if( swpwm==0 )
	   {
		 EPwm1Regs.CMPA.half.CMPA= 0;
		 EPwm1Regs.AQCTLB.bit.CAU = AQ_CLEAR; 		// set actions for EPWM1B
		 EPwm1Regs.AQCTLB.bit.CAD = AQ_CLEAR;
		 GpioDataRegs.GPACLEAR.bit.GPIO7 = 1;
	   }
}

void avg()
{
	 if( ai >= 16 ){ ai=0; }// 平均值
	 a[ai]=  analogy;
     ai++;
	 AVGan=0;
     for(bi=0;bi<16;bi++){
     AVGan=  AVGan + a[bi];}
     AVGan= AVGan >> 4 ;	//向右移4格=除以16
}

