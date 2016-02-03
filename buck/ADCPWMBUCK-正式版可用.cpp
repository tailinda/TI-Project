//###########################################################################
// Description:
//! \addtogroup f2803x_example_list
//! <h1> ADC Start of Conversion (adc_soc)</h1>
//!
//! This ADC example uses ePWM1 to generate a periodic ADC SOC - ADCINT1.
//! Two channels are converted, ADCINA4 and ADCINA2.
//!
//! \b Watch \b Variables \n
//! - Voltage1[10]    - Last 10 ADCRESULT0 values
//! - Voltage2[10]    - Last 10 ADCRESULT1 values
//! - ConversionCount - Current result number 0-9
//! - LoopCount       - Idle loop counter
//
//
//###########################################################################
// $TI Release: F2803x C/C++ Header Files and Peripheral Examples V130 $
// $Release Date: May  8, 2015 $
// $Copyright: Copyright (C) 2009-2015 Texas Instruments Incorporated -
//             http://www.ti.com/ ALL RIGHTS RESERVED $
//###########################################################################

#include "DSP28x_Project.h"     // Device Headerfile and Examples Include File
typedef struct {
	volatile struct EPWM_REGS *EPwmRegHandle;
	Uint16 EPwm_CMPA_Direction;
	Uint16 EPwm_CMPB_Direction;
	Uint16 EPwmTimerIntCount;
	Uint16 EPwmMaxCMPA;
	Uint16 EPwmMinCMPA;
	Uint16 EPwmMaxCMPB;
	Uint16 EPwmMinCMPB;
} EPWM_INFO;
// Prototype statements for functions found within this file.
__interrupt void adc_isr(void);
__interrupt void xint1_isr(void);
__interrupt void xint2_isr(void);
__interrupt void cputimer_isr(void);
void Adc_Config(void);
void InitEPwm1Example(void);

// Global variables used in this example:
Uint16 LoopCount;
Uint16 ConversionCount;
Uint16 Voltage1[255];
int xintcount = 0;
float  v1, v2, verr, vk1, vk2, vout, vr, z1=0, z2=0, vko1, vko2, vou1, vou2;
float k1, k2 ;
extern Uint16 RamfuncsLoadStart;
extern Uint16 RamfuncsLoadEnd;
extern Uint16 RamfuncsRunStart;

int array = 0;
main() {
	InitSysCtrl(); // 初始化system
	DINT;
	// 初始化PIE
	InitPieCtrl();
	// 初始化ADC
	InitPieVectTable();
	MemCopy(&RamfuncsLoadStart, &RamfuncsLoadEnd, &RamfuncsRunStart);
	InitFlash();
	//AdcOffsetSelfCal();
	InitAdc();

	IER = 0x0000;
	IFR = 0x0000;
	// 初始化中斷向量表

	InitEPwm1Gpio();
	GpioCtrlRegs.GPAMUX1.bit.GPIO0 = 1;  // GPIO0 = PWM1A
	GpioCtrlRegs.GPADIR.bit.GPIO0 = 1;   //GPIO0 = output
	EALLOW;
	SysCtrlRegs.PCLKCR0.bit.TBCLKSYNC = 0;
	InitEPwm1Example();

	InitCpuTimers();
	ConfigCpuTimer(&CpuTimer0, 1, 3906); //必須先配置  再致能 !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
	CpuTimer0Regs.TCR.all = 0x4000;   // 致能CPUTIMER0

	EDIS;

	// 設定中斷向量表位置
	EALLOW;
	PieVectTable.XINT1 = &xint1_isr;
	PieVectTable.XINT2 = &xint2_isr;
	PieVectTable.ADCINT1 = &adc_isr;
	PieVectTable.TINT0 = &cputimer_isr;
	EDIS;

	// 設置外部中斷腳位及燈號腳位
	EALLOW;
	GpioCtrlRegs.GPAMUX1.bit.GPIO1 = 0;         // GPIO ADC開關
	GpioCtrlRegs.GPADIR.bit.GPIO1 = 0;          // input
	GpioCtrlRegs.GPAQSEL1.bit.GPIO1 = 0;        // XINT1 Synch to SYSCLKOUT only
	GpioIntRegs.GPIOXINT1SEL.bit.GPIOSEL = 1;   // XINT1 is GPIO1
	GpioCtrlRegs.GPAPUD.bit.GPIO1 = 0;

	GpioCtrlRegs.GPAMUX1.bit.GPIO2 = 0;         // GPIO
	GpioCtrlRegs.GPADIR.bit.GPIO2 = 1;          // OUTput
	GpioDataRegs.GPACLEAR.bit.GPIO2 = 1; 		// 當ADC ON/OFF 標示燈

	GpioCtrlRegs.GPAMUX1.bit.GPIO3 = 0;			// GPIO ePWM開關
	GpioCtrlRegs.GPADIR.bit.GPIO3 = 0;
	GpioCtrlRegs.GPAQSEL1.bit.GPIO3 = 0;        // XINT2 Synch to SYSCLKOUT only
	GpioIntRegs.GPIOXINT2SEL.bit.GPIOSEL = 3;   // XINT2 is GPIO3
	GpioCtrlRegs.GPAPUD.bit.GPIO3 = 0;  		//

	GpioCtrlRegs.GPAMUX1.bit.GPIO4 = 0;         // GPIO
	GpioCtrlRegs.GPADIR.bit.GPIO4 = 1;          // OUTput
	GpioDataRegs.GPACLEAR.bit.GPIO4 = 1; 		// 當 ePWM ON/OFF 標示燈

	GpioCtrlRegs.GPAMUX1.bit.GPIO6 = 0;         // GPIO
	GpioCtrlRegs.GPADIR.bit.GPIO6 = 1;          // OUTput 閃爍用
	GpioDataRegs.GPACLEAR.bit.GPIO6 = 1;
	EDIS;

	IER |= M_INT1;  // 開啟GROUP1中斷
	EINT;
	ERTM;

	EALLOW;
	PieCtrlRegs.PIECTRL.bit.ENPIE = 1;  // 致能PIE(非必要)
	PieCtrlRegs.PIEIER1.bit.INTx1 = 0;
	PieCtrlRegs.PIEIER1.bit.INTx4 = 1;
	PieCtrlRegs.PIEIER1.bit.INTx5 = 1;
	//PieCtrlRegs.PIEIER1.bit.INTx7 = 1;  // !!!!!!!! 不能同時開啟  , 否則打架  !!!!!!!!!
	XIntruptRegs.XINT1CR.bit.POLARITY = 3; // interrupt occur on both falling and rising edge
	XIntruptRegs.XINT1CR.bit.ENABLE = 1; // Enable Xint1
	XIntruptRegs.XINT2CR.bit.POLARITY = 3; // interrupt occur on both falling and rising edge
	XIntruptRegs.XINT2CR.bit.ENABLE = 1; // Enable Xint2
	EDIS;

	LoopCount = 0;
	ConversionCount = 0;

// Configure ADC
// Note: Channel ADCINA4  will be double sampled to workaround the ADC 1st sample issue for rev0 silicon errata

	EALLOW;
	AdcRegs.ADCCTL1.bit.ADCENABLE = 0;  // 不致能ADC (在initadc中已經致能)
	AdcRegs.ADCCTL1.bit.INTPULSEPOS = 1; //ADCINT1 trips after AdcResults latch

	AdcRegs.INTSEL1N2.bit.INT1E = 0;	//disabled ADCINT1
	AdcRegs.INTSEL1N2.bit.INT1CONT = 0;	//Disable ADCINT1 Continuous mode
	AdcRegs.INTSEL1N2.bit.INT1SEL = 0;	//setup EOC0 to trigger ADCINT1 to fire

	AdcRegs.ADCSOC0CTL.bit.CHSEL = 1;	//set SOC0 channel select to ADCINA1
	AdcRegs.ADCSOC0CTL.bit.TRIGSEL = 1;	//set SOC0 start trigger on CPUtimer
	AdcRegs.ADCSOC0CTL.bit.ACQPS = 6;//set SOC0 S/H Window to 7 ADC Clock Cycles, (6 ACQPS plus 1)
	//AdcRegs.ADCINTFLGCLR.bit.ADCINT1 = 1;
	EDIS;

	//GpioCtrlRegs.AIOMUX1.bit. &= ~(0x08);  // ADCINA1
//
	EALLOW;
	SysCtrlRegs.XCLK.bit.XCLKOUTDIV = 2;
	EDIS;

	for (;;) {
		//volatile long double n;
		//for (n = 0; n < 5000; n++)
		//	;
		//EALLOW;
		//GpioDataRegs.GPATOGGLE.bit.GPIO6 = 1;
		//EDIS;

		if (GpioDataRegs.GPADAT.bit.GPIO1 == 1) {
			Voltage1[array] = AdcResult.ADCRESULT0;

			if (Voltage1[array] > 5000) {
				EALLOW;
				SysCtrlRegs.PCLKCR1.bit.EPWM1ENCLK = 0;
				SysCtrlRegs.PCLKCR0.bit.TBCLKSYNC = 0;
				SysCtrlRegs.LPMCR0.bit.LPM = 3;  // 全部休眠
				EDIS;
				
				EALLOW;
				SysCtrlRegs.CLKCTL.bit.WDHALTI = 1;  // 從休眠中開啟 , 全部重設!!
				EDIS;
			}

			v1 = 3031;	
			v2 = (int)Voltage1[array];
			k1 = 0.3;	//0.0001;
			k2 = 0.000055;
			verr = v1 - v2;
			vk2 = verr + z1;
			z1 = verr;
			vko2 =(int) vk2 * k2;
			if (vko2 >= 1400)
				{vko2 = 1400;}
			else if (vko2 <= -1300) 	///////////////////////
				{vko2 = -1300;}
			vou2 = vko2 + z2;
			z2 = vou2;
			vk1 = verr;
			vko1 =(int) vk1 * k1;
			vout =(int)( vou2 + vko1);
			if (vout >= 1400) {
				vout = 1400;
			} else if (vout <= 0) {
				vout = 0;

			}

			EALLOW;
			EPwm1Regs.CMPA.half.CMPA = 1500 - vout;     // Set compare A value
			EDIS;
			if (array < 255)
				array++;
			else
				array = 0;
		}
	}

}

__interrupt void xint1_isr(void) {
	EALLOW;
	//PieCtrlRegs.PIEIER1.bit.INTx7 = 0;
	PieCtrlRegs.PIEIER1.bit.INTx4 = 0;
	EDIS;

	if (GpioDataRegs.GPADAT.bit.GPIO1 == 0) {
		EALLOW;
		AdcRegs.ADCCTL1.bit.ADCENABLE = 0;
		GpioDataRegs.GPACLEAR.bit.GPIO2 = 1;
		//AdcRegs.INTSEL1N2.bit.INT1E = 0;
		AdcRegs.ADCINTFLGCLR.bit.ADCINT1 = 1;
		EDIS;
	} else if (GpioDataRegs.GPADAT.bit.GPIO1 == 1) {
		EALLOW;
		GpioDataRegs.GPASET.bit.GPIO2 = 1;
		AdcRegs.ADCCTL1.bit.ADCENABLE = 1;
		//AdcRegs.INTSEL1N2.bit.INT1E = 1;
		EDIS;
	}

	EALLOW;
	PieCtrlRegs.PIEIER1.bit.INTx4 = 1;
	//PieCtrlRegs.PIEIER1.bit.INTx7 = 1;
	PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;   // Acknowledge interrupt to PIE
	EDIS;

}
__interrupt void xint2_isr(void) {
	EALLOW;
	PieCtrlRegs.PIEIER1.bit.INTx5 = 0;
	//PieCtrlRegs.PIEIER1.bit.INTx7 = 0;
	EDIS;

	if (GpioDataRegs.GPADAT.bit.GPIO3 == 0) {
		EALLOW;
		EPwm1Regs.AQCTLA.bit.CAU = AQ_CLEAR;
		EPwm1Regs.AQCTLA.bit.CAD = AQ_CLEAR;
		//SysCtrlRegs.PCLKCR0.bit.TBCLKSYNC = 0 ;
		GpioDataRegs.GPACLEAR.bit.GPIO4 = 1;
		EDIS;
	} else if (GpioDataRegs.GPADAT.bit.GPIO3 == 1) {
		EALLOW;
		EPwm1Regs.AQCTLA.bit.CAU = AQ_SET;     // Set PWM1A on event A, up count
		EPwm1Regs.AQCTLA.bit.CAD = AQ_CLEAR; // Clear PWM1A on event A, down count
		SysCtrlRegs.PCLKCR0.bit.TBCLKSYNC = 1;
		GpioDataRegs.GPASET.bit.GPIO4 = 1;
		EDIS;

	}
	
	EALLOW;
	PieCtrlRegs.PIEIER1.bit.INTx5 = 1;
	//PieCtrlRegs.PIEIER1.bit.INTx7 = 1;
	PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;   // Acknowledge interrupt to PIE
	EDIS;
}

__interrupt void adc_isr(void) {

}

__interrupt void cputimer_isr(void) {
	if (GpioDataRegs.GPADAT.bit.GPIO1 == 1) {
		Voltage1[array] = AdcResult.ADCRESULT0;

		if (Voltage1[array] > 5000) {
			EALLOW;
			SysCtrlRegs.PCLKCR1.bit.EPWM1ENCLK = 0;
			SysCtrlRegs.PCLKCR0.bit.TBCLKSYNC = 0;
			SysCtrlRegs.LPMCR0.bit.LPM = 3;  // 全部休眠
			EDIS;
			EALLOW;
			SysCtrlRegs.CLKCTL.bit.WDHALTI = 1;  // 從休眠中開啟 , 全部重設!!
			EDIS;
		}

		v1 = 3031;
		v2 = Voltage1[array];
		k1 = 0.0001;
		k2 = 0.000055;
		verr = v1 - v2;
		vk2 = verr + z1;
		z1 = verr;
		vko2 = vk2 * k2;
		if (vko2 >= 1500)
			vko2 = 1500;
		else if (vou2 <= 0) ///////////////////////
			vou2 = 0;
		vou2 = vko2 + z2;
		z2 = vou2;
		vk1 = verr;
		vko1 = vk1 * k1;
		vout = vou2 + vko1;
		if (vout >= 1500) {
			vout = 1500;
		} else if (vout <= 0) {
			vout = 0;
		}

		if (GpioDataRegs.GPADAT.bit.GPIO3 == 1)
			SysCtrlRegs.PCLKCR0.bit.TBCLKSYNC = 1;
		vout = 1500 - (int) vout;
		EPwm1Regs.CMPA.half.CMPA = vout;     // Set compare A value

		if (array < 255)
			array++;
		else
			array = 0;
	}

}

void InitEPwm1Example() {
	SysCtrlRegs.PCLKCR1.bit.EPWM1ENCLK = 1;

	// Setup TBCLK
	EPwm1Regs.TBPRD = 1500;           // Set timer period 801 TBCLKs
	// EPwm1Regs.TBPHS.half.TBPHS = 0x0000;           // Phase is 0
	EPwm1Regs.TBCTR = 0x0000;                      // Clear counter

	// Set Compare values
	EPwm1Regs.CMPA.half.CMPA = 500;     // Set compare A value

	// Setup counter mode
	EPwm1Regs.TBCTL.bit.CTRMODE = TB_COUNT_UPDOWN; // Count up
	EPwm1Regs.TBCTL.bit.PHSEN = 0x00;        // Enable phase loading
	EPwm1Regs.TBCTL.bit.HSPCLKDIV = 0;       // Clock ratio to SYSCLKOUT
	EPwm1Regs.TBCTL.bit.CLKDIV = 0;
	EPwm1Regs.TBCTL.bit.SYNCOSEL = 0x01;

	// Setup shadowing
	EPwm1Regs.CMPCTL.bit.SHDWAMODE = CC_SHADOW;
	EPwm1Regs.CMPCTL.bit.SHDWBMODE = CC_SHADOW;
	EPwm1Regs.CMPCTL.bit.LOADAMODE = CC_CTR_ZERO;  // Load on Zero
	EPwm1Regs.CMPCTL.bit.LOADBMODE = CC_CTR_ZERO;

	// Set actions
	EPwm1Regs.AQCTLA.bit.CAU = AQ_SET;         // Set PWM1A on event A, up count
	EPwm1Regs.AQCTLA.bit.CAD = AQ_CLEAR;   // Clear PWM1A on event A, down count

	EPwm1Regs.DBCTL.bit.IN_MODE = 0;
	EPwm1Regs.DBCTL.bit.POLSEL = 2;
	EPwm1Regs.DBCTL.bit.OUT_MODE = 0;
	//EPwm1Regs.DBRED = 300;
	//EPwm1Regs.DBFED = 300;
}

