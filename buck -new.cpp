#include "DSP28x_Project.h"     // Device Headerfile and Examples Include File

// Prototype statements for functions found within this file.
__interrupt void cpu_timer0_isr(void);
void initialepwm(void);
// Global variables used in this example:
int count = 0;
float Vset = 3030;   //期望值
float Vo[255]; //Vo取樣值
float Verr = 0;  //期望值 - Vo取樣值
float Vtest12 = 0;  //
float Vki1 = 0;  //
float Vki2 = 0;  //
float Vtest21 = 0;  //
float Vki = 0;  //
float Vkp = 0;  //
float Vtotal;
float P = 0.01;  // Proportional 比例
float I = 0.0004;  // 積分

//燒入程式用// 
extern Uint16 RamfuncsLoadStart;
extern Uint16 RamfuncsLoadEnd;
extern Uint16 RamfuncsRunStart;
//燒入程式用// 

main() {
// Step 1. Initialize System Control:
// PLL, WatchDog, enable Peripheral Clocks
// This example function is found in the DSP2803x_SysCtrl.c file.
	InitSysCtrl();
	//燒入程式用// 
	MemCopy(&RamfuncsLoadStart, &RamfuncsLoadEnd, &RamfuncsRunStart);	
	InitFlash();
	//燒入程式用// 

// Step 2. Initialize GPIO:
// This example function is found in the DSP2803x_Gpio.c file and
// illustrates how to set the GPIO to it's default state.
// InitGpio();  // Skipped for this example

// For this case just init GPIO pins for ePWM1, ePWM2, ePWM3
// These functions are in the DSP2803x_EPwm.c file
	InitEPwm1Gpio();

// Step 3. Clear all interrupts and initialize PIE vector table:
// Disable CPU interrupts
	DINT;

// Initialize the PIE control registers to their default state.
// The default state is all PIE interrupts disabled and flags
// are cleared.
// This function is found in the DSP2803x_PieCtrl.c file.
	InitPieCtrl();

// Disable CPU interrupts and clear all CPU interrupt flags:
	IER = 0x0000;
	IFR = 0x0000;

// Initialize the PIE vector table with pointers to the shell Interrupt
// Service Routines (ISR).
// This will populate the entire table, even if the interrupt
// is not used in this example.  This is useful for debug purposes.
// The shell ISR routines are found in DSP2803x_DefaultIsr.c.
// This function is found in DSP2803x_PieVect.c.
	InitPieVectTable();

// Interrupts that are used in this example are re-mapped to
// ISR functions found within this file.
	EALLOW;
	// This is needed to write to EALLOW protected register
	PieVectTable.TINT0 = &cpu_timer0_isr;	//選CPU_TIMER0 
	EDIS;
	// This is needed to disable write to EALLOW protected registers

// Step 4. Initialize all the Device Peripherals:
	InitAdc();  // For this example, init the ADC
	AdcOffsetSelfCal();
	InitCpuTimers(); 	//initialize Cpu Timers

//Configure CPU Timer0 to interrupt every second:
	ConfigCpuTimer(&CpuTimer0, 1, 3906);	//計算公式設置256 

// To ensure precise timing, use write-only instructions to write to the entire register. Therefore, if any
// of the configuration bits are changed in ConfigCpuTimer and InitCpuTimers (in DSP2803x_CpuTimers.h), the
// below settings must also be updated.
	CpuTimer0Regs.TCR.all = 0x4000; // Use write-only instruction to set TSS = 0

// Step 5. User specific code, enable interrupts:
// Enable ADCINT1 in PIE
	PieCtrlRegs.PIEIER1.bit.INTx7 = 1;	// Enable INT 1.7 in the PIE
	IER |= M_INT1; 						// Enable CPU Interrupt 1
	EINT;
	// Enable Global interrupt INTM
	ERTM;
	// Enable Global realtime interrupt DBGM

// Configure ADC
	EALLOW;
	AdcRegs.ADCCTL1.bit.INTPULSEPOS = 1;//ADCINT1 trips after AdcResults latch
	AdcRegs.ADCSOC0CTL.bit.CHSEL = 0;	//通道	set SOC0 channel select to ADCINA0
	AdcRegs.ADCSOC0CTL.bit.TRIGSEL = 1;	//觸發	set SOC0 start trigger on CPU Timer 0
	AdcRegs.ADCSOC0CTL.bit.ACQPS = 6;//窗口	set SOC0 S/H Window to 7 ADC Clock Cycles, (6 ACQPS plus 1)
	EDIS;

// Configure GPIO
	EALLOW;
	GpioCtrlRegs.GPAMUX1.bit.GPIO3 = 0;	//GPIO3 for General purpose I/O
	GpioCtrlRegs.GPAMUX1.bit.GPIO5 = 0;	//GPIO5 for General purpose I/O
	GpioCtrlRegs.GPAMUX1.bit.GPIO7 = 0;	//GPIO7 for General purpose I/O
	GpioCtrlRegs.GPAMUX1.bit.GPIO9 = 0;	//GPIO9 for General purpose I/O
	GpioCtrlRegs.GPADIR.bit.GPIO3 = 0;	//GPIO3__Input for ADC
	GpioCtrlRegs.GPADIR.bit.GPIO5 = 0;	//GPIO5__Input for ePWM
	GpioCtrlRegs.GPADIR.bit.GPIO7 = 1;	//GPIO5__Output LED for ADC
	GpioCtrlRegs.GPADIR.bit.GPIO9 = 1;	//GPIO5__Output LED for ePWM
	EDIS;
	initialepwm();
// Wait for interrupt
	for (;;) {
	}
}
//初始化PWM 
void initialepwm(void) {
	//GpioDataRegs.GPATOGGLE.bit.GPIO9 = 1;
	EALLOW;
	SysCtrlRegs.PCLKCR0.bit.TBCLKSYNC = 0;
	EDIS;
	//設定ePWM 的 TB CC AQ
	EPwm1Regs.TBPRD = 1500; // Set timer period; TPWM = 2 x TBPRD x TTBCLK, FPWM = 1 / (TPWM)
	EPwm1Regs.TBPHS.half.TBPHS = 0x0000;           // Phase is 0
	EPwm1Regs.TBCTR = 0x0000;                      // Clear counter
	// Setup TBCLK
	EPwm1Regs.TBCTL.bit.CTRMODE = TB_COUNT_UPDOWN; // Count up and down
	EPwm1Regs.TBCTL.bit.PHSEN = TB_DISABLE;        // Disable phase loading
	EPwm1Regs.CMPCTL.bit.SHDWAMODE = CC_SHADOW;    // Load registers every ZERO
	EPwm1Regs.CMPCTL.bit.SHDWBMODE = CC_SHADOW;
	EPwm1Regs.CMPCTL.bit.LOADAMODE = CC_CTR_ZERO;
	EPwm1Regs.CMPCTL.bit.LOADBMODE = CC_CTR_ZERO;
	// Setup compare
	EPwm1Regs.CMPA.half.CMPA = 1000;			  //set value to compare
	// Set actions
	EPwm1Regs.AQCTLA.bit.CAU = AQ_SET;           // Set PWM1A on Zero
	EPwm1Regs.AQCTLA.bit.CAD = AQ_CLEAR;
	EALLOW;
	SysCtrlRegs.PCLKCR0.bit.TBCLKSYNC = 1;	//設1才會讓PWM動作 
	EDIS;
}

__interrupt void cpu_timer0_isr(void) {
	if (GpioDataRegs.GPADAT.bit.GPIO3 == 1) {
		GpioDataRegs.GPATOGGLE.bit.GPIO7 = 1;
		if(count <= 255){
		Vo[count] = AdcResult.ADCRESULT0;
			//期望值 減去 取樣值
			Verr = Vset - Vo[count];
			// PI
			Vki1 = Verr + Vtest12;
			Vtest12 = Verr;
			Vki2 = Vki1 * I;
			Vki = Vki2 + Vtest21;
			if (Vki >= 1250) {
				Vki = 1250;
			}	    //注意限制值
			Vtest21 = Vki;

			if (Vki <= 0) {
				Vki = 0;
			}   //注意限制值
			Vkp = Verr * P;
			Vtotal = Vkp + Vki;
			if (Vtotal >= 1400) {
				Vtotal = 1350;
			}    //注意限制值
			if (Vtotal <= 0) {
				Vtotal = 0;
			}  //注意限制值
			count++;
		}//end of count
		else{count = 0;}
	}  //end of Gpio3-sampling vo

	if (GpioDataRegs.GPADAT.bit.GPIO3 == 0) {
		GpioDataRegs.GPACLEAR.bit.GPIO7 = 1;
	}  //end of sample end

	if (GpioDataRegs.GPADAT.bit.GPIO5 == 1) {
		EPwm1Regs.CMPA.half.CMPA = 1500-(int)Vtotal;
		GpioDataRegs.GPATOGGLE.bit.GPIO9 = 1;
		EPwm1Regs.AQCTLA.bit.CAU = AQ_SET;           // Set PWM1A on Zero
		EPwm1Regs.AQCTLA.bit.CAD = AQ_CLEAR;
	}           //end of ePWM Output
	if (GpioDataRegs.GPADAT.bit.GPIO5 == 0) {
		GpioDataRegs.GPACLEAR.bit.GPIO9 = 1;
		EALLOW;
		SysCtrlRegs.PCLKCR0.bit.TBCLKSYNC = 0;
		EDIS;
		EPwm1Regs.CMPA.half.CMPA = 0;
		// Set actions
		EPwm1Regs.AQCTLA.bit.CAU = AQ_CLEAR;           // Set PWM1A on Zero
		EPwm1Regs.AQCTLA.bit.CAD = AQ_CLEAR;
		EALLOW;
		SysCtrlRegs.PCLKCR0.bit.TBCLKSYNC = 1;
		EDIS;
	}           //end of ePWM Out end
	PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;   // Acknowledge interrupt to PIE
}

