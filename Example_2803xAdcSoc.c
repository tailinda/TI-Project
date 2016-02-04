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

// Prototype statements for functions found within this file.
__interrupt void adc_isr(void);
void Adc_Config(void);
//__interrupt void cpu_timer0_isr(void);

// Global variables used in this example:
Uint16 LoopCount;
Uint16 ConversionCount;
Uint16 Voltage1[10];
Uint16 Voltage2[10];

main()
{
// Step 1. Initialize System Control:
// PLL, WatchDog, enable Peripheral Clocks
// This example function is found in the DSP2803x_SysCtrl.c file.
   InitSysCtrl();

// Step 2. Initialize GPIO:
// This example function is found in the DSP2803x_Gpio.c file and
// illustrates how to set the GPIO to it's default state.
// InitGpio();  // Skipped for this example

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
   EALLOW;  // This is needed to write to EALLOW protected register
   PieVectTable.ADCINT1 = &adc_isr;
   EDIS;    // This is needed to disable write to EALLOW protected registers

// Step 4. Initialize all the Device Peripherals:
   InitAdc();  // For this example, init the ADC
   AdcOffsetSelfCal();

// Step 5. User specific code, enable interrupts:

// Enable ADCINT1 in PIE
   PieCtrlRegs.PIEIER1.bit.INTx1 = 1;	// Enable INT 1.1 in the PIE
   IER |= M_INT1; 						// Enable CPU Interrupt 1
   EINT;          						// Enable Global interrupt INTM
   ERTM;          						// Enable Global realtime interrupt DBGM

   LoopCount = 0;
   ConversionCount = 0;

   InitCpuTimers();
       /* EALLOW;
        PieVectTable.TINT0 = &cpu_timer0_isr;
        EDIS;*/

// Configure ADC
// Note: Channel ADCINA4  will be double sampled to workaround the ADC 1st sample issue for rev0 silicon errata

   EALLOW;
   AdcRegs.ADCCTL1.bit.INTPULSEPOS	= 1;	//ADCINT1 trips after AdcResults latch
   AdcRegs.INTSEL1N2.bit.INT1E      = 1;	//Enabled ADCINT1
   AdcRegs.INTSEL1N2.bit.INT1CONT   = 0;	//Disable ADCINT1 Continuous mode
   AdcRegs.INTSEL1N2.bit.INT1SEL	= 2;	//setup EOC2 to trigger ADCINT1 to fire
   AdcRegs.ADCSOC0CTL.bit.CHSEL 	= 1;	//set SOC0 channel select to ADCINA1(dummy sample for rev0 errata workaround)
   AdcRegs.ADCSOC0CTL.bit.TRIGSEL 	= 1;	//set SOC0 start trigger on CPUtimer0, due to round-robin SOC0 converts first then SOC1, then SOC2
   AdcRegs.ADCSOC0CTL.bit.ACQPS 	= 6;	//set SOC0 S/H Window to 7 ADC Clock Cycles, (6 ACQPS plus 1)
   EDIS;

   ConfigCpuTimer(&CpuTimer0, 1, 3906);    //3906 = 60M /( 60*256 )
        CpuTimer0Regs.TCR.all = 0x4001;         // TIE cpu-timer enable, TSS=0 CPU-timer running
        // Enable ADCINT1 in PIE
        PieCtrlRegs.PIECTRL.bit.ENPIE = 1;
        PieCtrlRegs.PIEIER1.bit.INTx7 = 1;      // Enable CPUTIMER 1.7 in the PIE
        IER |= M_INT1;                          // Enable CPU Interrupt 1
        EINT;          						 // Enable Global interrupt INTM
        ERTM;

// Wait for ADC interrupt
   for(;;)
   {
      LoopCount++;
   }
}

__interrupt void  adc_isr(void)
{
  Voltage1[ConversionCount] = AdcResult.ADCRESULT1; //discard ADCRESULT0 as part of the workaround to the 1st sample errata for rev0
  Voltage2[ConversionCount] = AdcResult.ADCRESULT2;

  // If 20 conversions have been logged, start over
  if(ConversionCount == 9)
  {
     ConversionCount = 0;
  }
  else
  {
      ConversionCount++;
  }

  AdcRegs.ADCINTFLGCLR.bit.ADCINT1 = 1;		//Clear ADCINT1 flag reinitialize for next SOC
  PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;   // Acknowledge interrupt to PIE

  return;
}

//__interrupt void cpu_timer0_isr(void){}



