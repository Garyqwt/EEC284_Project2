//*****************************************************************************
//
// Copyright (C) 2014 Texas Instruments Incorporated - http://www.ti.com/ 
// 
// 
//  Redistribution and use in source and binary forms, with or without 
//  modification, are permitted provided that the following conditions 
//  are met:
//
//    Redistributions of source code must retain the above copyright 
//    notice, this list of conditions and the following disclaimer.
//
//    Redistributions in binary form must reproduce the above copyright
//    notice, this list of conditions and the following disclaimer in the 
//    documentation and/or other materials provided with the   
//    distribution.
//
//    Neither the name of Texas Instruments Incorporated nor the names of
//    its contributors may be used to endorse or promote products derived
//    from this software without specific prior written permission.
//
//  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS 
//  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT 
//  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
//  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT 
//  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, 
//  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT 
//  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
//  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
//  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT 
//  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE 
//  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//
//*****************************************************************************

//*****************************************************************************
//
// Application Name     - SPI Demo
// Application Overview - The demo application focuses on showing the required 
//                        initialization sequence to enable the CC3200 SPI 
//                        module in full duplex 4-wire master and slave mode(s).
//
//*****************************************************************************


//*****************************************************************************
//
//! \addtogroup SPI_Demo
//! @{
//
//*****************************************************************************

// Standard includes
#include <string.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>

// Driverlib includes
#include "hw_types.h"
#include "hw_memmap.h"
#include "hw_common_reg.h"
#include "hw_ints.h"
#include "spi.h"
#include "rom.h"
#include "rom_map.h"
#include "utils.h"
#include "prcm.h"
#include "uart.h"
#include "interrupt.h"
#include "timer.h"

// Common interface includes
#include "uart_if.h"
#include "systick_if.h"
#include "timer_if.h"

#include "pin_mux_config.h"


#define APPLICATION_VERSION     "1.4.0"
//*****************************************************************************
//
// Application Master/Slave mode selector macro
//
// MASTER_MODE = 1 : Application in master mode
// MASTER_MODE = 0 : Application in slave mode
//
//*****************************************************************************
#define MASTER_MODE      1

#define SPI_IF_BIT_RATE  10000 //10000
#define TR_BUFF_SIZE     100

#define MASTER_MSG       "This is CC3200 SPI Master Application\n\r"
#define SLAVE_MSG        "This is CC3200 SPI Slave Application\n\r"

#define TIMER_FREQ      80000000

#define NUMSAMPLE       5000

#define SYSCLK          80000000

// This is the time we delay inside the A0 timer interrupt handler.
#define SLOW_TIMER_DELAY_uS 2000

// maximum number of inputs that can be handled
// in one function call
#define MAX_INPUT_LEN   2000
// maximum length of filter than can be handled
#define MAX_FLT_LEN     63
// buffer to hold all of the input samples
#define BUFFER_LEN      (MAX_FLT_LEN - 1 + MAX_INPUT_LEN)

#define FILTER_LEN  5
double coeffs[ FILTER_LEN ] =
{
  -0.0110, 0.0939, 0.4895, 0.4895, 0.0939, -0.0110
};

//*****************************************************************************
//                 GLOBAL VARIABLES -- Start
//*****************************************************************************
static unsigned short g_ucTxBuff[TR_BUFF_SIZE];
static unsigned short g_ucRxBuff[TR_BUFF_SIZE];
static unsigned char ucTxBuffNdx;
static unsigned char ucRxBuffNdx;
unsigned long TimerInts;
static int Start = 0;

static unsigned long g_ulSamples[2];
static unsigned long g_ulFreq;

static unsigned long g_ulA2IntCount;

// array to hold input samples
double insamp[2000];

#if defined(ccs)
extern void (* const g_pfnVectors[])(void);
#endif
#if defined(ewarm)
extern uVectorEntry __vector_table;
#endif
//*****************************************************************************
//                 GLOBAL VARIABLES -- End
//*****************************************************************************


//*****************************************************************************
//
//! SPI Slave Interrupt handler
//!
//! This function is invoked when SPI slave has its receive register full or
//! transmit register empty.
//!
//! \return None.
//
//*****************************************************************************
static void SlaveIntHandler()
{
    unsigned long ulRecvData;
    unsigned long ulStatus;

    ulStatus = MAP_SPIIntStatus(GSPI_BASE,true);

    MAP_SPIIntClear(GSPI_BASE,SPI_INT_RX_FULL|SPI_INT_TX_EMPTY);

    if(ulStatus & SPI_INT_TX_EMPTY)
    {
        MAP_SPIDataPut(GSPI_BASE,g_ucTxBuff[ucTxBuffNdx%TR_BUFF_SIZE]);
        ucTxBuffNdx++;
    }

    if(ulStatus & SPI_INT_RX_FULL)
    {
        MAP_SPIDataGetNonBlocking(GSPI_BASE,&ulRecvData);
        g_ucTxBuff[ucRxBuffNdx%TR_BUFF_SIZE] = ulRecvData;
        Report("%c",ulRecvData);
        ucRxBuffNdx++;
    }
}


//*****************************************************************************
//
//! \TimerA0IntHandler
//!
//! Handles interrupts for Timer A2.
//! This interrupt handler clears the source of the interrupt and
//! increments a counter and returns.
//!
//! \param None.
//!
//! \return None.
//
//*****************************************************************************
static void
TimerA0IntHandler(void)
{
    unsigned long ulStatus;

    //
    // Clear all interrupts for Timer.
    //
    ulStatus = TimerIntStatus(TIMERA0_BASE, true);
    TimerIntClear(TIMERA0_BASE, ulStatus);

    //
    // Increment our global interrupt counter.
    //
    g_ulA2IntCount++;

}

//*****************************************************************************
//
//! SPI Master mode main loop
//!
//! This function configures SPI module as master and enables the channel for
//! communication
//!
//! \return None.
//
//*****************************************************************************
void MasterMain()
{

    unsigned long ulUserData;
    unsigned long ulDummy;

    //
    // Initialize the message
    //
    memcpy(g_ucTxBuff,MASTER_MSG,sizeof(MASTER_MSG));

    //
    // Set Tx buffer index
    //
    ucTxBuffNdx = 0;
    ucRxBuffNdx = 0;

    //
    // Reset SPI
    //
    SPIReset(GSPI_BASE);

    //
    // Configure SPI interface
    //
    SPIConfigSetExpClk(GSPI_BASE,MAP_PRCMPeripheralClockGet(PRCM_GSPI),
                     SPI_IF_BIT_RATE,SPI_MODE_MASTER,SPI_SUB_MODE_0,
                     (SPI_SW_CTRL_CS |
                     SPI_4PIN_MODE |
                     SPI_TURBO_OFF |
                     SPI_CS_ACTIVELOW |
                     SPI_WL_16));

    //
    // Enable SPI for communication
    //
    SPIEnable(GSPI_BASE);

    //
    // Print mode on uart
    //
    Message("Enabled SPI Interface in Master Mode\n\r");


    //
    // Send the string to slave. Chip Select(CS) needs to be
    // asserted at start of transfer and deasserted at the end.
    //
    SPITransfer(GSPI_BASE,g_ucTxBuff,g_ucRxBuff,50,
            SPI_CS_ENABLE|SPI_CS_DISABLE);

    //
    // Initialize variable
    //
    ulUserData = 0;


}

//*****************************************************************************
//
//! SPI Slave mode main loop
//!
//! This function configures SPI modelue as slave and enables the channel for
//! communication
//!
//! \return None.
//
//*****************************************************************************
void SlaveMain()
{
    //
    // Initialize the message
    //
    memcpy(g_ucTxBuff,SLAVE_MSG,sizeof(SLAVE_MSG));

    //
    // Set Tx buffer index
    //
    ucTxBuffNdx = 0;
    ucRxBuffNdx = 0;

    //
    // Reset SPI
    //
    MAP_SPIReset(GSPI_BASE);

    //
    // Configure SPI interface
    //
    MAP_SPIConfigSetExpClk(GSPI_BASE,MAP_PRCMPeripheralClockGet(PRCM_GSPI),
                     SPI_IF_BIT_RATE,SPI_MODE_SLAVE,SPI_SUB_MODE_0,
                     (SPI_HW_CTRL_CS |
                     SPI_4PIN_MODE |
                     SPI_TURBO_OFF |
                     SPI_CS_ACTIVEHIGH |
                     SPI_WL_8));

    //
    // Register Interrupt Handler
    //
    MAP_SPIIntRegister(GSPI_BASE,SlaveIntHandler);

    //
    // Enable Interrupts
    //
    MAP_SPIIntEnable(GSPI_BASE,SPI_INT_RX_FULL|SPI_INT_TX_EMPTY);

    //
    // Enable SPI for communication
    //
    MAP_SPIEnable(GSPI_BASE);

    //
    // Print mode on uart
    //
    Message("Enabled SPI Interface in Slave Mode\n\rReceived : ");
}

//*****************************************************************************
//
//! Board Initialization & Configuration
//!
//! \param  None
//!
//! \return None
//
//*****************************************************************************
static void
BoardInit(void)
{
/* In case of TI-RTOS vector table is initialize by OS itself */
#ifndef USE_TIRTOS
  //
  // Set vector table base
  //
#if defined(ccs)
    MAP_IntVTableBaseSet((unsigned long)&g_pfnVectors[0]);
#endif
#if defined(ewarm)
    MAP_IntVTableBaseSet((unsigned long)&__vector_table);
#endif
#endif
    //
    // Enable Processor
    //
    MAP_IntMasterEnable();
    MAP_IntEnable(FAULT_SYSTICK);

    PRCMCC3200MCUInit();
}



/***************************************************************************************
*    Title: Filter Code Definitions
*    Author: Shawn
*    Date: 10/29/2019
*    Code version: 1.0
*    Availability: https://sestevenson.wordpress.com/implementation-of-fir-filtering-in-c-part-1/
*
***************************************************************************************/


// FIR init
void firFloatInit( void )
{
    memset( insamp, 0, sizeof( insamp ) );
}

// the FIR filter function
void firFloat( double *coeffs, double *input, double *output,
       int length, int filterLength )
{
    double acc;     // accumulator for MACs
    double *coeffp; // pointer to coefficients
    double *inputp; // pointer to input samples
    int n;
    int k;

    // put the new samples at the high end of the buffer
    memcpy( &insamp[filterLength - 1], input,
            length * sizeof(double) );

    // apply the filter to each input sample
    for ( n = 0; n < length; n++ ) {
        // calculate output n
        coeffp = coeffs;
        inputp = &insamp[filterLength - 1 + n];
        acc = 0;
        for ( k = 0; k < filterLength; k++ ) {
            acc += (*coeffp++) * (*inputp--);
        }
        output[n] = acc;
    }
    // shift input samples back in time for next time
    memmove( &insamp[0], &insamp[length],
            (filterLength - 1) * sizeof(double) );

}




void exportToCSV(char *filename,double a[],int n){

    printf("\n Creating %s.csv file",filename);

    FILE *fp;

    int i,j;

    filename=strcat(filename,".csv");

    fp=fopen(filename,"w+");


    /* fopen() return NULL if last operation was unsuccessful */
    if(fp == NULL)
    {
        /* File not created hence exit */
        printf("Unable to create file.\n");
        exit(EXIT_FAILURE);
    }

    for(j=0;j<n;j++)    //n is the row
        fprintf(fp,"%lf\n",a[j]);

    fclose(fp);

    Report("\n %sfile created",filename);

    return;
}
/*
int main(){

    float a[750][1] = {};

    char str[100] = "Test";

    printf("\n Enter the filename :");

    //gets(str);

    exportToCSV(str,a,3,3);

    return 0;

}
*/

//Helper function to find local max
double findMax(double* InArray, int size, int* index, double* temp){
    int i = 0;
    temp = InArray;
    double max = -1;
    for(i = 0; i < size; i++){
        if(temp[i] > max){
            max = temp[i];
            *index = i;
        }
    }



    return max;

}


//Helper function to find local min
double findMin(double* InArray, int size, int* index, double* temp){
    int i = 0;

    temp = InArray;
    double min = 10000;
    for(i = 0; i < size; i++){
        if(temp[i] < min){
            min = temp[i];
            *index = i;
        }
    }
    return min;

}




//*****************************************************************************
//
//! Main function for spi demo application
//!
//! \param none
//!
//! \return None.
//
//*****************************************************************************
void main()
{
    //timer_t aTimer;
    uint16_t ulRecvData;
    int sampSize = 0;
    int j = 0;
    int k = 0;
    int p = 0;
    double realV = 0;
    unsigned long prevValue = -1;
    unsigned long ulStatus;
    double *tempArray;
    double localMax = 0;
    double localMin = 0;
    double time_taken = 0;
    //time_t current_time;


    double floatInput[NUMSAMPLE];          //2000
    double floatOutput[NUMSAMPLE];

    //
    // Initialize Board configurations
    //
    BoardInit();

    //
    // Muxing UART and SPI lines.
    //
    PinMuxConfig();

    //
    // Enable the SPI module clock
    //
    PRCMPeripheralClkEnable(PRCM_GSPI,PRCM_RUN_MODE_CLK);

    //
    // Initializing the Terminal.
    //
    InitTerm();

    //
    // Clearing the Terminal.
    //
    ClearTerm();

    //
    // Display the Banner
    //
    Message("\n\n\n\r");
    Message("\t\t   ********************************************\n\r");
    Message("\t\t        Heart Rate Detection Application  \n\r");
    Message("\t\t   ********************************************\n\r");
    Message("\n\n\n\r");

    //
    // Reset the peripheral
    //
    PRCMPeripheralReset(PRCM_GSPI);

#if MASTER_MODE

    MasterMain();

#else

    SlaveMain();

#endif

    prevValue = -1;
    g_ulA2IntCount = 0;


    // Configuring the timers
    //
    Timer_IF_Init(PRCM_TIMERA0, TIMERA0_BASE, TIMER_CFG_PERIODIC, TIMER_A, 0);

    //
    // Setup the interrupts for the timer timeouts.
    //
    Timer_IF_IntSetup(TIMERA0_BASE, TIMER_A, TimerA0IntHandler);

    Timer_IF_Start(TIMERA0_BASE, TIMER_A, 10); //10ms

    /*
    current_time = time(NULL);
    if (current_time == ((time_t)-1)){
        Message("Failure to obtain the current time.\n");
        exit(EXIT_FAILURE);
    }
    */

    while(j<NUMSAMPLE)
    {
        //
        // Enable Chip select
        //
        SPICSEnable(GSPI_BASE);

        SPIDataPut(GSPI_BASE,g_ucTxBuff[ucTxBuffNdx%TR_BUFF_SIZE]);
        ucTxBuffNdx++;

        SPIDataGet(GSPI_BASE,&ulRecvData);
        g_ucTxBuff[ucRxBuffNdx%TR_BUFF_SIZE] = ulRecvData;
        ulRecvData = ulRecvData >> 3;
        ulRecvData = (ulRecvData & 0x7FF);

        realV = (double)ulRecvData * 3.3 / 1024;

        if(g_ulA2IntCount != prevValue){
            prevValue = g_ulA2IntCount;
            //Report("%lf\n\r",realV);
            floatInput[sampSize] = realV;
            sampSize++;
        }

        ucRxBuffNdx++;
        j++;

       //
       // Disable chip select
       //
       SPICSDisable(GSPI_BASE);
    }
    /*
    current_time=time(NULL)-current_time;
    time_taken=(double)current_time;

    Report("Time taken is %lf\n\r", time_taken);

    exportToCSV("DarkTest",floatInput,NUMSAMPLE);
    */

    sampSize = sampSize - 1;                        //Decrement by 1 since overcount
    // initialize the filter
    firFloatInit();
    firFloat(coeffs, floatInput, floatOutput,
             sampSize, FILTER_LEN);                //sampSize is the size of samples




    Message("*****************************************\n\r");
    Message("******       Filtered Signal      *******\n\r");
    Message("*****************************************\n\r");

    //Print the filtered data in the terminal
    /*
    for(k = 0; k < sampSize; k++){
        Report("%lf\n\r",floatOutput[k]);
    }
    */




    int tempMaxIdx = -1;
    int tempMinIdx = -1;
    int localMaxIdx = -1;
    int localMinIdx = -1;
    int numDrops = 0;
    int HBR = 0;
    int traverseSize = 10;
    double maxDiff = 0;
    double localDiff = 0;
    tempArray = (double *)malloc(sizeof(double) * traverseSize);       //malloc an array with size traverseSize

    double *tempMax;
    tempMax = (double *)malloc(traverseSize * sizeof(double));
    double *tempMin;
    tempMin = (double *)malloc(traverseSize * sizeof(double));


    for(k = 0; k < sampSize - traverseSize; k++){

        tempArray = &(floatOutput[k]);
        localMax = findMax(tempArray, traverseSize, &tempMaxIdx, tempMax);
        localMin = findMin(tempArray, traverseSize, &tempMinIdx, tempMin);
        localDiff = localMax - localMin;
        if(localDiff > maxDiff && (tempMaxIdx < tempMinIdx) && (localDiff < 0.1)){
            maxDiff = localDiff;
        }
    }


    k = 0;
    while(k < (sampSize - traverseSize)){
        tempArray = &(floatOutput[k]);
        localMax = findMax(tempArray, traverseSize, &tempMaxIdx, tempMax);
        localMin = findMin(tempArray, traverseSize, &tempMinIdx, tempMin);
        if((localMax-localMin) > (maxDiff * 0.6) && (tempMaxIdx < tempMinIdx)){//Drop found
            localMaxIdx = tempMaxIdx + k;
            localMinIdx = tempMinIdx + k;
            numDrops++;
            k = localMinIdx - 1;        //Increment by 1 outside
        }
        k++;
    }

    HBR = numDrops / ((double)(sampSize/100)) * 60;

    Report("Your Heart Beat Rate is %d bpm\n\r", HBR);


    Message("Completed\n\r");
    TimerDisable(TIMERA0_BASE, TIMER_A);
    TimerIntDisable(TIMERA0_BASE, TIMER_TIMA_TIMEOUT);
    return;

}



