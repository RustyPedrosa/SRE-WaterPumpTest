/*****************************************************************************
* SRE-2 Vehicle Control Firmware for the TTTech HY-TTC 50 Controller (VCU)
******************************************************************************
* For project info and history, see https://github.com/spartanracingelectric/SRE-2
* For software/development questions, email rusty@pedrosatech.com
******************************************************************************
* Files
* The Git repository does not contain the complete firmware for SRE-2.  Modules
* provided by TTTech can be found on the CD that accompanied the VCU. These 
* files can be identified by our naming convetion: TTTech files start with a
* prefix in all caps (such as IO_Driver.h), except for ptypes_xe167.h which
* they also provided.
* For instructions on setting up a build environment, see the SRE-2 getting-
* started document, Programming for the HY-TTC 50, at http://1drv.ms/1NQUppu
******************************************************************************
* Organization
* Our code is laid out in the following manner:
* 
*****************************************************************************/

//-------------------------------------------------------------------
//VCU Initialization Stuff
//-------------------------------------------------------------------

//VCU/C headers
#include <stdio.h>
#include <string.h>
#include "APDB.h"
#include "IO_Driver.h"  //Includes datatypes, constants, etc - should be included in every c file
#include "IO_RTC.h"
#include "IO_DIO.h"
#include "IO_ADC.h"
#include "IO_PWM.h"
//#include "IO_CAN.h"

//Our code
//#include "initializations.h"

//Application Database, needed for TTC-Downloader
APDB appl_db =
    { 0                      /* ubyte4 versionAPDB        */
    ,{ 0 }                    /* BL_T_DATE flashDate       */
                          /* BL_T_DATE buildDate                   */
    ,{ (ubyte4)(((((ubyte4)RTS_TTC_FLASH_DATE_YEAR) & 0x0FFF) << 0) |
        ((((ubyte4)RTS_TTC_FLASH_DATE_MONTH) & 0x0F) << 12) |
        ((((ubyte4)RTS_TTC_FLASH_DATE_DAY) & 0x1F) << 16) |
        ((((ubyte4)RTS_TTC_FLASH_DATE_HOUR) & 0x1F) << 21) |
        ((((ubyte4)RTS_TTC_FLASH_DATE_MINUTE) & 0x3F) << 26)) }
    , 0                      /* ubyte4 nodeType           */
    , 0                      /* ubyte4 startAddress       */
    , 0                      /* ubyte4 codeSize           */
    , 0                      /* ubyte4 legacyAppCRC       */
    , 0                      /* ubyte4 appCRC             */
    , 1                      /* ubyte1 nodeNr             */
    , 0                      /* ubyte4 CRCInit            */
    , 0                      /* ubyte4 flags              */
    , 0                      /* ubyte4 hook1              */
    , 0                      /* ubyte4 hook2              */
    , 0                      /* ubyte4 hook3              */
    , APPL_START             /* ubyte4 mainAddress        */
    ,{ 0, 1 }                 /* BL_T_CAN_ID canDownloadID */
    ,{ 0, 2 }                 /* BL_T_CAN_ID canUploadID   */
    , 0                      /* ubyte4 legacyHeaderCRC    */
    , 0                      /* ubyte4 version            */
    , 500                    /* ubyte2 canBaudrate        */
    , 0                      /* ubyte1 canChannel         */
    ,{ 0 }                    /* ubyte1 reserved[8*4]      */
    , 0						         /* ubyte4 headerCRC          */
};

/*****************************************************************************
* Sensor
* Contains data returned from VCU about its sensors
****************************************************************************/
typedef struct _Sensor {
    bool fresh;       //Not used here, but this tells you whether the VCU has garbage (fresh == FALSE) or an actual value (fresh == TRUE).  Not all VCU functions return fresh.
    ubyte2 value;     //The sensor value returned by the VCU's function.  Note that certain functions return something other than a ubyte2.
    ubyte2 previous;  //We need to keep track of the previous value
    
} Sensor;

void Sensor_update (Sensor* sensor, ubyte2 newValue, bool newFresh)
{
    sensor->previous = sensor->value;
    sensor->value = newValue;
    sensor->fresh = newFresh;
}

/*****************************************************************************
* Main!
* Initializes I/O
* Contains sensor polling loop (always running)
****************************************************************************/
void main(void)
{
    /*******************************************/
    /*            Initializations              */
    /*******************************************/
    IO_Driver_Init(NULL); //Handles basic startup for all VCU subsystems

    //--------------------------------
    //Calculation variables
    //--------------------------------
    ubyte2 tempValue = 0;
    bool tempFresh = FALSE;
    ubyte2 waterPumpFrequency = 100;
    float4 waterPumpDutyPercent = 0;     //Duty cycle is a percent, here represented as a number between 0 and 1.

    bool changeFrequency = FALSE;

    IO_ErrorType error = IO_E_OK;

    //--------------------------------
    //Pin initializations and sensor/output variables
    //--------------------------------
    //Dash Lights
    IO_DO_Init(IO_ADC_CUR_00); IO_DO_Set(IO_ADC_CUR_00, FALSE); //TCS
    IO_DO_Init(IO_ADC_CUR_01); IO_DO_Set(IO_ADC_CUR_01, FALSE); //Eco
    IO_DO_Init(IO_ADC_CUR_02); IO_DO_Set(IO_ADC_CUR_02, FALSE); //Err
    IO_DO_Init(IO_ADC_CUR_03); IO_DO_Set(IO_ADC_CUR_03, FALSE); //RTD

    //PWM Outputs
    IO_PWM_Init(IO_PWM_02, 500, TRUE, FALSE, 0, FALSE, NULL); IO_PWM_SetDuty(IO_PWM_02, 0, NULL);  //Brake Light
    IO_PWM_Init(IO_PWM_05, waterPumpFrequency, TRUE, FALSE, 0, FALSE, NULL); IO_PWM_SetDuty(IO_PWM_05, waterPumpDutyPercent, NULL);  //Water pump signal

    Sensor Sensor_TCSKnob;
    Sensor_update(&Sensor_TCSKnob, 0, FALSE);
    IO_ADC_ChannelInit(IO_ADC_5V_04, IO_ADC_RESISTIVE, 0, 0, 0, NULL); //TCS Pot
    IO_ADC_Get(IO_ADC_5V_04, &tempValue, &tempFresh);

    Sensor Sensor_RTDButton;
    Sensor_update(&Sensor_TCSKnob, 0, FALSE);
    IO_DI_Init(IO_DI_00, IO_DI_PD_10K); //RTD Button
    IO_DI_Get(IO_DI_00, &tempValue);
    
    Sensor Sensor_EcoButton;
    Sensor_update(&Sensor_EcoButton, 0, FALSE);
    IO_DI_Init(IO_DI_01, IO_DI_PD_10K); //Eco Button
    IO_DI_Get(IO_DI_01, &tempValue);
    
    Sensor Sensor_TCSSwitch_Up;
    Sensor_update(&Sensor_TCSSwitch_Up, 0, FALSE);
    IO_DI_Init(IO_DI_02, IO_DI_PD_10K); //TCS Switch A
    IO_DI_Get(IO_DI_02, &tempValue);
    
    Sensor Sensor_TCSSwitch_Down;
    Sensor_update(&Sensor_TCSSwitch_Down, 0, FALSE);
    IO_DI_Init(IO_DI_03, IO_DI_PD_10K); //TCS Switch B
    IO_DI_Get(IO_DI_03, &tempValue);

    /*******************************************/
    /*       PERIODIC APPLICATION CODE         */
    /*******************************************/
    /* main loop, executed periodically with a defined cycle time (here: 5 ms) */
    ubyte4 timestamp_mainLoopStart = 0;
    //IO_RTC_StartTime(&timestamp_calibStart);
    while (1)
    {
        //----------------------------------------------------------------------------
        // Task management stuff (start)
        //----------------------------------------------------------------------------
        //Get a timestamp of when this task started from the Real Time Clock
        IO_RTC_StartTime(&timestamp_mainLoopStart);
        //Mark the beginning of a task - what does this actually do?
        IO_Driver_TaskBegin();

        //SerialManager_send(serialMan, "VCU has entered main loop.");

        
        /*******************************************/
        /*              Read Inputs                */
        /*******************************************/
        IO_ADC_Get(IO_ADC_5V_04, &tempValue, &tempFresh);
        Sensor_update(&Sensor_TCSKnob, tempValue, tempFresh);

        IO_DI_Get(IO_DI_00, &tempValue);
        Sensor_update(&Sensor_RTDButton, tempValue, tempFresh);
        
        IO_DI_Get(IO_DI_01, &tempValue);
        Sensor_update(&Sensor_EcoButton, tempValue, tempFresh);
        
        IO_DI_Get(IO_DI_02, &tempValue);
        Sensor_update(&Sensor_TCSSwitch_Up, tempValue, tempFresh);
        
        IO_DI_Get(IO_DI_03, &tempValue);
        Sensor_update(&Sensor_TCSSwitch_Down, tempValue, tempFresh);



        /*******************************************/
        /*          Perform Calculations           */
        /*******************************************/

        //--------------------------------
        //Calculate Duty Cycle
        //--------------------------------
        waterPumpDutyPercent = (Sensor_TCSKnob.value - 50.0) / 900.0;
		if (waterPumpDutyPercent < 0) { waterPumpDutyPercent = 0; }
		if (waterPumpDutyPercent > 1) { waterPumpDutyPercent = 1; }

        //--------------------------------
        //Determine whether to change Frequency
        //--------------------------------
		//Only set changeFrequency to TRUE if the eco button was just pressed (and previously it was NOT pressed)
		changeFrequency = (Sensor_EcoButton.value == TRUE && Sensor_EcoButton.previous == FALSE);

        //--------------------------------
        //Calculate new Frequency
        //--------------------------------
        if (changeFrequency == TRUE)
        {
            //Determine next frequency setting to test
            switch (waterPumpFrequency)
            {
			    case 20:
					waterPumpFrequency = 40;
				break;
				case 50:
					waterPumpFrequency = 100;
				break;

                case 100:
                    waterPumpFrequency = 125;
                break;
                
                case 125:
                    waterPumpFrequency = 150;
                break;
                
                case 150:
                    waterPumpFrequency = 200;
                break;
                
                case 200:
                    waterPumpFrequency = 250;
                break;
                
                case 250:
                    waterPumpFrequency = 500;
                break;
                
                case 500:
                    waterPumpFrequency = 1000;
                break;

                case 1000:
				default:
                    waterPumpFrequency = 20;
                break;
            }
        }

        /*******************************************/
        /*              Enact Outputs              */
        /*******************************************/
		//Lights
		IO_DO_Set(IO_ADC_CUR_00, waterPumpFrequency == 100 ? TRUE : FALSE); //Set TCS light on at _ frequency
		IO_DO_Set(IO_ADC_CUR_01, Sensor_EcoButton.value);  //Set eco light if button is held
		IO_DO_Set(IO_ADC_CUR_02, waterPumpDutyPercent > 0 ? TRUE : FALSE); //Set Err light on if water pump > 0% duty
		IO_DO_Set(IO_ADC_CUR_03, changeFrequency); //Turn on RTD light if frequency is being changed (should only blink for duration of 1 loop of main)

		IO_PWM_SetDuty(IO_PWM_02, 0xFFFF * (waterPumpFrequency / 1000.0), NULL);  //Set brake light brightness based on frequency (brighter = faster frequency)
		//IO_PWM_SetDuty(IO_PWM_02, 65535 * waterPumpDutyPercent, NULL);  //Set brake light brightness based on duty cycle (brighter = higher duty cycle)
        
        //Update the water pump frequency if needed
        if (changeFrequency == TRUE)
        {
            IO_PWM_DeInit(IO_PWM_05);
            IO_PWM_Init(IO_PWM_05, waterPumpFrequency, TRUE, FALSE, 0, FALSE, NULL);
			//changeFrequency = FALSE;
        }

        IO_PWM_SetDuty(IO_PWM_05, 0xFFFF * waterPumpDutyPercent, NULL);  //Water pump signal

        //----------------------------------------------------------------------------
        // Task management stuff (end)
        //----------------------------------------------------------------------------

        //Task end function for IO Driver - This function needs to be called at the end of every SW cycle
        IO_Driver_TaskEnd();

		//Don't start the next loop until _ time has passed
        while (IO_RTC_GetTimeUS(timestamp_mainLoopStart) < 5000) // 1000 = 1ms
        {
        }

    } //end of main loop

    //----------------------------------------------------------------------------
    // VCU Subsystem Deinitializations
    //----------------------------------------------------------------------------
    //IO_ADC_ChannelDeInit(IO_ADC_5V_00);
    //Free memory if object won't be used anymore

}


