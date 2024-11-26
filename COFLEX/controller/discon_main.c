/*
 * File    : discon_main.c
 * 
 * Abstract:
 *      A DISCON main for the use with GH BLADED that runs under most
 *      operating systems.
 *
 *      This file may be a useful starting point when targeting a new
 *      controller.
 *
 *  Updated 11-Apr-2017 by B. Jonkman, for Matlab R2017a
 *
 * Compiler specified defines:
 *	RT              - Required.
 *  MODEL=modelname - Required.
 *	NUMST=#         - Required. Number of sample times.
 *	NCSTATES=#      - Required. Number of continuous states.
 *  TID01EQ=1 or 0  - Optional. Only define to 1 if sample time task
 *                    id's 0 and 1 have equal rates.
 *  MULTITASKING    - Optional. (use MT for a synonym).
 *	SAVEFILE        - Optional (non-quoted) name of .mat file to create.
 *          		  Default is <MODEL>.mat
 */

/*=========*
 * Headers *
 *=========*/
#include <float.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
FILE *pFile; 
//#include <windows.h>

#include "rtwtypes.h"
#include "rtmodel.h"
#include "rt_sim.h"
#include "rt_logging.h"
#ifdef UseMMIDataLogging
#include "rt_logging_mmi.h"
#endif
#include "rt_nonfinite.h"
#include "ext_work.h"

/*=========*
 * Defines *
 *=========*/
 
// Generic helper definitions for shared library support
#if defined _WIN32 || defined __CYGWIN__
  #define DISCON_DLL_IMPORT __declspec(dllimport)
  #define DISCON_DLL_EXPORT __declspec(dllexport)
  #define DISCON_DLL_LOCAL
#else
  #if __GNUC__ >= 4
    #define DISCON_DLL_IMPORT __attribute__ ((visibility ("default")))
    #define DISCON_DLL_EXPORT __attribute__ ((visibility ("default")))
    #define DISCON_DLL_LOCAL  __attribute__ ((visibility ("hidden")))
  #else
    #define DISCON_DLL_IMPORT
    #define DISCON_DLL_EXPORT
    #define DISCON_DLL_LOCAL
  #endif
#endif 
 
#define DISCON_API DISCON_DLL_EXPORT
#define DISCON_LOCAL DISCON_DLL_LOCAL
 
// #  ifdef __GNUC__
// #    define CDECL __attribute__ ((__cdecl__))
// #  else
// #    define CDECL __cdecl
// #  endif 

#  ifdef __GNUC__
#    define CDECL 
#  else
#    define CDECL __cdecl
#  endif 
  
 
 
#ifndef TRUE
#define FALSE (0)
#define TRUE  (1)
#endif


#ifndef EXIT_FAILURE
#define EXIT_FAILURE  1
#endif
#ifndef EXIT_SUCCESS
#define EXIT_SUCCESS  0
#endif

#define QUOTE1(name) #name
#define QUOTE(name) QUOTE1(name)    /* need to expand name    */

#ifndef RT
#error "must define RT"
#endif

#ifndef MODEL
#error "must define MODEL"
#endif

#ifndef NUMST
#error "must define number of sample times, NUMST"
#endif

#ifndef NCSTATES
#error "must define NCSTATES"
#endif

#ifndef SAVEFILE
#define MATFILE2(file) #file ".mat"
#define MATFILE1(file) MATFILE2(file)
#define MATFILE MATFILE1(MODEL)
#else
#define MATFILE QUOTE(SAVEFILE)
#endif

#define RUN_FOREVER -1.0

#define EXPAND_CONCAT(name1,name2) name1 ## name2
#define CONCAT(name1,name2) EXPAND_CONCAT(name1,name2)
#define RT_MODEL            CONCAT(MODEL,_rtModel)

#define EXPAND_CONCAT3(name1,name2,name3) name1 ## _ ## name2.name3
#define CONCAT3(name1,name2,name3) EXPAND_CONCAT3(name1,name2,name3)
#define SIG_MODEL(suffix,name)     CONCAT3(MODEL,suffix,name)

#define NINT(a) ((a) >= 0.0 ? (int)((a)+0.5) : (int)((a)-0.5))
#define MIN(a,b) ((a)>(b)?(b):(a))

/*====================*
 * External functions *
 *====================*/
extern RT_MODEL *MODEL(void);

extern void MdlInitializeSizes(void);
extern void MdlInitializeSampleTimes(void);
extern void MdlStart(void);
extern void MdlOutputs(int_T tid);
extern void MdlUpdate(int_T tid);
extern void MdlTerminate(void);

#if NCSTATES > 0
  extern void rt_ODECreateIntegrationData(RTWSolverInfo *si);
  extern void rt_ODEUpdateContinuousStates(RTWSolverInfo *si);

# define rt_CreateIntegrationData(S) \
    rt_ODECreateIntegrationData(rtmGetRTWSolverInfo(S));
# define rt_UpdateContinuousStates(S) \
    rt_ODEUpdateContinuousStates(rtmGetRTWSolverInfo(S));
# else
# define rt_CreateIntegrationData(S)  \
      rtsiSetSolverName(rtmGetRTWSolverInfo(S),"FixedStepDiscrete");
# define rt_UpdateContinuousStates(S) /* Do Nothing */
#endif


/*==================================*
 * Global data local to this module *
 *==================================*/
static struct {
    int_T    stopExecutionFlag;
    int_T    isrOverrun;
    int_T    overrunFlags[NUMST];
    int_T    eventFlags[NUMST];
    const    char_T *errmsg;
} GBLbuf;

#ifdef EXT_MODE
#  define rtExtModeSingleTaskUpload(S)                          \
   {                                                            \
        int stIdx;                                              \
        rtExtModeUploadCheckTrigger(rtmGetNumSampleTimes(S));   \
        for (stIdx=0; stIdx<NUMST; stIdx++) {                   \
            if (rtmIsSampleHit(S, stIdx, 0 /*unused*/)) {       \
                rtExtModeUpload(stIdx,rtmGetTaskTime(S,stIdx)); \
            }                                                   \
        }                                                       \
   }
#else
#  define rtExtModeSingleTaskUpload(S) /* Do nothing */
#endif


/*=================*
 * Local functions *
 *=================*/

/* Function: initiateController ===========================================
 *
 * Abstract:
 *      Initialize the controller of the compiled Matlab Simulink block.
 */
static RT_MODEL *S;
DISCON_LOCAL int initiateController(char *errorMsg) {
    const char *status;
    
    /* Initialize global memory */
    //rtExtModeParseArgs(argc, argv, NULL);
    (void)memset(&GBLbuf, 0, sizeof(GBLbuf));
    
    /* Initialize the model */
    rt_InitInfAndNaN(sizeof(real_T));
    
    S = MODEL();
    if (rtmGetErrorStatus(S) != NULL) {
        (void)fprintf(stderr, "Error during model registration: %s\n",
                rtmGetErrorStatus(S));
        exit(EXIT_FAILURE);
        sprintf(errorMsg, "Error during model registration: %s",
                rtmGetErrorStatus(S));
        return -1;
    }
    rtmSetTFinal(S, RUN_FOREVER);
    
    MdlInitializeSizes();
    MdlInitializeSampleTimes();
    
    status = rt_SimInitTimingEngine(rtmGetNumSampleTimes(S),
            rtmGetStepSize(S),
            rtmGetSampleTimePtr(S),
            rtmGetOffsetTimePtr(S),
            rtmGetSampleHitPtr(S),
            rtmGetSampleTimeTaskIDPtr(S),
            rtmGetTStart(S),
            &rtmGetSimTimeStep(S),
            &rtmGetTimingData(S));
    
    if (status != NULL) {
        (void)fprintf(stderr,
                "Failed to initialize sample time engine: %s\n", status);
        exit(EXIT_FAILURE);
        sprintf(errorMsg, "Failed to initialize sample time engine: %s", status);
        return -1;
    }
    rt_CreateIntegrationData(S);
    
	#ifdef UseMMIDataLogging
    rt_FillStateSigInfoFromMMI(rtmGetRTWLogInfo(S),&rtmGetErrorStatus(S));
    rt_FillSigLogInfoFromMMI(rtmGetRTWLogInfo(S),&rtmGetErrorStatus(S));
	#endif
    GBLbuf.errmsg = rt_StartDataLogging(rtmGetRTWLogInfo(S),
			rtmGetTFinal(S),
            rtmGetStepSize(S),
            &rtmGetErrorStatus(S));
    if (GBLbuf.errmsg != NULL) {
        (void)fprintf(stderr, "Error starting data logging: %s\n", GBLbuf.errmsg);
        exit(EXIT_FAILURE);
        sprintf(errorMsg,  "Error starting data logging: %s", GBLbuf.errmsg);
        return -1;
    }
    rtExtModeCheckInit(rtmGetNumSampleTimes(S));
    rtExtModeWaitForStartPkt(rtmGetRTWExtModeInfo(S),
            rtmGetNumSampleTimes(S),
            (boolean_T *)&rtmGetStopRequested(S));
    
    
    sprintf(errorMsg, "** Starting the controller **");
    MdlStart();
    
    if (rtmGetErrorStatus(S) != NULL) {
        GBLbuf.stopExecutionFlag = 1;
    }
    
    return 0;
}  /* end initiateController */


/* Function: calcOutputController =========================================
 *
 * Abstract:
 *      Perform one step of the model. This function is modeled such that
 *      it could be called from an interrupt service routine (ISR) with
 *      minor modifications.
 */
#if !defined(MULTITASKING)  /* SINGLETASKING */
DISCON_LOCAL int calcOutputController(float rUserVar1, float rUserVar2, float rUserVar3, float rUserVar4, 
        float rUserVar5,float rUserVar6, float rUserVar7, float rUserVar8, float rUserVar9, float rUserVar10, 
        float rUserVar11,float rUserVar12,float rUserVar13,float rUserVar14,float rUserVar15,float rUserVar16,
        float rUserVar17,float rUserVar18,float rUserVar19,float rUserVar20, 
        float rGeneratorSpeed, float rRatedSpeed, float rBelowRatedPitch, float rForeAftTower, float rSideTower,
        float rRotorAzimuth, float rOP1RootMoment, float rOP2RootMoment,
        float rOP3RootMoment, float rMeasuredPitch, float rMeasuredTorque, 
        float rModeGain, float rHubWindSpeed, float *rTorqueDemand, float *rBlade1Pitch, 
        float *rBlade2Pitch, float *rBlade3Pitch, char *errorMsg) {
    real_T tnext;
    
    SIG_MODEL(U,Generator_Speed) = rGeneratorSpeed;
    SIG_MODEL(U,Rated_Speed) = rRatedSpeed;
    SIG_MODEL(U,Below_Rated_Pitch_Angle) = rBelowRatedPitch;
    SIG_MODEL(U,Fore_Aft_Tower_Accel) = rForeAftTower;
    SIG_MODEL(U,Sidewards_Tower_Accel) = rSideTower;
    SIG_MODEL(U,Measured_Pitch) = rMeasuredPitch;
    SIG_MODEL(U,Measured_Torque) = rMeasuredTorque;
    SIG_MODEL(U,Mode_Gain) = rModeGain;
    SIG_MODEL(U,Hub_Wind_Speed) = rHubWindSpeed;
    SIG_MODEL(U,Rotor_Azimuth_Angle) = rRotorAzimuth;
    SIG_MODEL(U,Blade1_OP_Root_Moment) = rOP1RootMoment;
    SIG_MODEL(U,Blade2_OP_Root_Moment) = rOP2RootMoment;
    SIG_MODEL(U,Blade3_OP_Root_Moment) = rOP3RootMoment;
    
    SIG_MODEL(U,userVar1) = rUserVar1;
    SIG_MODEL(U,userVar2) = rUserVar2;
    SIG_MODEL(U,userVar3) = rUserVar3;
    SIG_MODEL(U,userVar4) = rUserVar4;
    SIG_MODEL(U,userVar5) = rUserVar5;
    SIG_MODEL(U,userVar6) = rUserVar6;
    SIG_MODEL(U,userVar7) = rUserVar7;
    SIG_MODEL(U,userVar8) = rUserVar8;
    SIG_MODEL(U,userVar9) = rUserVar9;
    SIG_MODEL(U,userVar10) = rUserVar10;
    SIG_MODEL(U,userVar11) = rUserVar11;
    SIG_MODEL(U,userVar12) = rUserVar12;
    SIG_MODEL(U,userVar13) = rUserVar13;
    SIG_MODEL(U,userVar14) = rUserVar14;
    SIG_MODEL(U,userVar15) = rUserVar15;
    SIG_MODEL(U,userVar16) = rUserVar16;
    SIG_MODEL(U,userVar17) = rUserVar17;
    SIG_MODEL(U,userVar18) = rUserVar18;
    SIG_MODEL(U,userVar19) = rUserVar19;
    SIG_MODEL(U,userVar20) = rUserVar20;
    
    /* Check and see if base step time is too fast */
    if (GBLbuf.isrOverrun++) {
        GBLbuf.stopExecutionFlag = 1;
        sprintf(errorMsg, "GLbuf.isrOverrun");
        return -1;
    }
    
    /* Check and see if error status has been set */
    if (rtmGetErrorStatus(S) != NULL) {
        GBLbuf.stopExecutionFlag = 1;
        sprintf(errorMsg, "rtmGetErrorStatus(S) != NULL");
        return -1;
    }
    
    /* enable interrupts here */
    
    /*
     * In a multi-tasking environment, this would be removed from the base rate
     * and called as a "background" task.
     */
    rtExtModeOneStep(rtmGetRTWExtModeInfo(S),
                     rtmGetNumSampleTimes(S),					 
             (boolean_T *)&rtmGetStopRequested(S));
    
    tnext = rt_SimGetNextSampleHit();
    rtsiSetSolverStopTime(rtmGetRTWSolverInfo(S), tnext);
    
    MdlOutputs(0);
    
    rtExtModeSingleTaskUpload(S);
    
    GBLbuf.errmsg = rt_UpdateTXYLogVars(rtmGetRTWLogInfo(S),
                                        rtmGetTPtr(S));
    if (GBLbuf.errmsg != NULL) {
        GBLbuf.stopExecutionFlag = 1;
        return -1;
    }
    
    //rt_UpdateSigLogVars(rtmGetRTWLogInfo(S), rtmGetTPtr(S)); // JAB: This line was original to the R2013b build, but it appears to not be supported by the Mathworks api in R2016a and later. Although unsophisticated, the fix a uncommenting this line appears to be sufficient.
    
    MdlUpdate(0);
    rt_SimUpdateDiscreteTaskSampleHits(rtmGetNumSampleTimes(S),
            rtmGetTimingData(S),
            rtmGetSampleHitPtr(S),
            rtmGetTPtr(S));
    
    if (rtmGetSampleTime(S, 0) == CONTINUOUS_SAMPLE_TIME) {
        rt_UpdateContinuousStates(S);
    }
    
    GBLbuf.isrOverrun--;
    rtExtModeCheckEndTrigger();
    
    rTorqueDemand[0] = SIG_MODEL(Y,Generator_Torque);   
    rBlade1Pitch[0] = SIG_MODEL(Y,Collective_Pitch_Angle);
    rBlade2Pitch[0] = SIG_MODEL(Y,Collective_Pitch_Angle);
    rBlade3Pitch[0] = SIG_MODEL(Y,Collective_Pitch_Angle);

    return 0;
}  /* end calcOutputController */

#else /* MULTITASKING */

# if TID01EQ == 1
#  define FIRST_TID 1
# else
#  define FIRST_TID 0
# endif

DISCON_LOCAL int calcOutputController(float rUserVar1, float rUserVar2, float rUserVar3, float rUserVar4, 
        float rUserVar5,float rUserVar6, float rUserVar7, float rUserVar8, float rUserVar9, float rUserVar10, 
        float rUserVar11,float rUserVar12,float rUserVar13,float rUserVar14,float rUserVar15,float rUserVar16,
        float rUserVar17,float rUserVar18,float rUserVar19,float rUserVar20, 
        float rGeneratorSpeed, float rRatedSpeed, float rBelowRatedPitch, float rForeAftTower, float rSideTower,
        float rRotorAzimuth, float rOP1RootMoment, float rOP2RootMoment,
        float rOP3RootMoment, float rMeasuredPitch, float rMeasuredTorque, 
        float rModeGain,float rHubWindSpeed, float *rTorqueDemand, float *rBlade1Pitch, 
        float *rBlade2Pitch, float *rBlade3Pitch, char *errorMsg) {    int_T  i;
    real_T tnext;
    int_T  *sampleHit = rtmGetSampleHitPtr(S);
    
    SIG_MODEL(U,Generator_Speed) = rGeneratorSpeed;
    SIG_MODEL(U,Rated_Speed) = rRatedSpeed;
    SIG_MODEL(U,Below_Rated_Pitch_Angle) = rBelowRatedPitch;
    SIG_MODEL(U,Fore_Aft_Tower_Accel) = rForeAftTower;
    SIG_MODEL(U,Sidewards_Tower_Accel) = rSideTower;
    SIG_MODEL(U,Measured_Pitch) = rMeasuredPitch;
    SIG_MODEL(U,Measured_Torque) = rMeasuredTorque;
    SIG_MODEL(U,Mode_Gain) = rModeGain;
    SIG_MODEL(U,Hub_Wind_Speed) = rHubWindSpeed;                                                               
    SIG_MODEL(U,Rotor_Azimuth_Angle) = rRotorAzimuth;
    SIG_MODEL(U,Blade1_OP_Root_Moment) = rOP1RootMoment;
    SIG_MODEL(U,Blade2_OP_Root_Moment) = rOP2RootMoment;
    SIG_MODEL(U,Blade3_OP_Root_Moment) = rOP3RootMoment;
    
    SIG_MODEL(U,userVar1) = rUserVar1;
    SIG_MODEL(U,userVar2) = rUserVar2;
    SIG_MODEL(U,userVar3) = rUserVar3;
    SIG_MODEL(U,userVar4) = rUserVar4;
    SIG_MODEL(U,userVar5) = rUserVar5;
    SIG_MODEL(U,userVar6) = rUserVar6;
    SIG_MODEL(U,userVar7) = rUserVar7;
    SIG_MODEL(U,userVar8) = rUserVar8;
    SIG_MODEL(U,userVar9) = rUserVar9;
    SIG_MODEL(U,userVar10) = rUserVar10;
    SIG_MODEL(U,userVar11) = rUserVar11;
    SIG_MODEL(U,userVar12) = rUserVar12;
    SIG_MODEL(U,userVar13) = rUserVar13;
    SIG_MODEL(U,userVar14) = rUserVar14;
    SIG_MODEL(U,userVar15) = rUserVar15;
    SIG_MODEL(U,userVar16) = rUserVar16;
    SIG_MODEL(U,userVar17) = rUserVar17;
    SIG_MODEL(U,userVar18) = rUserVar18;
    SIG_MODEL(U,userVar19) = rUserVar19;
    SIG_MODEL(U,userVar20) = rUserVar20;
    
    /* Check and see if base step time is too fast */
    if (GBLbuf.isrOverrun++) {
        GBLbuf.stopExecutionFlag = 1;
        sprintf(errorMsg, "GLbuf.isrOverrun");
        return -1;
    }
    
    /* Check and see if error status has been set */
    if (rtmGetErrorStatus(S) != NULL) {
        GBLbuf.stopExecutionFlag = 1;
        sprintf(errorMsg, "rtmGetErrorStatus(S) != NULL");
        return -1;
    }
    
    /* enable interrupts here */
    
    /*
     * In a multi-tasking environment, this would be removed from the base rate
     * and called as a "background" task.
     */
    rtExtModeOneStep(rtmGetRTWExtModeInfo(S),
            rtmGetNumSampleTimes(S),
            (boolean_T *)&rtmGetStopRequested(S));
    
    /* Update discrete events */
    tnext = rt_SimUpdateDiscreteEvents(rtmGetNumSampleTimes(S),
            rtmGetTimingData(S),
            rtmGetSampleHitPtr(S),
            rtmGetPerTaskSampleHitsPtr(S));
    rtsiSetSolverStopTime(rtmGetRTWSolverInfo(S), tnext);
    for (i=FIRST_TID+1; i < NUMST; i++) {
        if (sampleHit[i] && GBLbuf.eventFlags[i]++) {
            GBLbuf.isrOverrun--;
            GBLbuf.overrunFlags[i]++;    /* Are we sampling too fast for */
            GBLbuf.stopExecutionFlag=1;  /*   sample time "i"?           */
            return -1;
        }
    }
    
    /* Step the model for the base sample time */
    MdlOutputs(FIRST_TID);
    
    rtExtModeUploadCheckTrigger(rtmGetNumSampleTimes(S));
    rtExtModeUpload(FIRST_TID, rtmGetTaskTime(S, FIRST_TID));
    
    GBLbuf.errmsg = rt_UpdateTXYLogVars(rtmGetRTWLogInfo(S),
            rtmGetTPtr(S));
    if (GBLbuf.errmsg != NULL) {
        GBLbuf.stopExecutionFlag = 1;
        return -1;
    }
    
    //rt_UpdateSigLogVars(rtmGetRTWLogInfo(S), rtmGetTPtr(S)); // JAB: This line was original to the R2013b build, but it appears to not be supported by the Mathworks api in R2016a and later. Although unsophisticated, the fix a uncommenting this line appears to be sufficient.
    
    MdlUpdate(FIRST_TID);
    
    if (rtmGetSampleTime(S, 0) == CONTINUOUS_SAMPLE_TIME) {
        rt_UpdateContinuousStates(S);
    }
    else {
        rt_SimUpdateDiscreteTaskTime(rtmGetTPtr(S),
                rtmGetTimingData(S), 0);
    }
    
    #if FIRST_TID == 1
            rt_SimUpdateDiscreteTaskTime(rtmGetTPtr(S),
            rtmGetTimingData(S), 1);
    #endif    
            
    /* mModel step complete for base sample time, now it is okay to 
       re-interrupt this ISR. */       
    GBLbuf.isrOverrun--;
    
    /* Step the model for any other sample times */
    for (i=FIRST_TID+1; i<NUMST; i++) {
        /* If task "i" is running, don't run any lower priority task */
        if (GBLbuf.overrunFlags[i]) return -1;
        
        if (GBLbuf.eventFlags[i]) {
            GBLbuf.overrunFlags[i]++;
            
            MdlOutputs(i);
            
            rtExtModeUpload(i, rtmGetTaskTime(S, i));
            
            MdlUpdate(i);
            
            rt_SimUpdateDiscreteTaskTime(rtmGetTPtr(S),
                    rtmGetTimingData(S), i);
            
            /* Indicate task complete for sample time "i" */
            GBLbuf.overrunFlags[i]--;
            GBLbuf.eventFlags[i]--;
        }
    }
    
    rtExtModeCheckEndTrigger();
    
	rTorqueDemand[0] = SIG_MODEL(Y,Generator_Torque);
    rBlade1Pitch[0] = SIG_MODEL(Y,Collective_Pitch_Angle);
    rBlade2Pitch[0] = SIG_MODEL(Y,Collective_Pitch_Angle);
    rBlade3Pitch[0] = SIG_MODEL(Y,Collective_Pitch_Angle);
    
    return 0;
}  /* end calcOutputController */

#endif /* MULTITASKING */
/* Function: performCleanup ===============================================
 *
 * Abstract:
 *      Execute model on a generic target such as a workstation.
 */
DISCON_LOCAL int performCleanup(char *errorMsg) {
    
	#ifdef UseMMIDataLogging
    rt_CleanUpForStateLogWithMMI(rtmGetRTWLogInfo(S));
    rt_CleanUpForSigLogWithMMI(rtmGetRTWLogInfo(S));
	#endif
    rt_StopDataLogging(MATFILE, rtmGetRTWLogInfo(S));
    
    rtExtModeShutdown(rtmGetNumSampleTimes(S));
    
    if (GBLbuf.errmsg) {
        (void)fprintf(stderr, "%s\n", GBLbuf.errmsg);
        exit(EXIT_FAILURE);
        sprintf(errorMsg, "%s", GBLbuf.errmsg);
        return -1;
    }
    
    if (rtmGetErrorStatus(S) != NULL) {
        (void)fprintf(stderr, "ErrorStatus set: \"%s\"\n", rtmGetErrorStatus(S));
        exit(EXIT_FAILURE);
        sprintf(errorMsg, "ErrorStatus set: \"%s\"", rtmGetErrorStatus(S));
        return -1;
    }
    
    if (GBLbuf.isrOverrun) {
        (void)fprintf(stderr,
                "%s: ISR overrun - base sampling rate is too fast\n",
                QUOTE(MODEL));
        exit(EXIT_FAILURE);
        sprintf(errorMsg,
                "%s: ISR overrun - base sampling rate is too fast",
                QUOTE(MODEL));
        return -1;
    }
    
    sprintf(errorMsg, "** Stopping the controller **");
    MdlTerminate();
    
    return 0;
}  /* end performCleanup */

float* SetParams(float *avrSwap) 
{
	char mystring [200];
	int iStatus;

	iStatus	=	NINT(avrSwap[0]);
	
	if (iStatus == 0)
	{
		pFile = fopen ("BladedDLL.par","r");
		
		if (pFile == NULL) {avrSwap[128]=1;}
		else {
			fgets (mystring , 200 , pFile);
			avrSwap[119]=atof(mystring);	
			fgets (mystring , 200 , pFile);
			avrSwap[120]=atof(mystring);
			fgets (mystring , 200 , pFile);
			avrSwap[121]=atof(mystring);
			fgets (mystring , 200 , pFile);
			avrSwap[122]=atof(mystring);
			fgets (mystring , 200 , pFile);
			avrSwap[123]=atof(mystring);
			fgets (mystring , 200 , pFile);
			avrSwap[124]=atof(mystring);
			fgets (mystring , 200 , pFile);
			avrSwap[125]=atof(mystring);
			fgets (mystring , 200 , pFile);
			avrSwap[126]=atof(mystring);
			fgets (mystring , 200 , pFile);
			avrSwap[127]=atof(mystring);
			fgets (mystring , 200 , pFile);
			avrSwap[128]=atof(mystring);
			fgets (mystring , 200 , pFile);
			avrSwap[129]=atof(mystring);
			fgets (mystring , 200 , pFile);
			avrSwap[130]=atof(mystring);
			fgets (mystring , 200 , pFile);
			avrSwap[131]=atof(mystring);
			fgets (mystring , 200 , pFile);
			avrSwap[132]=atof(mystring);
			fgets (mystring , 200 , pFile);
			avrSwap[133]=atof(mystring);
			fgets (mystring , 200 , pFile);
			avrSwap[134]=atof(mystring);
			fgets (mystring , 200 , pFile);
			avrSwap[135]=atof(mystring);
			fgets (mystring , 200 , pFile);
			avrSwap[136]=atof(mystring);
			fgets (mystring , 200 , pFile);
			avrSwap[137]=atof(mystring);
			fgets (mystring , 200 , pFile);
			avrSwap[138]=atof(mystring);
			fclose (pFile);
			}
	}
	return(avrSwap);
}

float* writeLogFile(float *avrSwap) 
{
	FILE *fptr;
	
	fptr = fopen("AVRSwap.log","a");
	
	if (pFile == NULL) {
        avrSwap[128] = 1;
    } else {
        fprintf(fptr, "%.4f\t%.4f\t%.4f\t%.4f\t%.4f\t%.4f\t%.4f\t%.4f\t%.4f\t%.4f\t%.4f\t%.4f\t%.4f\t%.4f\t%.4f\t%.4f\t%.4f\t%.4f\t%.4f\t%.4f\t\n",
                SIG_MODEL(Y, Log1), SIG_MODEL(Y, Log2), SIG_MODEL(Y, Log3), SIG_MODEL(Y, Log4),
                SIG_MODEL(Y, Log5), SIG_MODEL(Y, Log6), SIG_MODEL(Y, Log7), SIG_MODEL(Y, Log8),
                SIG_MODEL(Y, Log9), SIG_MODEL(Y, Log10), SIG_MODEL(Y, Log11), SIG_MODEL(Y, Log12),
                SIG_MODEL(Y, Log13), SIG_MODEL(Y, Log14), SIG_MODEL(Y, Log15), SIG_MODEL(Y, Log16),
                SIG_MODEL(Y, Log17), SIG_MODEL(Y, Log18), SIG_MODEL(Y, Log19), SIG_MODEL(Y, Log20));
        fclose(fptr);
    }

    return avrSwap;
}

/*===================*
 * Visible functions *
 *===================*/

/* Function: DISCON =======================================================
 *
 * Abstract:
 *      Execute model for GH BLADED target.
 */
/*==================*
 * Shared functions *
 *==================*/

void DISCON_API CDECL DISCON(float *avrSwap, int *aviFail,
          char *accInfile, char *avcOutname, char *avcMsg) {
    int iStatus;
    char errorMsg[257]; // inFile[257], outName[1025];
    float rTime, rSample, rGeneratorSpeed, rRatedSpeed, rBelowRatedPitch, 
            rRotorAzimuth, rOP1RootMoment, rOP2RootMoment, rOP3RootMoment,
            rForeAftTower, rSideTower, rMeasuredPitch, rMeasuredTorque, 
            rModeGain,rHubWindSpeed;
    float rUserVar1, rUserVar2, rUserVar3, rUserVar4, rUserVar5, rUserVar6, rUserVar7, rUserVar8, rUserVar9, rUserVar10,     
            rUserVar11,rUserVar12,rUserVar13,rUserVar14,rUserVar15,rUserVar16,rUserVar17,rUserVar18,rUserVar19,rUserVar20;
    static float rTorqueDemand, rPitchDemand, rBlade1Pitch, rBlade2Pitch, 
            rBlade3Pitch;
    
    /* Take local copies of strings */
    //memcpy(inFile, accInfile, NINT(avrSwap[49]));
    //inFile[NINT(avrSwap[49])+1] = '\0';
    //memcpy(outName, avcOutname, NINT(avrSwap[50]));
    //outName[NINT(avrSwap[50])+1] = '\0';
    
    /* Set message to blank */
    memset(errorMsg, ' ', 257);
    
    /* Set constants */
    SetParams(avrSwap);
    
    /* Load variables from Bladed (See Appendix A) */
    iStatus          = NINT(avrSwap[0]);
    rTime            = avrSwap[1];
    rSample          = avrSwap[2];
    rMeasuredPitch   = avrSwap[3];
    rBelowRatedPitch = avrSwap[4];
    rModeGain        = avrSwap[15];
    rRatedSpeed      = avrSwap[18];
    rGeneratorSpeed  = avrSwap[19];
    rMeasuredTorque  = avrSwap[22];
    rHubWindSpeed    = avrSwap[26];
    rOP1RootMoment   = avrSwap[29]; 
    rOP2RootMoment   = avrSwap[30];
    rOP3RootMoment   = avrSwap[31];
    rForeAftTower    = avrSwap[52];
    rSideTower       = avrSwap[53];
    rRotorAzimuth    = avrSwap[59];
    
    rUserVar1		= avrSwap[119];
    rUserVar2		= avrSwap[120];
    rUserVar3		= avrSwap[121];
    rUserVar4		= avrSwap[122];
    rUserVar5		= avrSwap[123];
    rUserVar6		= avrSwap[124];
    rUserVar7		= avrSwap[125];
    rUserVar8		= avrSwap[126];
    rUserVar9		= avrSwap[127];
    rUserVar10		= avrSwap[128];
    rUserVar11		= avrSwap[129];
    rUserVar12		= avrSwap[130];
    rUserVar13		= avrSwap[131];
    rUserVar14		= avrSwap[132];
    rUserVar15		= avrSwap[133];
    rUserVar16		= avrSwap[134];
    rUserVar17		= avrSwap[135];
    rUserVar18		= avrSwap[136];
    rUserVar19		= avrSwap[137];
    rUserVar20		= avrSwap[138];
    
    /* determine iStatus */
    aviFail[0] = 0;
    if (iStatus == 0) {
        /* Initialize Controller */
        aviFail[0] = initiateController(errorMsg);
        rPitchDemand = rMeasuredPitch;     
        rTorqueDemand = rMeasuredTorque;
    }
    else if (iStatus >= 0) {
        /* Main calculation */
        aviFail[0] = calcOutputController(rUserVar1, rUserVar2, rUserVar3, rUserVar4, rUserVar5, rUserVar6, rUserVar7, rUserVar8, rUserVar9, rUserVar10, rUserVar11, rUserVar12, rUserVar13, rUserVar14, rUserVar15, rUserVar16, rUserVar17, rUserVar18, rUserVar19, rUserVar20,
                        rGeneratorSpeed, rRatedSpeed,
                        rBelowRatedPitch, rForeAftTower, rSideTower,
                        rRotorAzimuth, rOP1RootMoment, rOP2RootMoment,
                        rOP3RootMoment, rMeasuredPitch, rMeasuredTorque, 
                        rModeGain,rHubWindSpeed, &rTorqueDemand, &rBlade1Pitch, 
                        &rBlade2Pitch, &rBlade3Pitch, errorMsg);
						//&rBlade2Pitch, &rBlade3Pitch, &errorMsg);
    }
    else if (iStatus == -1) {
        /* Main calculation */
        aviFail[0] = calcOutputController(rUserVar1, rUserVar2, rUserVar3, rUserVar4, rUserVar5, rUserVar6, rUserVar7, rUserVar8, rUserVar9, rUserVar10, rUserVar11, rUserVar12, rUserVar13, rUserVar14, rUserVar15, rUserVar16, rUserVar17, rUserVar18, rUserVar19, rUserVar20,
                        rGeneratorSpeed, rRatedSpeed,
                        rBelowRatedPitch, rForeAftTower, rSideTower,
                        rRotorAzimuth, rOP1RootMoment, rOP2RootMoment,
                        rOP3RootMoment, rMeasuredPitch, rMeasuredTorque, 
                        rModeGain,rHubWindSpeed, &rTorqueDemand, &rBlade1Pitch, 
                        &rBlade2Pitch, &rBlade3Pitch, errorMsg);
						//&rBlade2Pitch, &rBlade3Pitch, &errorMsg);
        
        /* Perform Cleanup */
        aviFail[0] = performCleanup(errorMsg);
    }
    else {
        aviFail[0] = -1;
        sprintf(errorMsg, "iStatus is not recognized: %d", iStatus);
    }
    
    /* Store variables too Bladed (See Appendix A) */
    avrSwap[27] = 1; /* Individual Pitch control */
    avrSwap[34] = 1; /* Generator contactor status */
    avrSwap[35] = 0; /* Shaft brake status: 0=off */
    avrSwap[40] = 0; /* Demanded yaw actuator torque */
    avrSwap[41] = rBlade1Pitch;  /* Blade 1 pitch angle demand */
    avrSwap[42] = rBlade2Pitch;  /* Blade 2 pitch angle demand */
    avrSwap[43] = rBlade3Pitch;  /* Blade 3 pitch angle demand */
    avrSwap[46] = rTorqueDemand; /* Torque angle demand */
    avrSwap[47] = 0; /* Demanded nacelle yaw rate */
    avrSwap[54] = 0; /* Pitch override */
    avrSwap[55] = 0; /* Torque override */
    avrSwap[64] = 0; /* Number of variables returned for logging */
    avrSwap[71] = 0; /* Generator start-up resistance */
    avrSwap[78] = 1; /* Request for loads: 0=none */
    avrSwap[79] = 0; /* Variable slip current status */
    avrSwap[80] = 0; /* Variable slip current demand */
    
    /* Return message */
    memcpy(avcMsg, errorMsg, MIN(256, NINT(avrSwap[48])));
	
	/* Write Log variables */
	writeLogFile(avrSwap);
    
    return;
} /* end DISCON */

/* EOF: discon_main.c */
