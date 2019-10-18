/*

	This file contains a definition of the data-structure
	that should contains all variables that will describe the AUV/ASV
	at any given time. This involves:
	1) Inputs from 		NavSensors,
	2) Outputs from 	PID controllers,
	3) Controller Configs	PID controllers/Guidance Controllers

	What is the advantage of having this?
	a) More difficult to have a mix-up in variable values defined globally.
	b) Makes it easier to pass values via shared memory. (No reassignment).
	c) Makes it easier to pass similar values to GUI communication program
	   if same data-structure is used there as well.
*/ 	


#ifndef _ASV_DATA_H_
	#define _ASV_DATA_H_
#define SONTEK_DVL	3

#define ASV_ON		1
#define ASV_OFF		0

#define CAN_OK		1
#define CAN_NOTRAFF	0

#define GPS_FAIL	0
#define GPS_FIX_NO	1
#define GPS_FIX_YES	2

#define AHRS_FAIL	0
#define AHRS_CALLIB	1
#define AHRS_GOOD	2

#define NO_MISS		0
#define MISS_LOADED	1
#define MISS_EXEC	2
#define MISS_STOP	3
#define MISS_PAUSE	4
#define MISS_COMPLETE	5

#define TIMER_MODE	1
#define WAYPT_MODE	2
#define LINE_FOLLOW     3          // added on 27march 2007

#define DVL_FAIL	0
#define DVL_GOOD	1

#define PPTR_FAIL	0
#define PPTR_GOOD	1
#define PPTR_NODEFAIL	2
#define PPTR_INVALID	3

#define SCIENCE_FAIL	0
#define SCIENCE_GOOD	1

#define GUID_OFF	0
#define GUID_WAYPT	1
#define GUID_TRACKING	2

#define WAYPT_NOTACHV	0
#define WAYPT_ACHV	1

#define USE_GPS		0
#define USE_DVL		1

#define CAN_FAIL	0
#define CAN_GOOD	1
#define CAN_BUS_OFF	2

// PID  Types definitions
#define ONLY_P		0	// to be used for PID types
#define ONLY_PI		1
#define ONLY_PD		2
#define PID		3
#define RATE_OFF	0	// to define PD/PID rate structure.
#define RATE_ON		1	
#define CONT_OFF	0
#define CONT_OPEN	1
#define CONT_CLOSE	2
#define CONT_OTHER	3	// Fin/Rudder setup mode...

#define ACT_FIN		 0
#define ACT_RUD 	 1

#define THR_NODEFAIL	0
#define THR_NODEGOOD	1

#define FIN_NODEFAIL	0
#define FIN_NODEGOOD	1

#define RUD_NODEFAIL	0
#define FIN_NODEGOOD	1

#define LogAHRS		0x0001
#define LogDVL          0x0002
#define LogGPSLatLong   0x0004
#define LogActuator     0x0008
#define LogBattery      0x0010
#define LogPressure     0x0020
#define LogHeadingPID   0x0040
#define LogDepthPID     0x0080
#define LogPitchPID     0x0100
#define LogNavVel       0x0200
#define LogNavDrec      0x0400
#define LogNavGps 	0x0800
#define LogGuidance     0x1000
#define LogRef          0x2000

// ----------------------------------------------------------------------------
//  Structures used to communicate between PC and MZ-104. 
//  Reason for using long is to take away the problem of stuffing of integers
//  However, there may still be a little-big endian problems from Windows/Linux
// ----------------------------------------------------------------------------
typedef struct
{
	float X,Y,Z,X2,Y2;  // added X2,Y2 on 27 march07
}Position;

typedef struct
{
	long secs, millisecs;
}TimeOut;

typedef struct
{
	float Thrust, Heading, Depth, Pitch, Left_Fin, Right_Fin, Top_Rud, Down_Rud;
}Parameters;

typedef struct
{
	float Kp_Thrust, Ki_Thrust, Kd_Thrust;
	float Kp_Heading, Ki_Heading, Kd_Heading;
	float Kp_Pitch, Ki_Pitch, Kd_Pitch;
	float Kp_Depth, Ki_Depth, Kd_Depth;
	float Kq_Depth, Ktheta_Depth, Kz_Depth, Ki_z_Depth, Ka_Depth;
        float Ref_Rate, Stern_Sat, Cut_Off_Freq;
        int Rate_Limit,Lqr_Pid,Low_Pass_Filter;        //Lqr = 1 and Pid = 0;

	float K_nav;		//Navigation filter gain.	
	int Comp_Filter;	//enable or disable complimentary filter
	
	int Use_Water_Bottom_Vel; // Enable Water velocity or Bottom Velocity.
	int Use_GPS_DVL; //Perform Navigation based on GPS or DVL is Selected.

	float Kr_Heading, Kyaw_Heading, Ki_yaw_Heading;
        float Rud_Sat;
        int  Fin_Out_Check;
      

	int OpenClose_Thrust, OpenClose_Heading, OpenClose_Pitch,
		 OpenClose_Depth, OpenClose_Lqr_Depth;

	int pidType_Thrust,pidType_Heading,pidType_Pitch,pidType_Depth;
	int RFback_Thrust, RFback_Heading, RFback_Pitch, RFback_Depth;
	float AcceptRad, VelGain;	// ( Guidance Parameters ).
	int GuidanceOn;
	float Max_Depth;
	unsigned int Timeout;
	int Set_Time_Thrust, Set_Band_Thrust, Set_Time_Heading, Set_Band_Heading;
	int Set_Time_Pitch, Set_Band_Pitch, Set_Time_Depth, Set_Band_Depth;
}ControllerSettings;

typedef struct
{
	ControllerSettings PrevSettings;
}ControllerSettings2;
// Contains definitions for transferring 

typedef struct
{	int ASV_OnOff, CAN_status, Miss_status, LineNum,GuidanceStatus;
	int WayPtAchv, NAV_status;
	unsigned long data_len, log_data;
	unsigned short int Log_Format,new_file;
	float T1,T2;
	char miss_filename[100];
//        float temp_SHTXX,hum_SHTXX,dew_SHTXX;         // added on 27dec07
}ASVData;

typedef struct
{
	float elect_volt, act_volt, thr_volt;
	float elect_count, act_count, thr_count;
	float elect_amp, act_amp, thr_amp;
	float temp;
	long int   BAT_MON_Status;
}BAT_MONData;

typedef struct
{
	float YawAngle, Pitch, Roll;
	float YawRate, PitchRate, RollRate;
	float Xaccel, Yaccel, Zaccel;
	float Temp;
	long int   AHRS_Status;
}AHRSData;

typedef struct
{
	float DesHeading;	// Heading ref
	float DesPitch;		// Pitch   ref
	float Left_Fin;		// open loop left fin
	float Right_Fin;	// To set different fin angles in open loop.
        float Top_Rud;		//open loop top rudder
	float Down_Rud;		// to set different rudder angles in open loop
	float DesDepth;		// Depth   ref
	float DesThrust;	// Thrust  ref
        float Delta_dis;        // added on 16 april 2007
	Position DesPos;	// X,Y,Z   ref
}PIDDataIn;

typedef struct
{
	float FinCmd;		// Fin PID output
	float ThrCmd;		// Thr PID output
	float RudCmd;		// Rud PID output
}PIDDataOut;

typedef struct
{
	PIDDataIn  PID_inputs;
	PIDDataOut PID_outputs;
}PIDData;

typedef struct
{
	int lat_deg, lon_deg;
	float lon_min, lat_min;
	float vel; float course;
	char num_of_sat; char data_quality; char lat_dir; char lon_dir;
	long int GPS_status;
}GPSData;
/*
typedef struct
{
	float bottomvel[SONTEK_DVL];
	float bottomrange[SONTEK_DVL];
	float watervel[SONTEK_DVL];
	float amplitude[SONTEK_DVL];
	float temp, power;
	float vel,depth;
	int BottomVelStatus,WaterVelStatus;
	long int   DVL_status;

}DVLData;

typedef struct
{
	float pressure,surface;
	float temp;
	float depth;
	long int   PPTR_Status;
	unsigned char   Leak_Status;
}PPTRData;*/

typedef struct
{
	AHRSData ahrs_data;
	GPSData	 gps_data;
	//DVLData	 dvl_data;
	//PPTRData pptr_data;
}NavSensorData;


typedef struct
{
	char Chlorophyll_Check, Oxygen_Check, CTD_Check;
	float ChloroSample, OxygenSample, CTDSample;
	float ChloroAvg, OxygenAvg, CTDAvg;
	float Chlorophyll;
	float Oxygen;
	float Conductivity;
	float Temperature;
	float Depth;
	float Turbidity;
	int Log_Oxygen,Log_Chloro,Log_CTD;
	long int SCIENCE_Status;
}ScienceData;

typedef struct
{
	int Chlorophyll_Check, Oxygen_Check, CTD_Check;
	float SampRate, AvgRate;
	int Log_Oxygen,Log_Chloro,Log_CTD;
}ScienceParameters;

typedef struct
{
	float X,Y,Z;
	double EST_POS_X,EST_POS_Y;
	double UTME, UTMN;
	double UTME_ref, UTMN_ref; 	// Origin in X,Y co-ods
	float Dead_X, Dead_Y, Dead_Z;
	float Kalman_X, Kalman_Y, Kalman_Z;
}NAV_params;


typedef struct
{
	ASVData	 	ASV_toGUI;		// ASV update
	AHRSData 	AHRS_toGUI;		// AHRS update
	GPSData	 	GPS_toGUI;		// GPS  update
	//DVLData	 	DVL_toGUI;		// DVL  update 
	//PPTRData 	PPTR_toGUI;		// PPTR update (via CAN)
	BAT_MONData 	BAT_toGUI;		// Battery Mon update
	PIDData 	PID_toGUI;		// PID update
	NAV_params 	NAV_toGUI;	// Navigation update
	ScienceData 	SCIENCE_toGUI;	// Science update
}ASV_status;

typedef struct
{
	char UpdatePIDinputs;
	char UpdateUTMrefs;
	char UpdateControllers;
	char Wait_Depth;
	char Wait_Pitch;
	char Wait_Heading;
	char Wait_Waypt;
	char GPS_pause;		// A flag for testing Dead-reckoning.
	char Callib_onoff;	// A flag to callibrate the AHRS.
	char Update_logging;
	char Set_Zero_Rud_Fin;
	char Get_Sci_Stat;
        char Erase_Sci_Data;
        char Setup_Clock;
        char Set_Samp_Rate;
        char Num_Samp_Rate;
        char Start_Log_Sci;
        char Stop_Log_Sci;
	char Write_Science;
//	char KeepSTN;		//added on 9/11/06
        char DVL_BottomVelInv; // added on 7 may07
        char DVL_WaterVelInv;  // added on 7 may07

 //     char CoreServerAlive;  // added on 29 jan08
}Shared_flags;

struct pid 
{
	int on,type,rf;
	float acc_err, last_err;
	float kp, ki, kd, rate, meas, ref, out, max, min;
};

struct lqr
{
	int first_diff;
        float kq, ktheta, kz, ki_z, ka;
        float kr, kyaw, ki_yaw;		//yaw control LQR
        float last_err, last_rate, last_yaw;
        float meas, rate, ref, z, yaw, out, ref_rl, diff, cut_off_freq;
        float stern_sat, delta_s, rate_limit, low_pass_filter, ref_rate, antiwindup;
        float rud_sat,delta_r;
};

#endif
