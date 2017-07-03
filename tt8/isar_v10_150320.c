/**
A new software code for isar. 
Much simplified for use with a companion laptop/pc.
100128--I am in a big pickle. ISAR01 TT8 has failed and I have to 
deploy in just a few days.  But I do not have the source code for the 
version of isaros that is in the isar now. I have decided to take this
emergency opportunity to rewrite the isar code to a much much simpler
version with EEPROM.
**/
#define		PROGRAMNAME	"isar"
#define		VERSION		"10"
#define		EDITDATE	"150320" 
#define  EEPROM_ID		5
//v02 100304 -- include isar sn and use different switch polarities. See ReadSwitch();
//v03 100305 -- include an option to ignore shutter operation
//v04 100322 -- under dev
//...
//  Fixed Angel[] in Action menu
/** To do:
 1. Add nbb, Nsky, Nocean to eeprom menu
 2. Adjust for shutter switches for I1 and i4
 3. 
**/
//v07 101207 rmr -- Trying to make it so it can be terminated if there is a hardware failure.
//		startup option to go to test mode.
//		encoder check, goes to test option
//v08 110704 -- allow option to quit.
//    110730 -- change quit option to an option to go to test
//v09 110731 -- pni txd problem. Workaround-let tcm free run.
//v10 150320 -- Add drum threshold to eeprom, default = 0.5

// ============ INCLUDES
#include <time.h>
#include <tt8.h>
#include <tat332.h>
#include <sim332.h>
#include <qsm332.h>
#include <tpu332.h>
#include <tpudef.h>
#include <dio332.h>
#include <tt8pic.h>
#include <tt8lib.h>
#include <stdio.h>
#include <stdlib.h>
#include <userio.h>
#include <math.h>
#include <string.h>
#include <PicoDCF8.h>

// ======= DEFINES
#define		OK	1
#define		NOTOK	0
#define		ON	1
#define		OFF	0
#define		TOGGLE	2
#define 	NO 0
#define 	YES 1
#define		MISSING		-999
#define		NEG -1
#define 	POS 1

// POWER
#define		SPARE	0
#define		IRT		1
#define		BB1		2
#define		BB2		3

// MOTOR DEFINES  motormotor
#define		FWD 1
#define 	REV -1
#define 	STOP 0
#define		SCAN 1
#define 	DOOR 2

// SCAN DRUM DEFINES
//v10 #define		SCAN_TOLERANCE 0.2

// SHUTTER OPERATION
#define			DOOR_TIMEOUT 30
#define			TRANSITION_SECS		300

// TT8 DEFINES  tt8tt8
#define         IRTRX   14    // irtirt
#define         IRTTX   13

// 485 DEFINES   485485
#define         RX485   7              //TPU2 for 485 (RX)
#define         TX485   8               //TPU5 for 485 (TX)

// PNI DEFINES  pnipni
#define RXPNI 10        //PNI RX
#define TXPNI 9        //PNI TX
#define	PNIWAIT 500 // = timeout in msec  // v9 increase waiting time


// GPS DEFINES   gpsgps
#define		GPSRX	12
#define		GPSTX	11
#define		END		13

// MODES OF OPERATIONtt
#define TEST -1
#define RUN	1

// EEPROM VARIABLES
// Scann angles: abb1, abb2, asky, asea, a1, a2
#define MEMSTART 100
#define default_abb1	280
#define default_abb2	325
#define default_asky	55
#define default_aocean	125
#define default_a1	0
#define default_a2	0
#define default_org 90
#define default_testmode 0
#define default_drumref 0
#define default_Nbb 30
#define default_Nsky 10
#define default_Nocean 40
#define default_rain_threshold .090
#define default_isar_sn 4
#define default_shutter 1
#define default_SCAN_TOLERANCE 0.5

// WATCHDOG TIMER
#define	WDOGSECS	10

// SHUTTER OPERATION
#define 	DRY			-1
#define		TRANSITION	0
#define		RAIN		1

// ======= VARIABLES
struct eeprom {
	ushort	id;
	float abb1, abb2, aocean, asky, a1, a2, testmode, drumref;
	int Nbb, Nsky, Nocean, IsarSN, ShutterFlag;
	float rain_threshold, SCAN_TOLERANCE; //v10
};

struct eeprom *ee;

int	SwPwrFlag[6];
int PrintFlag;
int Missing;

// POWER FLAGS
int		bb1flag, bb2flag;
// MOTOR motormotor
int		ScanMotorFlag;
int		DoorState;  		// open or closed
int		DoorMotorFlag;		// FWD, REV, STOP

// PNI & GPS
float	pitch, roll, compass, sog, cog, var, temperature;
double 	lat, lon;

char  reply[100], str[100];

struct tm   *rtc, *t;

// ======= MAIN LOOP VARIABLES
int		Nsamps[4];
float	Angle[4];

// ISAR STANDARD DATA RECORD STRUCTURE
// This is used to hold an engineering data record.  Typically
// these data will all be archived in real time to disk for
// off line re-processing.  This is the default Level0 engineering
// product.
struct idata{
	int	     hallsw[2];	  		// Shutter switch states (0|1)
	double	 scan_pos;	  		// Scan drum position (Deg)
    float    pni[4];            // pitch, roll, azimuth (Deg) temperature
	double   position[2];   	// lat, lon (Deg)
	float    gps[3];	 		// sog, cog, var
    float    kt15_target_temp;  // Digital temperature from the KT15
    float    kt15_ambient_temp; // Temperature of the KT15 unit
	double   ad18[8];			// 0:bb1 p1 thermistor
								// 1:bb1 p2 thermistor
								// 2:bb1 p3 thermistor
								// 3:bb2 p1 thermistor
								// 4:bb2 p2 thermistor
								// 5:bb2 p3 thermistor
								// 6:KT15 analog output
								// 7:Rain guage analog output
    unsigned int  ad12[8];      // 0:5 Volt reference voltage for BB thermistors
                                // 1:Channel A Thermistor
                                // 2:Channel B Thermistor
                                // 3:Channel C Thermistor
                                // 4:Channel D Thermistor
                                // 5:Window Thermistor                                                                                                            // 5:spare
                                // 6:TT8 tthermistor
                                // 7:Input power
};
struct idata 		*drec;
char				rawrecord[200];



// ========== FUNCTION PROTOTYPES ============================================
//fcnfcn
int Startup(void);
// WATCHDOG TIMER
void    PingWatchDog(void);
// ADC PROTOS
void 		Read_Analog(void);
unsigned 	GetADC(unsigned);
float		Read_BattVoltage(void);
void		GetTT8Temp(double*, double*, double*); // temp, resistance, mvolts
void		GetOtherTemps(double*, double*);
double		Steinhart44006(double);
void		Cal44006(double, double *);  // fills the 3xN cal array
int			GetBBTemp(double, double, double *v, double *temp, double *mean);	// fills BTemp[6] and BBmV[6]
// 4017
void 		SampleADCExt(void);
int 		RS_485(char*, char*, unsigned);  // general 485 function
int			Read4017All(double *); // read all 8 chans into float[0-7]
int  		Read4017(int , double *);  // read one channel of the 4017
// GPS PROTOS   gpsgps
int     	ReadGPS(char *);
int 		ParseGPS(char *, char *); // gps str in, gps data str out
int 		ReadGpsStr(char *str, struct tm *t, double *lat, double *lon, float *sog, float *cog, float *var);
// MATH FUNCTIONS
double 		d_round(double x);
void		MeanStdev(double*, double*, int, double);
int 		sign (float input);
float 		DiffAngle(float , float );
float		CheckAngle(float);
// PNI -- 	PRECISION NAVIGATION PITCH/ROLL/AZIMUTH
void 		ReadPNI(float*, float*, float*, float*);  // Pitch Roll, Azimuth and Temperature
unsigned 	PniSendStr( char*); // send a string to the PNI tilt sensor
unsigned 	PniGetStr( char*);  // receive a string from PNI
void		PniCommand( char*, char *);  // send a command to the PNI
int 		SerPutStr(char*);
// ENCODER PROTOS
void 		DisableEncoder(void);
void 		EnableEncoder(void);
float 		readEncoder (float);
float 		PointScanDrum(float, float);
// EEPROM PROTOS
void StoreUee(struct eeprom *);
void ReadUee(struct eeprom *);
void PrintUee(struct eeprom *);
// MOTOR PROTOS   motormotor
int 	ScanMotor(int);
int		DoorMotor(int);
// MENU PROTOS
void	SetMode(int);
void	Action(char *);
// POWER PROTOS   powerpower
void 	SwPower (int, int);
// SHUTTER SWITCH
void 	ReadSwitch(int*, int*);
int		CloseDoor(void);
int		OpenDoor(void);
// KT15 PROTOS
float	GetKt15TargetTemp(void);
float 	GetKt15AmbientTemp(void);
int 	SendIrtCommand(char *, char *);
// TIME DATE
time_t	GetTime(char *);
time_t ShowTime(void);
// MAIN LOOP
void 	MakeRecord(char *, struct idata *);
void 	GetData(struct idata *);

time_t  wdogtime;  // time for the next watchdog reset
time_t 	tdelay;

int RunMode; 		// TEST RUN 

// NOTE -- MAIN PROGRAM


main()
{
	char		in_buffer[128];  // pointer only
	char		str[65];		// general use
	int			ipoint, isamp, rain_mode, ipointmax; 
	long 		cfsize, cffree;
	char 		byte;
	char  		chr1;
	float		fdum;
	double 		vorg;
	int		 	dt, i1, i2;
	time_t		tdoor;

	Missing = MISSING;
	
	/**************
	INITIALIZE TT8
	***************/
	InitTT8(NO_WATCHDOG, TT8_TPU);

	Startup();
	rtc = (struct tm*)calloc(1, sizeof(struct tm));
	t = (struct tm*)calloc(1, sizeof(struct tm));
	t->tm_year = 2010-1900; t->tm_mon=0; t->tm_mday=1;
	t->tm_hour = t->tm_min = t-> tm_sec = 0;
	printf("Initialize Time: %04d,%02d,%02d,%02d,%02d,%02d\n",
		t->tm_year, t->tm_mon,t->tm_mday,
		t->tm_hour,t->tm_min,t->tm_sec);
	SetTimeTM(t,NULL);
	ShowTime();
	
	// TURN ON THE IRT AND THE HEATED BB
	SwPower(IRT, ON);  // start up with IRT on
	SwPower(BB1, OFF);
	SwPower(BB2, ON);

	// OTHER INITIALIZATIONS
	ee = (struct eeprom *)calloc(1, sizeof(struct eeprom));
	drec = (struct idata *)calloc(1,sizeof(struct idata));
	
	/*********************
	// INITIALIZE EEPROM STORED VARIABLES
	**********************/
	printf("Check EEPROM\n");
	ReadUee(ee);			// read eeprom structure to pointer ee
	if(ee->id == EEPROM_ID )
	{
		printf("EEPROM ID CHECKS\n");
	}
	else
	{
		printf("Initialize eeprom...\n");
		//SetUeeDefault();
		ee->id = EEPROM_ID;
		ee->abb1 = default_abb1;
		ee->abb2 = default_abb2;
		ee->asky = default_asky;
		ee->aocean = default_aocean;
		ee->a1 = default_a1;
		ee->a2 = default_a2;
		ee->testmode = default_testmode;
		ee->drumref = default_drumref;
		ee->Nbb = default_Nbb;
		ee->Nsky = default_Nsky;
		ee->Nocean = default_Nocean;
		ee->rain_threshold = default_rain_threshold;
		ee->IsarSN = default_isar_sn;
		ee->ShutterFlag = default_shutter;
		ee->SCAN_TOLERANCE = default_SCAN_TOLERANCE;
		StoreUee(ee);
		ReadUee(ee);
		PrintUee(ee);
	}
	
	// OPPORTUNITY TO BAIL
	tdelay = time(NULL) + 5;
	printf("Enter 'T' or 't' to go directly to test mode.\n");
	while( difftime(tdelay, time(NULL)) > 0 ) {
		if ( SerByteAvail() ) {
			byte = SerGetByte();
			if ( byte == 't' || byte == 'T' ) {
				// ENTER test MODE
				RunMode = TEST;
				/***************************
				Enter test mode
				Exit by changing mode or by time out
				***************************/
				while( RunMode == TEST ) {
					// CHECK WATCHDOG AND UPDATE
					if( difftime(wdogtime, time(NULL)) <= 0 )
					{
						PingWatchDog();
						wdogtime = time(NULL) + WDOGSECS;
						if(ee->testmode)printf("*");
					}
					printf("\n> ");
					gets(in_buffer);
					if( in_buffer[0] == 'G' || in_buffer[0] == 'g' ) {
						puts("RETURN TO SAMPLING");
						break;
					}
					else Action(in_buffer);
				}
			}
		}
	}
	
	/*************
	SIGN ON PROGRAM
	*****************/
	printf("\n Program Start. Name: %s, Ver: %s, Edit Date: %s\n", PROGRAMNAME, VERSION, EDITDATE);
	
	if (ee->testmode == 1) 
		printf("Ready to go -- TEST MODE\n");
	else
		printf("Ready to go -- NORMAL OPERATION\n");
	
	/***************************
	Start up -- check the org and position the door accordingly
	*****************************/
	Read4017(7,&vorg); Read4017(7,&vorg); // first read is sometimes bad
	if(vorg < ee->rain_threshold) {
		puts("Dry start--");
		if( ee->ShutterFlag) OpenDoor();
		rain_mode = DRY;
		ipointmax=3;
	} else {
		puts("Wet start--");
		if( ee->ShutterFlag) CloseDoor();
		rain_mode = RAIN;
		ipointmax=1;
	}
	
	/****************
	MAIN LOOP WITH MENU
	******************/
	Angle[0]=ee->abb1; Angle[1]=ee->abb2; Angle[2]=ee->asky; Angle[3]=ee->aocean;
	Nsamps[0]=ee->Nbb; Nsamps[1]=ee->Nbb; Nsamps[2]=ee->Nsky; Nsamps[3]=ee->Nocean;
	
	ipoint = 0;
	while (1) {
		TPUSetPin(6,0);  		//Turn ON BB thermistor ref
		SwPower(IRT, ON);  // start up with IRT on
		SwPower(BB1, OFF);
		SwPower(BB2, ON);
		if(ee->testmode) printf("POINT SCAN DRUM TO %.1f DEG\n",Angle[ipoint]);
		while( fabs(fdum-Angle[ipoint]) > 0.5 ) {
			fdum = PointScanDrum(Angle[ipoint], ee->drumref);
		}
		if(ee->testmode) printf("ipoint=%d, Set angle=%.1f, Actual angle=%.1f\n", ipoint,Angle[ipoint], fdum);
		
		// CHECK ON THE DOOR -- TRY TO CLOSE/OPEN
		ReadSwitch(&i1, &i2);
		if ( rain_mode == DRY && i1 && ee->ShutterFlag) {
			puts("Try to open the shutter");
			OpenDoor();
		}
		else if( rain_mode == RAIN && i2 && ee->ShutterFlag ) {
			puts("Try to close the shutter");
			CloseDoor();
		}
		
		//NOTE -- MAIN SAMPLING LOOP

		//********** SAMPLING LOOP ***********************
		for( isamp=0; isamp<Nsamps[ipoint]; isamp++ ) {
			/**********************
			WATCHDOG TIMER RESET
			***********************/
			if( difftime(wdogtime, time(NULL)) <= 0 )
			{
				PingWatchDog();
				wdogtime = time(NULL) + WDOGSECS;
				if(ee->testmode)printf("*");
			}
			
			/*****************************
			// CHECK FOR KEYBOARD ENTRY
			******************************/
			if( SerByteAvail())
			{
				chr1 = SerGetByte();
				
				// FIRST CHECK FOR SINGLE STROKE Test CHARACTERS
				switch(chr1)
				{
					/**********************
					Test MODE LOOP
					keykey
					**********************/
					case 'T':
					case 't':
						// ENTER test MODE
						RunMode = TEST;
						/***************************
						Enter test mode
						Exit by changing mode or by time out
						***************************/
						while( RunMode == TEST )
						{
							// CHECK WATCHDOG AND UPDATE
							if( difftime(wdogtime, time(NULL)) <= 0 )
							{
								PingWatchDog();
								wdogtime = time(NULL) + WDOGSECS;
								if(ee->testmode)printf("*");
							}
							printf("\n> ");
							gets(in_buffer);
							if( in_buffer[0] == 'G' || in_buffer[0] == 'g' ) {
								puts("RETURN TO SAMPLING");
								if( rain_mode == DRY && ee->ShutterFlag) OpenDoor();
								else if(ee->ShutterFlag) CloseDoor();
								break;
							}
							else Action(in_buffer);
						}
						break;
				}
			}
			GetData(drec);
			MakeRecord(rawrecord, drec);
			printf("%s\n", rawrecord);
			/************************************
			CHECK FOR RAIN
			*************************************/
			Read4017(7,&vorg); Read4017(7,&vorg); // first read is sometimes bad
			// ORG > THRESHOLD ==> RAIN
			if (vorg >= ee->rain_threshold) {
				if( rain_mode == DRY ) {
					fdum = PointScanDrum(325, ee->drumref);
					if(ee->ShutterFlag) CloseDoor();
				}
				rain_mode = RAIN;
			// ORG < THRESHOLD == NO RAIN
			} else {
				// RAIN -> TRANSITION MODE
				if( rain_mode == RAIN) {
					rain_mode = TRANSITION;
					tdoor = GetTime(str) + TRANSITION_SECS;
				// NO RAIN, TRANSITION 
				} else if( rain_mode == TRANSITION){
					dt = (int)(tdoor - GetTime(str));
					printf("Open in %d secs\n", dt);
					if (dt <= 0) {
						if(ee->ShutterFlag) OpenDoor();
						rain_mode = DRY;
					}
				}
			}
			if ( rain_mode == RAIN || rain_mode == TRANSITION ) ipointmax = 1;
			else ipointmax = 3;
			
			//********** END RAIN ROUTINE ******************
		}
		ipoint++; if(ipoint > ipointmax) ipoint=0;
	}
	TPUSetPin(6,1);  		//Turn OFF BB reference v
	exit(0);
}

//************************************************
// UTILITY FUNCTIONS
//************************************************

// NOTE -- UTILITY FUNCTIONS

void	Action(char *cmd)
/*****************************************************************
	Read message and take action.
	Create an output packet in out_buffer and send;
	input:
		in_buffer = pointer to a message string
		in_fromlist = char string, e.g. "121115"
			meaning the message came from 12 in the route 15->11->12
		RMR 991101
******************************************************************/
{
	char chr;
	float fdum, fdum1, fdum2;
	double ddum1, ddum2, ddum3, ddum4, ddum5;
	double v[8], tbb[8], tav[2];
	int		i1,i2, i3;
	ulong	udum;

	
	// TAKE ACTION AND PREPARE AN OUTPUT MESSAGE IN out_message
	switch(*cmd)
	{
	   //	menumenu
		case '?':
		case '/':
			PingWatchDog();
			printf("\n ***  OPERATION MENU ver %s, last edit: %s  ***\n", VERSION, EDITDATE);
			puts("------- EEPROM -----------------------------------");
			puts(" E -- show eeprom             ERfff.f -- rain threshold, volts");
			puts(" ECfff.f -- BB1 point angle   EHfff.f -- BB2 point angle");
			puts(" ESfff.f -- SKY point angle   EOfff.f -- OCEAN point angle");
			puts(" EDfff.f -- Drum zero ref     EBnn    -- Black body sample count");
			puts(" EUnn    -- SKY sample count  ETnn    -- OCEAN (target) count");
			puts(" ENnn    -- ISAR SN           EMnn    -- Shutter motor on=1, off=0");
			puts(" EEff.ff -- DRUM ERROR");
			puts(" a --> 12-bit ADC             A --> 18-bit ADC (4017)");
 			puts(" t --> TT8, window temp       T --> Read BB temps");
			puts(" V --> Read Battery Volts     v --> Read ORG volts");
			puts(" b --> BB1 heater toggle      B --> BB2 heater toggle");
			puts(" K --> KT15 on/off            k --> spare pwr on/off");
			puts(" H --> Read KT15 target and ambient T");
			puts(" i --> !! Scan Motor FWD      I --> !! Scan Motor REV");
			puts(" j --> !! Door Motor open     J --> !! Door Motor close");
			puts(" f --> Open Door              F --> Close Door");
			puts(" p --> Read PNI               P --> PNI command");
			puts(" d --> Read drum position     Dfff.ff --> Point Drum to angle");
			puts(" S --> Read switch states		UyyyyMMddhhmmss --> set clock");
			puts(" C --> complete data record");
			puts(" r --> toggle between test mode and operation");
			puts(" L --> Location, Read GPS");

			puts(" G to continue sampling.      X --> QUIT to TOM8");
			
		break;
		
		
		// OPEN AND CLOSE THE SHUTTER
		case 'f':
			printf("Open door request: ");
			if(ee->ShutterFlag) {
				i1 = OpenDoor();
				if(i1==OK) printf("SUCCESS\n");
				else printf("FAILS\n");
			} else puts("motor function is off");
			break;
		case 'F':
			printf("Close door request: ");
			if(ee->ShutterFlag) {
				i1 = CloseDoor();
				if(i1==OK) printf("SUCCESS\n");
				else printf("FAILS\n");
			} else puts("motor function is off");
			break;
			
		// SET CLOCK
		case 'U':
		case 'u':
			i1 = strlen(cmd);
			if( i1 == 1 ) ShowTime();
			else if( i1 == 15 ) {
				sscanf(cmd,"%*c%4d%2d%2d%2d%2d%2d",
					&udum, &t->tm_mon,&t->tm_mday,
					&t->tm_hour,&t->tm_min,&t->tm_sec);
				t->tm_mon--;  // mons 0-11
				t->tm_year = udum-1900;
				printf("Time: %04d,%02d,%02d,%02d,%02d,%02d\n",
					t->tm_year, t->tm_mon,t->tm_mday,
					t->tm_hour,t->tm_min,t->tm_sec);
				SetTimeTM(t, NULL);
				ShowTime();
			}
			break;
		// 12 BIT ADC READ
		case 'a' :
			Read_Analog();
			break;
		case 'A' :
			printf("Read 18bit Ext ADC\n");
			TPUSetPin(6,0);  		//Turn ON BB Power
			SampleADCExt();
			TPUSetPin(6,1);		//Turn OFF BB Power
			break;


		// ======== BATTERY VOLTAGE
		case 'V':
			fdum = Read_BattVoltage();
			break;
		// ======= ORG VOLTS
		case 'v':
			puts("READ ORG MILLIVOLTS");
			while( !SerByteAvail() ) {
				Read4017(7, &ddum1);
				printf("ORG volts = %.3lf,   rain rate = %.1f mm/hr \n", ddum1, (ddum1-0.07)*(ddum1-0.07)*20);
				DelayMilliSecs(1000);
			}
			break;
		
		// ============== BB TEMPERATURES
		case 'T':
			TPUSetPin(6,0);  		//Turn ON BB Power
			puts("READ BLACK BODY TEMPERATURES");
			while( !SerByteAvail() ) {
				PClear(E,5);
				GetBBTemp(5.0, 10000, v, tbb, tav);
				printf("\nBB1 v: %.4lf %.4lf %.4lf,   BB2 v: %.4lf %.4lf %.4lf\n",
				 v[0], v[1], v[2], v[3], v[4], v[5]);
				printf("BB1 temp: %.3lf %.3lf %.3lf,   BB2 temp: %.3lf %.3lf %.3lf\n",
				 tbb[0], tbb[1], tbb[2], tbb[3], tbb[4], tbb[5]);
				printf("BB1 mean temp=%.3lf,   BB2 mean temp: %.3lf\n",
				 tav[0], tav[1]);
				DelayMilliSecs(1000);
				PSet(E,5);
			}
			TPUSetPin(6,1);		//Turn OFF BB Power
			break;
		// ========== TT8  AND OTHER TEMPS THERMISTOR
		case 't':
			GetTT8Temp(&ddum3, &ddum4, &ddum5); // temp, ohms, millivolts
			GetOtherTemps(&ddum1, &ddum2);
			printf("TT8: Chan 6 = %.1lf mv, Rtherm = %.1lf ohms, Temp = %.1lf C\n",ddum5, ddum4, ddum3);
			printf("Other temps = %.1lf, %.1lf\n", ddum1, ddum2);
			break;
		
		// ======= KT15 DIGITAL DATA
		case 'H':
		case 'h':
		    fdum1=GetKt15TargetTemp();
		    fdum2=GetKt15AmbientTemp();
    		printf("test KT15 target temp=%.1f,  ambient temp=%.1f\n", fdum1,fdum2);
			break;
		
		// ======= POWER SWITCHES
		case 'k' :
			puts("TOGGLE SPARE POWER");
			SwPower(SPARE,TOGGLE);
			break;
		case 'K' :
			puts("TOGGLE IRT POWER");
			SwPower(IRT,TOGGLE);
			break;
		// BB1 heater toggle
		case 'b' :
			puts("TOGGLE BB1 POWER");
			SwPower(BB1,TOGGLE);
			break;
		// BB2 heater toggle
		case 'B' :
			puts("TOGGLE BB2 POWER");
			SwPower(BB2,TOGGLE);
			break;
		// =========== SCAN MOTOR
		// FORWARD
		case 'i' :
			if( ScanMotorFlag == FWD || ScanMotorFlag == REV ) ScanMotor(STOP);
			else ScanMotor(FWD);
			break;
		// REVERSE
		case 'I' :
			if(ScanMotorFlag == FWD || ScanMotorFlag == REV  ) ScanMotor(STOP);
			else ScanMotor(REV);
			break;
		// ========== SHUTTER MOTOR
		// OPEN
		case 'j' :
			DoorMotor(STOP);
			puts("Caution, can damage gear belt. Proceed (y/n)?");
			if( SerGetByte() == 'y' ) DoorMotor(FWD);
			puts("Hit any key to stop.");
			if ( SerByteAvail() ) DoorMotor(STOP);
			break;
		// CLOSE
		case 'J' :
			DoorMotor(STOP);
			puts("Caution, can damage gear belt. Proceed (y/n)?");
			if( SerGetByte() == 'y' ) DoorMotor(REV);
			puts("Hit any key to stop.");
			if ( SerByteAvail() ) DoorMotor(STOP);
			break;
			
		// =========== READ PNI
		case 'p':
			ReadPNI(&pitch, &roll, &compass, &temperature);
			printf("PNI: pitch=%.1f, roll=%.1f, compass=%.1f, temperature=%.1lf\n",
			  pitch, roll, compass, temperature);
			break;
		case 'P':
			SerPutStr("Enter string: ");
			scanf("%s", cmd);
			sprintf(str,"Sending PNI string: %s\n",cmd);
			SerPutStr(str);

			PniCommand(cmd, reply);
			printf("reply=%s\n",reply);
			break;
		
		// ============= READ GPS DATA
		case 'L':
		case 'l':
			if( ReadGPS(reply) == OK )
			{
				printf("GPS string: %s\n", reply);
				ParseGPS(reply, str);
				printf("ParseGPS(): %s\n",str);
				// decode the string
				ReadGpsStr(str, rtc, &lat, &lon, &sog, &cog, &var);
				printf("GPS: %4d-%02d-%02d %02d:%02d:%02d\n",
					rtc->tm_year+1900, rtc->tm_mon+1, rtc->tm_mday, rtc->tm_hour,
					rtc->tm_min, rtc->tm_sec);
				printf("lat=%.6lf, lon=%.6lf, sog=%.2f, cog=%.1f, var=%.1f\n",
					lat, lon, sog, cog, var);
			}
			else
				printf("ReadGPS() fails\n");
			break;

		//================= ENCODER
		// POINT THE ENCODER
		case 'D':
			sscanf(cmd,"%*c%f",&fdum);
			printf("POINT SCAN DRUM TO %.1f DEG\n",fdum);
			fdum = PointScanDrum(fdum, ee->drumref);
			printf("Final position = %.1f deg\n", fdum);
			break;
		// READ DRUM POSITION
		case 'd':
			// Enable the Encoder
			EnableEncoder();
			fdum = readEncoder(ee->drumref);
		    DisableEncoder();
			printf("Drum position = %.1f deg\n", fdum);
			break;
		
		// HALL EFFECT SWITCHES
		case 's' :
		case 'S' :
			printf("Read switches...\n");
			while( !SerByteAvail() ) {
				PingWatchDog();
				ReadSwitch(&i1, &i2);
				printf("Switch 1: %d,  switch 2: %d\n", i1, i2);
				DelayMilliSecs(500);
			}
			break;
		
		// ======== TEST MODE
		case 'r':
			if ( ee->testmode == 0 ) {
				puts("Switch mode to TEST");
				ee->testmode = 1;
			} else {
				puts("Switch mode to RUN");
				ee->testmode = 0;
			}
			break;
		
		
		// =========== COMPLETE DATA STRING
		case 'c':
		case 'C':
			puts("COMPLETE DATA STRING");
			TPUSetPin(6,0);  		//Turn ON BB thermistor ref
			GetData(drec);
			MakeRecord(rawrecord, drec);
			printf("%s\n", rawrecord);
			TPUSetPin(6,1);  		//Turn OFF BB ref
			break;
			
		// =========EEPROM SET
		case 'E':
		case 'e':
			switch( cmd[1] )
			{
				 
				// DRUM ANGLE TOLERANCE
				case 'E':
					sscanf(cmd,"%*c%*c%f",&fdum);
					printf("SCAN_TOLERANCE = %.2f\n",fdum);
					ee->SCAN_TOLERANCE=fdum;
					chr='y';
					break;
				
				
				// BLACK BODY SAMPLE COUNT
				case 'B':
				case 'b':
				 sscanf(cmd, "%*c%*c%d", &i1);
				 printf("SET BB SAMPLE COUNT TO %d\n", i1);
				 ee->Nbb = i1;
				 chr  = 'y';
				 break;

				// SKY SAMPLE COUNT
				case 'u':
				case 'U':
				 sscanf(cmd, "%*c%*c%d", &i1);
				 printf("SET SKY SAMPLE COUNT TO %d\n", i1);
				 ee->Nsky = i1;
				 chr  = 'y';
				 break;

				// OCEAN SAMPLE COUNT
				case 't':
				case 'T':
				 sscanf(cmd, "%*c%*c%d", &i1);
				 printf("SET OCEAN SAMPLE COUNT TO %d\n", i1);
				 ee->Nocean = i1;
				 chr  = 'y';
				 break;

				// SHUTTER MOTOR ON/OFF
				case 'M':
				case 'm':
				 sscanf(cmd, "%*c%*c%d", &i1);
				 printf("SET SHUTTER MOTOR FLAG TO %d\n", i1);
				 ee->ShutterFlag = i1;
				 chr  = 'y';
				 break;
				 
				// ISAR SERIAL NUMBER
				case 'n':
				case 'N':
				 sscanf(cmd, "%*c%*c%d", &i1);
				 printf("SET ISAR SERIAL NUMBER TO %d\n", i1);
				 ee->IsarSN = i1;
				 chr  = 'y';
				 break;
				 
				// COLD BB ANGLE
				case 'c':
				case 'C':
					sscanf(cmd,"%*c%*c%f",&fdum);
					printf("SET COLD BB1 ANGLE = %.1f\n",fdum);
					ee->abb1=Angle[0]=fdum;
					chr='y';
					break;
				// HOT BB ANGLE
				case 'h':
				case 'H':
					sscanf(cmd,"%*c%*c%f",&fdum);
					printf("SET HOT BB2 ANGLE = %.1f\n",fdum);
					ee->abb2=Angle[1]=fdum;
					chr='y';
					break;
				// SKY ANGLE
				case 's':
				case 'S':
					sscanf(cmd,"%*c%*c%f",&fdum);
					printf("SET SKY ANGLE = %.1f\n",fdum);
					ee->asky=Angle[2]=fdum;
					chr='y';
					break;
				// OCEAN ANGLE
				case 'o':
				case 'O':
					sscanf(cmd,"%*c%*c%f",&fdum);
					printf("SET OCEAN ANGLE = %.1f\n",fdum);
					ee->aocean=Angle[3]=fdum;
					chr='y';
					break;
				case 'd':
				case 'D':
					sscanf(cmd,"%*c%*c%f",&fdum);
					printf("SET DRUM REFERENCE = %.1f\n",fdum);
					ee->drumref = fdum;
					chr='y';
					break;
				case 'R':
				case 'r':
					sscanf(cmd,"%*c%*c%f",&fdum);
					printf("SET RAIN THRESHOLD = %.4f\n",fdum);
					ee->rain_threshold = fdum;
					chr='y';
					break;
				default:
					printf("Error in eeprom command: %s\n",cmd);
					chr='n';
					PrintUee(ee);
			}
			if(chr == 'y' || chr == 'Y')
			{
				StoreUee(ee);
				ReadUee(ee);
				PrintUee(ee);
			}
			break;
		case 'X':
		case 'x':
			//if(ee->ShutterFlag) CloseDoor();
			ResetToMon();
			break;

		default:
			break;
	}
	SerInFlush();
	return;
}


void GetData(struct idata *rec)
/************************************************************************
// Fills an standard ISAR UNCALIBRATED record structure.
************************************************************************/
{
	char repl[128], tmp[128];
	int i,j,k;
	
	// Initialise data structure
	rec->scan_pos=MISSING; 
	rec->kt15_target_temp=MISSING;
	rec->kt15_ambient_temp=MISSING;
	for(i=0;i<2;i++){
		rec->hallsw[i]=MISSING;
		rec->position[i]=MISSING;
	}
	for(i=0;i<3;i++)
		rec->gps[i]=MISSING;
	for(i=0;i<4;i++)
		rec->pni[i]=MISSING;
	for(i=0;i<8;i++){
		rec->ad18[i]=MISSING;
		rec->ad12[i]=MISSING;
	}
	
    // Get KT15 digital data
    rec->kt15_target_temp=GetKt15TargetTemp();
    rec->kt15_ambient_temp=GetKt15AmbientTemp();

	// Read the shutter Hall effect switches
	ReadSwitch(&rec->hallsw[0],&rec->hallsw[1]);

	for(i=0;i<8;i++){
		// Read the window thermistor mV signal.
		// Read the scan mirror thermistor mV signal.
		// Read the TT8 board thermistor temperature
		// Read the power supply
		rec->ad12[i] = AtoDReadMilliVolts(i);  // This can be bad
		rec->ad12[i] = AtoDReadMilliVolts(i);
	}
	
	// Read the USdigital encoder scan drum position
    EnableEncoder();
	rec->scan_pos = readEncoder(ee->drumref);
    DisableEncoder();
    if(ee->testmode)printf("Encoder position=%.2f\n", rec->scan_pos);
	
	// Read the PNI compass pitch roll and azimuth
    ReadPNI(&rec->pni[0],&rec->pni[1],&rec->pni[2],&rec->pni[3]);
	if(ee->testmode)printf("PNI: pitch=%.1f, roll=%.1f, compass=%.1f, temperature=%.1lf\n",
		 rec->pni[0],rec->pni[1],rec->pni[2],rec->pni[3]);

	// Read the GPS lat, lon, sog, cog, var and
	if( ReadGPS(reply) == OK ) {
		ParseGPS(reply, str);
		ReadGpsStr(str, rtc, &rec->position[0], &rec->position[1], 
		&rec->gps[0], &rec->gps[1], &rec->gps[2]);
	} else {
		rec->position[0]=MISSING; rec->position[1]=MISSING;
		rec->gps[0]=MISSING; rec->gps[1]=MISSING; rec->gps[2]=MISSING;
	}
	
    // Read all the 4017 channels. 1st read
    Read4017All(rec->ad18);
	
	return;
}


void MakeRecord(char *rawrecord, struct idata *rec)
/****************************************************************
*****************************************************************/
{
   	char timestring[30];
   	time_t tnow;
   	struct tm *t;
	
	GetTime(timestring);
	//
	sprintf(rawrecord,"$IS001,%s," 								// date and time
	" %6.2f," 	    										// scan_pos
    " %6.4f,%6.4f,%6.4f,%6.4f,%6.4f,%6.4f,%6.4f,%6.4f," 		//ad18[0->7]
    " %4d,%4d,%4d,%4d,%4d,%4d,%4d,%4d,"      				//ad12[0..7]
	" %1d,%1d,"  						// Hall switch 1 & 2
    "%5.1f,%5.1f,%5.1f,%5.1f," 			// pni sensor
	"%8.6f,%8.5f,"      				// lat & long
	"%4.1f,%4.1f,%4.1f," 				// gps data
    "%5.1f,%5.1f*00",          			// KT15 target and ambient tempartures
    timestring,             			// Date and Time at the start of the record
	rec->scan_pos,	   			// Scan drum position (Deg)
	rec->ad18[7],	   			// ORG analog mV
	rec->ad18[6],	   			// KT15 analog mV
	rec->ad18[5],	   			// BB2 T#3
	rec->ad18[4],	   			// BB2 T#2
	rec->ad18[3],	   			// BB2 T#1
	rec->ad18[2],	   			// BB1 T#3
	rec->ad18[1],	   			// BB1 T#2
	rec->ad18[0],	   			// BB1 T#1
    rec->ad12[0],           	// 5V reference for thermistors
    rec->ad12[1],           	// Thermistor A
    rec->ad12[2],           	// Thermistor B
    rec->ad12[3],           	// Thermistor C
    rec->ad12[4],           	// Thermistor D
    rec->ad12[5],           	// Window Thermistor
    rec->ad12[6],           	// TT8 Thermistor
    rec->ad12[7],           	// Input power
	rec->hallsw[0],    			// Shutter switch states
	rec->hallsw[1],	   			// Shutter switch states
	rec->pni[0],       			// pitch (Deg)
	rec->pni[1],       			// roll (Deg)
	rec->pni[2],       			// azimuth (Deg)
	rec->pni[3],				// PNI temperature
	rec->position[0],  			// latitude(Deg)
	rec->position[1],  			// longitude(Deg)
	rec->gps[0],       			// sog (kts)
	rec->gps[1],       			// cog (Deg)
	rec->gps[2],       			// var (Deg)
    rec->kt15_target_temp,  	// Target temperature from the KT15 (K)
    rec->kt15_ambient_temp); 	// Internal reference temperature of KT15 (K)		
	return;
}

/***************************************************
ANALOG TO DIGITAL CONVERSION
****************************************************/

void Read_Analog (void)
/*******************************************
	Read all 8 analog channels
*******************************************/
{
   int i;
   unsigned int		val;
   char	str[64];
   double adcmean[8], adcstdev[8];
   unsigned nsamps;

	nsamps = 0;

	// clear stat registers
	for(i=0; i<8; i++)
	{
		adcmean[i] = adcstdev[i] = 0;
	}


	/*****************
	* ADC LOOP
	******************/
	SerInFlush();
	while(!SerByteAvail())
	{
		printf("  ");
		for(i=0; i<8; i++)
		{
			val = GetADC(i);
			adcmean[i] += (double)val;
			adcstdev[i] += (double)val * (double)val;
			printf("%8d",val);
		}
		printf("\n");
		nsamps++;
		DelayMilliSecs(1000);
	}
	/*****************
	* COMPUTE STATS FOR THE LOOP
	******************/
	for(i=0; i<8; i++)
	{
		//printf("%10.4g, %12.4g\n", adcmean[i], adcstdev[i]);
		adcmean[i] /= (double)nsamps;
		adcstdev[i] = adcstdev[i] / (double)(nsamps) -
			adcmean[i] * adcmean[i];
		//printf("%12.6g\n",adcstdev[i]);
		adcstdev[i] = sqrt(adcstdev[i]);
		//printf("%12.6g\n",adcstdev[i]);
	}
	/*****************
	* PRINT OUT STATS
	******************/
	printf("Number samples: %d\n",nsamps);
//	printf("Mean :");
	printf("    ");
	for(i=0; i<8; i++)
	{
		printf("%8.1lf",adcmean[i]);
	}
	printf("\n");
//	printf("Stdev:");
	printf("   ");
	for(i=0; i<8; i++)
	{
		printf("%8.1lf",adcstdev[i]);
	}
	printf("\n");
}

unsigned GetADC(unsigned chan)
/*******************************************************************/
{
	#define NSAMPS	9
	unsigned int val[NSAMPS], mx, mn, imx, imn, nsamps, ix, i;
	double x;

	// TAKE NSAMP SAMPLES, MARK MAX AND MIN VALUES
	mx = 0;  mn = 5000;  imx=imn=0;
	x = 0;
	for(i=0; i<NSAMPS; i++){
		val[i] = AtoDReadMilliVolts(chan);
		x += val[i];

		if( val[i] >= mx ){
			imx = i;
			mx = val[i];
		}
		if( val[i] < mn ){
			imn = i;
			mn = val[i];
		}
	}

	//REMOVE MAX AND MIN VALUES THEN TAKE A MEAN
	x = x - val[imx] - val[imn];
	x = x / (double)(NSAMPS - 2) + 0.5;
	ix = (unsigned)x;

	return ix;
}


float GetKt15TargetTemp(void)
/********************************************
// Gets a KT15 target temperature.
//
// History:
// 2003-05-05, C J Donlon. Original
*********************************************/
{
   int i=0;
   char rep[100];

   rep[0]='\0';
   SendIrtCommand("TEMP\n", rep);
   if(rep[7] == 'K'){
      rep[6]='\0';
      return atof(rep);
   }else
      return MISSING;
}

float GetKt15AmbientTemp(void)
/*****************************************************
// Gets a KT15 reference temperature.
//
// History:
// 2003-05-05, C J Donlon. Original
******************************************************/
{
   int i=0;
   char rep[100],te[10];

   rep[0]='\0';
   SendIrtCommand("AMB ?\n", rep);
   if(rep[0] == 'A'){
      for(i=0;i<5;i++)
         te[i]=rep[i+9];
      te[6]='\0';
      return atof(te);
   }else
      return MISSING;
}


int SendIrtCommand(char *cmd, char *rep)
/*****************************************************
// Send a command to the KT15 and returns the reply
//
// History
// 2002-04-15 C J Donlon. New routine
// 2003-02-24 C J Donlon. No return string
// 2003-04-30 R Edwards.   Fixed delays etc                 
// 2003-05-22 C J Donlon.  Changed local var from reply to rep to avoid conflict
//                         with the global var reply at compile
******************************************************/
{
	int i;

	// Flush the TX buffer
	TSerInFlush(IRTTX);
    TSerInFlush(IRTRX);

	// Send the command to the KT15.  Commands are assumed not
	// to have a \n
    for(i=0;i<strlen(cmd);i++)
		TSerPutByte(IRTTX, cmd[i]);

	// Make sure there is a \n on the end of the command
	// so that it gets executed by the KT15
	TSerPutByte(IRTTX,'\n');
    DelayMilliSecs(200);

	// read the reply
    i=0; strcpy(rep,"No KT15 reply");
	while( TSerByteAvail(IRTRX) )
	{
        rep[i] = TSerGetByte(IRTRX);
		i++;
        rep[i+1]='\0';
	}

	// RETURN OK/NOTOK
	if( i == 0 )
		return NOTOK;
	else
		return OK;
}


float Read_BattVoltage(void)
/****************************
Read Battery Voltage
****************************/
{
	unsigned int battmv;
	float battv;

	AtoDReadMilliVolts(7);	// FIRST READING IS THROWN OUT
	DelayMilliSecs(5);
	battmv = ( AtoDReadMilliVolts(7) );
	battv = ((battmv * 5.142) / 1000);
	printf( "Chan 7 = %d,  Battery Voltage = %.1f\n", battmv, battv);

	return battv;
}

void GetTT8Temp(double *t, double *rt, double *vt)
/****************************
Read TT8 thermistor
****************************/
{
	unsigned int idum;
	
	AtoDReadMilliVolts(6);	// FIRST READING IS THROWN OUT
	idum = (AtoDReadMilliVolts(6));
	*vt = (double)idum;
	// 5 V REF, 10k REF RESISTOR
	*rt = 10000.0 * *vt / (5000 - *vt);
	*t = Steinhart44006(*rt);

	return;
}

void GetOtherTemps(double *t1, double *t2)
/****************************
Read two other thermistor on adc 4 and 5
reynolds isar 000114
****************************/
{
	double rt, vt;

	vt = (double)AtoDReadMilliVolts(4) / 1000;
	// 5 V REF, 10k REF RESISTOR
	rt = 10000.0 * vt / (5 - vt);
	*t1 = Steinhart44006( rt);

	vt = (double)AtoDReadMilliVolts(5) / 1000;
	// 5 V REF, 10k REF RESISTOR
	rt = 10000.0 * vt / (5 - vt);
	*t2 = Steinhart44006( rt);

	return;
}

/****************************************************
4017 18 BIT ADC
*****************************************************/
int	 Read4017(int chan, double *v)
/*************************************************
// READ THE 4017 CHANNEL
// input = chan (0-7)
//   v = output voltage
// return OK/NOTOK
//
// History:
// 14-01-2000 M R Reynolds: Original
***************************************************/
{
	char	s[10], r[65];
	double fdum;

	sprintf(s, "#03%d",chan);
	RS_485(s,r, 12000);  // required LMdelay for 4017 = 12000

	if( strlen(r) == 0 || sscanf((r+1),"%lf", &fdum ) == 0){
		*v = (double)MISSING;
		return NOTOK;
	}
	else{
		*v = fdum;
		return OK;
	}
}


int Read4017All(double *mv)
//
//
//
// READ ALL THE 4017 CHANNELs
// History:
// 14-01-2000 M R Reynolds: Original
// 2003-05-02: C J Donlon.  Modified to become a single read on the 4017 using
//			    #03 character.
// 2003-05-05: Reynolds & Donlon:  Updated to use a readall followed by a channel read
//                                 if the read all fails
{
        int i,j;
        char reply[120], s[10];
        double fdum;
	
	// Read all channels of the 4017 with 1 command
	// Returns: +2.5823+2.5823+2.5823+1.9889+1.9889+1.9889+0.6756+0.0456
        // is sometimes bad. Try 4 times then give up

        i=0; j=0;
        while(i < 4){
                i++;
		    // Get thge data
                RS_485("#03",reply,12000);

                // Parse the reply
                j=sscanf(reply,"%lf%lf%lf%lf%lf%lf%lf%lf",
                        (mv),(mv+1),(mv+2),(mv+3),(mv+4),(mv+5),(mv+6),(mv+7));

                // If j== 8 then we have everything we need
                if( j==8 )
                    break;
        }

        // If we have les than 8 values then try reading each value
        // as a separate read to the 4017
        if(j != 8){
               // Reset j
               j=0;
               for(i=0;i<8;i++){
                        sprintf(s,"#03%d",i);
                        RS_485(s,reply,12000);
                        if(strlen(reply) == 0 || sscanf((reply+1),"%lf",&fdum)==0)
                            *(mv+i) = (double)MISSING;
                        else{
                            *(mv+i) = fdum;
                            j++;
                        }
               }
        }
        // Return the number of measurements successfully
        // retrieved
        return j;
}


int	RS_485 (char *s, char *r, unsigned bitdelay)
/*********************************************************
//
//	RS-485 Channel Enable
//
//  Send the string 's' out the 485 port.
//  <cr> will be added to the end of the transmission
//
//  History:
//  17-11-1999 R M Reynolds origial -  need to tighten times for faster response
//  2003-05-02: C J Donlon  Modified to allow return of all ADAM4017 channels 
//  in 1 read.  Timing was preventing a full retun of all chars
**************************************************************/
{
	unsigned i, len;
	short rxok;
	ushort	indx;
	int	ch;
	char str[10];

	TSerInFlush(RX485);
	len = strlen(s);  // string length

	// turn 485 port to tx
	PSet(E,4);		// RE-  High for TX

	// Send the output characters
	for(i=0; i<len; i++)
		TSerPutByte( TX485, s[i] );
	TSerPutByte(TX485,13); // cr

	// return to receive
	LMDelay(bitdelay);  // 4017: 12000==> 0.5 * 12000 microsecs
	PClear(E,4);	    // RE-  Low for RX

	indx = 0;
	// Read an initial char
        // Allow time to load rx buffer: Assume 60 chars return from a readall on
	// the 4017 giving 64ms @ 9600 baud.
	// Use 75ms to provide adequate overhead
	DelayMilliSecs(75);  
	while( TSerByteAvail(RX485) ){
		ch = TSerGetByte(RX485);
		*(r+indx) = ch;
		indx++;
	}
	*(r+indx) = '\0';

	if( indx == 0 ) rxok = NOTOK;
	else rxok = OK;

	return rxok;
}


void SampleADCExt(void)
/********************************************************
Test the 18-bit Adam ADC circuit
**********************************************************/
{
	double dum[8], ddum[8], ddumsq[8];
	int i, npts;

	ddum[0]=ddum[1]=ddum[2]=ddum[3]=ddum[4]=ddum[5]=ddum[6]=ddum[7]=0;
	ddumsq[0]=ddumsq[1]=ddumsq[2]=ddumsq[3]=ddumsq[4]=ddumsq[5]=ddumsq[6]=ddumsq[7]=0;
	npts=0;
	SerInFlush();
	while( !SerByteAvail() )
	{
		Read4017All(dum);
		printf("%8.4lf %8.4f %8.4lf %8.4lf %8.4lf"
		"%8.4lf %8.4lf %8.4lf\n",
		 dum[0],dum[1],dum[2],dum[3],dum[4],
		 dum[5], dum[6], dum[7]);

		for(i=0; i<8; i++)
		{
			ddum[i] += (double)dum[i];
			ddumsq[i] += (double)dum[i] * (double)dum[i];
		}
		npts++;
		DelayMilliSecs(1000);
	}
	for(i=0; i<8; i++)
		MeanStdev((ddum+i), (ddumsq+i), npts, Missing);

	printf("%7.2f %7.2f %7.2f %7.2f %7.2f %7.2f %7.2f %7.2f\n",
		ddum[0],ddum[1],ddum[2],ddum[3],
		ddum[4],ddum[5],ddum[6],ddum[7]);
	printf("%7.2f %7.2f %7.2f %7.2f %7.2f %7.2f %7.2f %7.2f\n",
		ddumsq[0],ddumsq[1],ddumsq[2],ddumsq[3],
		ddumsq[4],ddumsq[5],ddumsq[6],ddumsq[7]);
	return;
}

// NOTE -- MATH FUNCTIONS

void	MeanStdev(double *sum, double *sum2, int N, double missing)
/********************************************
Compute mean and standard deviation from
the count, the sum and the sum of squares.
991101
*********************************************/
{
	if( N <= 2 )
	{
		*sum = missing;
		*sum2 = missing;
	}
	else
	{
		*sum /= (double)N;		// mean value
		*sum2 = *sum2/(double)N - (*sum * *sum); // sumsq/N - mean^2
		*sum2 = *sum2 * (double)N / (double)(N-1); // (N/N-1) correction
		if( *sum2 < 0 ) *sum2 = 0;
		else *sum2 = sqrt(*sum2);
	}
	return;
}


int sign (float input)
//======================================================
{
	return ((input < 0.0) ? NEG: POS);
}

float DiffAngle(float a2, float a1)
/**************************************************
// Compute the smallest angle arc between the
// a2 and a1.
//
//  History
//  01-11-1999 M R Reynolds original
***************************************************/
{
	float arc;

	arc = a2 - a1;
	if( abs(arc) > 180 ){
		if( sign(arc) > 0 )
			arc -= 360;
		else
			arc += 360;
	}

	return arc;
}


float	CheckAngle(float angle_in)
/****************************************
//  Check the input angle number and be sure it is
//  in the range 0 <= angle < 360.
//
//  History
//  01-11-1999 M R Reynolds original
********************************************/
{
	while( angle_in < 0 ) angle_in += 360.;
	while( angle_in >= 360. ) angle_in -= 360;
	return angle_in;
}





double Steinhart44006(double r)
/*************************************************
COMPUTE TEMPERATURE IN DEG C USING THE STEINHART-HART EQUATION
**************************************************/
{
	double	lnr, t1;
	double	cal[3];

	Cal44006(r, cal);

	// STEINHART-HART EQUATION -- deg K
	if(r < 4000 || r > 45000)
	{
		//printf("Therm resistance out of range\n");
		t1 = MISSING;
	}
	else
	{
		lnr = log(r);
		t1 = cal[0] + cal[1] * lnr  + cal[2] * lnr * lnr * lnr;
		t1 = 1.0 / t1 - 273.15;
		//printf("Steinhart: r = %.1lf,   temp = %.3lf\n", r, t1);
	}

	return t1;
}

void Cal44006(double r, double cal[])
/***********************************
Fill the calibration arrays for the Steinhart-Hart
computation.
r = resistance in ohms
cal[3] = the proper calibration
************************************/
{
	if( r < 6000 )
	{
		cal[0] = 9.810151e-4;
		cal[1] = 2.475142e-4;
		cal[2] = 1.190931e-7;
	}
	else if( r >= 6000 && r < 10000)
	{
		cal[0] = 9.973204e-4;
		cal[1] = 2.445059e-4;
		cal[2] = 1.340502e-7;
	}
	else if( r >= 10000 && r < 18000)
	{
		cal[0] = 9.725326e-4;
		cal[1] = 2.481298e-4;
		cal[2] = 1.230960e-7;
	}
	else if( r >= 18000 && r < 30000)
	{
		cal[0] = 9.987421e-4;
		cal[1] = 2.435048e-4;
		cal[2] = 1.432809e-7;
	}
	else if( r >= 30000 && r < 40000)
	{
		cal[0] = 7.637322e-4;
		cal[1] = 2.764841e-4;
		cal[2] = 4.738435e-8;
	}
	else if( r >= 40000)
	{
		cal[0] = 1.259431e-3;
		cal[1] = 2.078245e-4;
		cal[2] = 2.422488e-7;
	}
	//printf("cal_1=%.6e, cal_2=%.6e, cal_3=%.6e\n",cal[0], cal[1], cal[2]);
	return;
}

int		GetBBTemp(double vref, double Rref, double v[], double temp[], double tav[])	// fills BTemp[6] and BBmV[6]
/***************************************************
Read the two black body thermistors and then compute
a mean temperature for each one
INPUT:
 v[6] = output volts e.g 3.4456

OUTPUT
 temp[6] = output temperature corresponding to channels 0-5
 tav[2] =  best average guess for BB temps

DEFINES:
  MISSING  -99

isar version 000106 -- rmr
edit 010804 -- use switching
*****************************************************/
{
	int i, n;
	double rt, ta;
	
	Read4017All(v);
	
	/*****************
	READ ADC AND GET VOLTAGES
	*******************/
	for(i=0; i<6; i++) {
		if( *(v+i) != MISSING ) { //Read4017(i, (v+i)) == NOTOK )
			rt = Rref * *(v+i) / (vref - v[i]);
			temp[i] = Steinhart44006(rt);
		}
		else temp[i] = MISSING;
	}
	
	/***************
	COMPUTE AVERAGE TEMP
	skip bad values
	****************/
	// BB 1  -- AVG ONLY 1 AND 2
	n=0; ta=0;
	for(i=0; i<2; i++) {
		if( temp[i] != MISSING ) {
			n++;
			ta += temp[i];
		}
	}
	if( n == 0 )
		tav[0] = MISSING;
	else
		tav[0] = ta / (double)n;

	// BB 2 -- AVG ONLY 4 AND 5
	n=0; ta=0;
	for(i=3; i<5; i++) {
		if( temp[i] != MISSING ) { // missing = -99
			n++;
			ta += temp[i];
		}
	}
	if( n == 0 )
		tav[1] = MISSING;
	else
		tav[1] = ta / (double)n;

	return OK;
}



// NOTE -- PNI FUNCTIONS

void PniCommand(char *cmd, char *reply)
/*******************************************************
Send a string, with cr (13) on the end, to the PNI then
receive the reply.

reynolds 000105
********************************************************/
{
	unsigned len;

	// BE SURE STRING ENDS WITH A CR (13)
	len = strlen(cmd);
	if(cmd[len] != 13)
	{
		cmd[len] = 13;
		cmd[len+1]='\0';
	}
	printf("Cmd: %s", cmd);

	TSerInFlush(RXPNI);
	PniSendStr(cmd);
        PniGetStr(reply);
	return;
}


unsigned PniGetStr( char *s)
/*******************************************************
 Get a string from the precision navigation Inc. PNI tilt/az sensor.
 Need external define PNIWAIT delay time.
 History
 05-01-2000 M R Reynolds
 26-02-2003 C J Donlon Now outputs the raw data
 2003-03-26 C J Donlon Only prints if flag is set
*********************************************************/
{
	#define MAXSTR 64
	ulong t1;
        int ch, len, strflag, ix;

	t1 = TensMilliSecs() + PNIWAIT/10; // wait time in msec/10
	len = 0;
	strflag = NOTOK;
	TSerInFlush(RXPNI);
	ix=0; // flag for collecting data
	while(TensMilliSecs() < t1) {
		if( TSerByteAvail(RXPNI) ) {
			ch = TSerGetByte(RXPNI);
			if ( ch == '$' && ix == 0 ) {
				ix = 1;
				*(s+len) = ch;
				len++;
			}
			else if ( ix == 1) {
				*(s+len) = ch;
				len++; 
	            if( ch == 13 ) {
					*(s+len) = '\0';
					strflag = OK;
					break;
				}
				else if ( len > 60 ) {
					puts("PniGetStr error, too many characters.");
					break;
				}
			}
		}
	}
	*(s+len) = '\0';
	if( strflag == NOTOK ) { 
		puts("$ERRER, (Warning) PniGetStr() fails\n");
		return NOTOK;
	}
    else {
		return OK;
    }
}



unsigned PniSendStr( char *str )
/************************************************************
Send a string to the PNI tilt/azimuth sensor
IN: str is the string of characters.
You need to define RXPNI channel.
This function ensures a <cr> is on the end of the string.
OUT:
Returns number of characters sent.
History
05-01-2000 M R Reynolds
************************************************************/
{
	unsigned len, i;

	len = strlen(str);	// Be sure string has CR at end
	if( *(str + len) != 13 ){
		*(str+len) = 13;
		*(str+len+1) = '\0';
		len++;
	}

	for(i=0; i<len; i++)
		TSerPutByte(TXPNI,*(str+i));

	return len;
}

void ReadPNI(float *p, float *r, float *a, float *t)
/***********************************************************
// Read the PNI pitch/roll/azimuth sensor
//
// History
// 05-01-2000 M R Reynolds
// 2003-03-04 C J Donlon Added temperature field
***********************************************************/
{
	char *ptr;
	char *endptr;
    char cmd[6], reply[31];

	// SEND AN ENQUIRY COMMAND
	TSerInFlush(RXPNI);

        //strcpy(cmd,"s?");
        //PniSendStr( cmd );

	// READ IN THE RESPONSE STRING
	// v9 set bad values to zero.
	if ( PniGetStr(reply) == OK ){
			ptr = strchr(reply,'C');
		if( ptr == NULL )
			*a = 0;
		else
			*a = strtod( (ptr+1), &endptr );
		
		ptr = strchr(reply,'R');
		if( ptr == NULL )
			*r = 0;
		else
			*r = strtod( (ptr+1), &endptr );
		
		ptr = strchr(reply,'P');
		if( ptr == NULL )
			*p = 0;
		else
			*p = strtod( (ptr+1), &endptr );
		
		ptr = strchr(reply,'T');
		if( ptr == NULL )
			*t = 0;
		else
			*t = strtod( (ptr+1), &endptr );
	} 
	// v9 set all to zero if no data.
	else {
		*p = *r = *a = *t = 0;
	}
	return;
}

int	SerPutStr(char *s)
/*************************
Put out a string one character at a time
991101
**************************/
{
	int	i;
	//good to here

	for(i=0; i<strlen(s); i++)
	{
		SerPutByte(s[i]);
	}
	return strlen(s);
}

// NOTE -- GPS FUNCTIONS

int ReadGpsStr(char *str, struct tm *t, double *lat,
	double *lon, float *sog, float *cog, float *var)
/*************************************************************
 Read GPS string from parseGPS() function.  Output
 the navigation variables.

 input string: yyMMdd, hhmmss, llmm.mmm, lllmm.mmm, sss.ss, ddd.d, xx.x
 defines: MISSING

 output vars: time, lat, lon, sog, cog, var

 History
  08-01-2000 M R Reynolds
  19-04-2002 C J Donlon  Added additional tests to ensure good data
***************************************************************/
{
	double	 	f[2];
	long		d[2];
	int 		i;
	float 		fdum;

	sscanf(str, "%ld,%ld,%lf,%lf,%f,%f,%f",
		&d[0], &d[1], &f[0], &f[1], sog, cog, var);

	// If the time, lat & lon look good then use them
	if(d[0] != -999 && d[1]!= -999 && f[0]!=-99 && f[1]!=-99){

		// yy MM dd
		i = d[0] / 10000;
		if( i < 70 ) i += 100;  // yy >= 2000
				t->tm_year = i;
		i = d[0]/100;   i = i%100;  t->tm_mon = i-1;
		t->tm_mday = d[0]%100;
		// hh mm ss
		t->tm_hour = d[1]/10000;
		i = d[1]/100;  t->tm_min = i%100;
		t->tm_sec = d[1]%100;
	
		// lat
		i = (int) (fabs(f[0])/100.0); // degrees
		fdum = fabs(f[0]) - (double)i*100.0;
		*lat = sign(f[0]) * (float)i + fdum/60;
		// lon
		i = (int) (fabs(f[1])/100.0); // degrees
		//printf("i=%d\n",i);
		fdum = fabs(f[1]) - (double)i*100.0;
		//printf("fdum=%.4f\n",fdum);
		*lon = sign(f[1]) * ((float)i + fdum/60);
		// sog
		if( *sog == -99 ) *sog = MISSING;
		// cog
		if( *cog == -99 ) *cog = MISSING;
		// var
		if( *var == -99 ) *var = MISSING;
				return OK;
	}
	else {
		 *lat=MISSING;
		 *lon=MISSING;
		 *sog=MISSING;
		 *cog=MISSING;
		 *var=MISSING;
		 return NOTOK;
	}
}

int ReadGPS(char *strGPS)
/********************************************************************
//  Read Trimble GPS data.
//
//  History
//  07-01-2000 M R Reynolds original
//  2002-04-30 C J Donlon   Wait time updated to 1200ms from 1000ms
//  2003-03-03 C J Donlon   Changed method of waiting to speed up
//                          the routine.  Now we wait for a char to
//                          come onto the buffer for a set period of time
//  2003-03-26 C J Donlon   Added option to suppress printing of data to the screen
//  2004-03-14 C J Donlon   Moved print raw gps data to GetGPS();
//  2005-06-28 W Wimmer		Tidied the function up  
// 100201 rmr Adapted from isaro 7.17
***********************************************************************/
{
    int chr,spGPS,reportflag=0;
	unsigned long ulTime = 0;
	unsigned long ulTimeOut = 500000;
	bool bFoundStart = 0;

	// initialize output string
	spGPS = 0;
	strGPS[spGPS] = '\0';
	
	
	// IF NO DATA IN THE BUFFER THAN WAIT JUST ONE SEC
    // -------------------------------------------------------------------------------
    // Read the GPS buffer
    // -------------------------------------------------------------------------------
    
    //START TIMEOUT - SAFETY FEATURE
    StopWatchStart(); 
    // CHECK FOR AVAILABLE BYTES -- read the buffer looking for '$'
	// key hit to exit   
    while (ulTime < ulTimeOut)
    {
     	// READ CHARACTERS IN THE BUFFER with Timeout
		//chr = TSerGetByteTimeout(GPSRX,10000);
		if (TSerByteAvail(GPSRX)) chr = TSerGetByte(GPSRX);
		else {
			ulTime = StopWatchTime();
			continue;
		}	
		// IF '$' IS FOUND 
		// reset the read buffer and counter
		if(chr == '$')
		{
			// DELAY SMALL TIME TO WAIT FOR CHARACTERS
			// DelayMilliSecs(25);
			spGPS = 0;
			strGPS[spGPS] = '\0';
			bFoundStart = 1;
		}
		else {
			strGPS[spGPS] = (chr);
			spGPS++;
			strGPS[spGPS] = '\0';
		}
		
		if (chr == END && bFoundStart) {
			return OK;	
		}		
		
		
		if( spGPS > 150 ) {
			if(ee->testmode)puts("GPS message too long.");
			return NOTOK;
		}
		ulTime = StopWatchTime();
	}
	
	if( ulTime > ulTimeOut) {
    	if(ee->testmode) puts("Timeout on GPS port");
		return NOTOK;
	}
    return NOTOK;  // no '$' in buffer
   	
}



int 	ParseGPS(char *strGPS, char *bufGPS)
/*******************************************************************
 Parse the GPS string
 GPRMC,hhmmss.s,A,llmm.mmm,a,yyyyy.yyy,a,sss.ss,ddd.d,ddMMyy,xx.x,a*
 |     |          |        | |         | |      |     |      |
 0     6          17      26 28       38 40     47    53     60
 id    time       lat/min  | long/min  | speed  course ddMMyy mag dev
						 N/S=+/-     E/W = +/-
 GPGGA,hhmmss.s,llmm.mmm,a,yyyyy.yyy,a,x,ss,x.x,x.x,M
 |     |        |          |           |        |
 0     6		16         27          39       48
 id    time     lat/min    long/min   quality   Altitude
                                      0|1|2
 input: strGPS[500]
 output bufGPS[200]
	,yyMMdd,hhmmss,llmm.mmm,lllmm.mmm,sss.ss,ddd.d,xx.x

 History
 05-01-2000 M R Reynolds original expects NEMA 0183 RMC type string
 12-04-2002 C J Donlon.  Changed logic to return OK or NOTOK and
						   set the default string in the output buffer
						   to be 1900 01 01 00:00:00 when gps read fails
 19-04-2002 C J Donlon  Set all output to be -999 and -99
 2003-03-05 C J Donlon  Will now take advantage of GPGGA string time.  Returns
                        a dummy date of 990101.
****************************************************************/
{
        int i,ii,val;
        char    header[10],tim[10],id[8],ns,ew;
        double lat,lon;

	// CLEAR THE OUTPUT BUFFER set it to Jan 1st 1900
	// i.e. 000101
	strcpy(bufGPS,"-999,-999,-99.0,-99.0,-99.0,-99.0,-99.0\0");

        //CHECK THE NMEA HEADER
        for(i=0; i<5; i++)
             header[i] = strGPS[i];
        header[5] = '\0';


	// IF IN STRING IS OK THEN PARSE OTHERWISE RETURN NOTOK
        // AND THE STRING ABOVE.  DO GPRMC first as this has time,
        // if no GPRMC then try for a GPGGA
        if(strcmp(header,"GPRMC") == 0){
            if( strlen(strGPS) > 49 ){

                // COPY DATE INTO BUFGPS  ddMMyy ==>> yyMMdd
                i = 0;
                bufGPS[i++] = strGPS[57];
                bufGPS[i++] = strGPS[58];
                bufGPS[i++] = strGPS[55];
                bufGPS[i++] = strGPS[56];
                bufGPS[i++] = strGPS[53];
                bufGPS[i++] = strGPS[54];

                // COPY THE TIME
                bufGPS[i++] = ',';
                bufGPS[i++] = ' ';
                for(ii=0; ii<6; ii++)
                      bufGPS[i++] = strGPS[6+ii]; 
                bufGPS[i++] = ',';

                // LATITUDE SIGN
                if( strGPS[26] == 'S' ) bufGPS[i++] = '-';
                else bufGPS[i++] = ' ';

                // LATITUDE
                for(ii=0; ii<8; ii++)
                      bufGPS[i++] = strGPS[17+ii];
                bufGPS[i++] = ',';

                // LONGITUDE SIGN
                if( strGPS[38] == 'W' ) bufGPS[i++] = '-';
                else bufGPS[i++] = ' ';

                // LONGITUDE
                for(ii=0; ii<9; ii++)
                     bufGPS[i++] = strGPS[28+ii];

                // COPY SPEED
                bufGPS[i++] = ',';
                bufGPS[i++] = ' ';
                for(ii=0; ii<6; ii++)
                     bufGPS[i++] = strGPS[40+ii];

                // COPY COURSE
                bufGPS[i++] = ',';
                     bufGPS[i++] = ' ';
                for(ii=0; ii<5; ii++)
                     bufGPS[i++] = strGPS[47+ii];

                // COPY MAGNETIC DEVIATION
                bufGPS[i++] = ',';
                bufGPS[i++] = ' ';
                for(ii=0; ii<4; ii++) 
                     bufGPS[i++] = strGPS[60+ii];
                bufGPS[i] = '\0';

                return OK;
             }
        }

        // Try for a GPRMC string
        if(strcmp(header,"GPGGA") == 0){ 
           // GPGGA,hhmmss.s,llmm.mmm,a,yyyyy.yyy,a,x,ss,x.x,x.x,M
           // |     |        |          |           |        |
           // 0     6        15         26          38       47
           // id    time     lat/min    long/min   quality   Altitude
           //                                       0|1|2
           // GPGGA,095345.0,5053.558,N,00123.637,W,1,05,4.27,-00018,M,047,M,,*6A


           lat=-999.0;lon=-999.0;ew='N';ns='E';
           if(strlen(strGPS) > 38) {

                // Time
                for(i=6;i<12;i++)
                   tim[i-6]=strGPS[i];
                tim[6]='\0';

                // Latitude
                for(i=15;i<23;i++)
                   header[i-15]=strGPS[i];
                header[8]='\0';
                ns=strGPS[24];
                lat=atof(header);
                val=(int)lat/100;
                lat=val + ((lat-(double)val*100)/60.);   
                if(ns == 'S')
                    lat=-lat;

                // Longitude
                for(i=26;i<35;i++)
                   header[i-26]=strGPS[i];
                header[i-26]='\0';
                ew=strGPS[36];
                lon=atof(header);
                val=lon/100;
                lon=val + ((lon-(double)val*100)/60.);
                if(ew == 'W')
                    lon=-lon;

                // Build the output string
                sprintf(bufGPS,"990101,%s,%f,%f,-99,-99,-99",
                      tim,lat,lon);
                return OK;
             }
         }

return NOTOK;
}



/******************************************************************
POWER
******************************************************************/
// NOTE --- SWITCHED POWER AND MOTOR CONTROL

void SwPower(int chan, int cmd)
/*********************************
Toggle Switch Power 0,1,2,3 <==> A,B,C,D

input:
 chan = 0,1,2,3 <=> switched channels A,B,C,D
 TPU PIN = 0,1,2,3 also
 cmd = ON, OFF, TOGGLE  as defined at the top.

version isar-101 991227 rmr
**********************************/
{
	switch (cmd)
	{
		case ON:
			if( SwPwrFlag[chan] == OFF )
			{
				SerPutByte('P');
				switch(chan) {
					case 0: PSet(E,0); SerPutByte('A'); break;
					case 1: PSet(E,1); SerPutByte('B'); break;
					case 2: PSet(E,2); SerPutByte('C'); break;
					case 3:
					default: PSet(E,3); SerPutByte('D'); break;
				}
				SwPwrFlag[chan] = ON;
			}
			break;

		case OFF:
			if( SwPwrFlag[chan] == ON ) {
				SerPutByte('p');
				switch(chan)
				{
					case 0: PClear(E,0); SerPutByte('A'); break;
					case 1: PClear(E,1); SerPutByte('B'); break;
					case 2: PClear(E,2); SerPutByte('C'); break;
					case 3:
					default: PClear(E,3); SerPutByte('D'); break;
				}
				SwPwrFlag[chan] = OFF;
			}
			break;

		case TOGGLE:
			if( SwPwrFlag[chan] == ON )
				SwPower(chan,OFF);
			else
				SwPower(chan,ON);
	}
	return;
}


/*****************************************************
MOTOR TOOLBOX
motormotor
******************************************************/

int ScanMotor(int direction)
// Operate the scan motor in a FWD, REV, or STOP
// mode.
// Output= ScanMotorFlag;
// defines: FWD, REV, STOP
// global variables: ScanMotorFlag;
//
// History
// 08-01-2000 M R Reynolds
//
{
	int flag;

	switch(direction){
		case FWD:
			if( ScanMotorFlag != FWD ){
				TPUSetPin(3,1);
				TPUSetPin(2,0);
				flag = FWD;
			}
			break;

		case REV:
			if( ScanMotorFlag != REV ){
				TPUSetPin(2,1);
				TPUSetPin(3,0);
				flag = REV;
			}
			break;

		case STOP:
			if( ScanMotorFlag != STOP ){
				TPUSetPin(3,0);
				TPUSetPin(2,0);
				flag = STOP;
			}
			break;
	}

	ScanMotorFlag = flag;
	return flag;
}

int DoorMotor(int direction)
//====================================================
// Operate the door motor in a FWD, REV, or STOP
// mode.
// Output= DoorMotorFlag;
// defines: FWD, REV, STOP
// global variables: DoorMotorFlag;
//
// History
// 08-01-2000 M R Reynolds
//
{
	int flag;

	// Ping the watchdog timer
        PingWatchDog();

	switch(direction){
		case FWD:
			if( DoorMotorFlag != FWD ){
				TPUSetPin(4,1);
				TPUSetPin(5,0);
				flag = FWD;
			}
			break;

		case REV:
			if( DoorMotorFlag != REV ){
				TPUSetPin(5,1);
				TPUSetPin(4,0);
				flag = REV;
			}
			break;

		case STOP:
			if( DoorMotorFlag != STOP ){
				TPUSetPin(5,0);
				TPUSetPin(4,0);
				flag = STOP;
			}
			break;
	}

	DoorMotorFlag = flag;
	return flag;
}


// NOTE -- DOOR OPERATION FUNCTIONS

void ReadSwitch(int *state1, int *state2)
/*********************************************
Read hall Effect switch status
110731 v09 -- the switch block has been repaired so I01 and I04 are the same.
**********************************************/
{
	if ( ee->IsarSN == 1 ) {
		*state1 = Pin(E,6);
		*state2 = Pin(E,7);
	} else {
		*state1 = Pin(E,6);
		*state2 = Pin(E,7);
	}	
	return;
}



int	CloseDoor(void)
/**********************************************************
//  Close the ISAR shutter.  Start the door motor and
//  wait for the switch to close.  If it does not
//  function within DOOR_TIMEOUT secs, then stop and
//  send an error message
//
//  return: CLOSED or PROBLEM
//
//  History:
//  2002-03-15 C J Donlon  Revised for new Mk II isar design with switches in
//                        the correct position.  No need for Delay settings
 2010-02-01 rmr Rebuilt the switch fitting on isar01 so now
         ISAR04             ISAR01
 CLOSED i1=1, i2=0		i1=0, i2=1
 OPEN   i1=0, i2=1		i1=1, i2=0
 ??		i1=1, i2=1		i1=0, i2=0
***********************************************************/
{
	ulong	clk;
	int	i1,i2,OverShootMsecs;
	
	// Get THE TIME
	clk = MilliSecs();
	OverShootMsecs = 0;
	
	// Is the shutter OPEN? (i1=0 & i2=1)
	ReadSwitch(&i1, &i2);
	if(i2){
		DoorMotor(REV);
		while(i2) {
			if ( MilliSecs() - clk > (ulong)DOOR_TIMEOUT*1000 ) {
				DoorMotor(STOP);
				puts("Shutter CLOSE switch timeout");
				return NOTOK;
			}
			ReadSwitch(&i1, &i2);
		}
		DoorMotor(STOP);
	}
	return OK;
}



int	OpenDoor(void)
/*************************************************************
// Open the door.  Start the door motor and
// wait for the open switch to close.  If it does not
// function within DOOR_TIMEOUT secs, then stop and
// send an error message
//
// return OPEN or PROBLEM
//
//  History:
//  2002-03-15 C J Donlon  Revised for new Mk II isar design with switches in
//                        the correct position.  No need for Delay settings
2010-2-1 rmr rewired the switch bracket and reverse the direction of
	switching.
	OPEN	i1=0	i2=1
	CLOSED	i1=1	i2=0
	??		i1=1	i2=1
**************************************************************/
{
	ulong	clk;
	int	i1,i2,OverShootMsecs;
	
	// Get THE TIME
	clk = MilliSecs();
	OverShootMsecs = 0;

	// Get the switch states: 
	ReadSwitch(&i1, &i2);
	if(i1){
		// Start the motor until the switch is closed
		DoorMotor(FWD);
		while (i1){
			if( MilliSecs() - clk > (ulong)DOOR_TIMEOUT*1000 ){
				DoorMotor(STOP);
				puts("(Warning) Shutter OPEN switch timeout");
				return NOTOK;
			}
			ReadSwitch(&i1, &i2);
		}
		DoorMotor(STOP);
	}
	return OK;
}



// NOTE -- EEPROM FUNCTIONS

void StoreUee(struct eeprom *pu)
/**********************************************
Determines the size of the structure and stores it entirely
in eeprom space
991101
***********************************************/
{
	ushort i, location;
	uchar *ptst;

	location = MEMSTART;
	ptst = (uchar*)pu;
	printf("StoreUee...\n");

	if(PrintFlag) printf("Store Uee variables\n");
	location = MEMSTART;

	for(i=0; i < sizeof(struct eeprom); i++)
	{
		UeeWriteByte(location++, *ptst );  // get the byte
		ptst = ptst+1;
	}

	return;
}


void ReadUee(struct eeprom *pu)
/**********************************************
991101
**********************************************/
{
	ushort	i,location;
	uchar *ptst;

	location = MEMSTART;
	ptst = (uchar*)pu;

	printf("ReadUee:\n");
	for(i=0; i < sizeof(struct eeprom); i++)
	{
		UeeReadByte(location++, ptst++ );  // get the byte
	}


	return;
}

void PrintUee(struct eeprom *ep)
/****************************************
Print out the eeprom structure
991101
*****************************************/
{
	printf("PrintUee: \n"
			 "  C BB1 angle = %.2f deg\n"
			 "  H BB2 angle = %.2f deg\n"
			 "  S sky angle = %.2f deg\n"
			 "  O ocean angle = %.2f deg\n"
			 "  spare a1 = %.2f deg\n"
			 "  spare a2 = %.2f deg\n"
			 "  D drum zero ref = %.1f deg\n"
			 "  B BB sample count = %d\n"
			 "  U sky sample count = %d\n"
			 "  T ocean sample count = %d\n"
			 "  R rain threshold, volts = %.4f\n"
			 "  N Isar SN = %02d\n"
			 "  M Shutter on/off  = %d\n"
			 "  E SCAN_TOLERANCE,
			 ep->abb1, ep->abb2, ep->asky, ep->aocean, ep->a1, ep->a2, ep->drumref,
			 ep->Nbb, ep->Nsky, ep->Nocean, ep->rain_threshold, ep->IsarSN, ep->ShutterFlag,
			 ep->SCAN_TOLERANCE);
	return;
}


// NOTE -- SCAN DRUM FUNCTIONS


float PointScanDrum(float requestpos, float refpos)
/******************************************************
// Move the encoder angle to a requested position
// Choose the smallest arc angle and move in that direction.
// Within +/-8 degrees of the requested position
// We need to cycle the drive very fast to get the necessary read
// resolution using readEncoder().  Without this approach,
// readEncoder() returns a position of 2-3 degrees at full speed
// which is way beyond the precision of the A2 encoder.
//
// We may overshoot as we approach the requestpos and if there
// is not an unequal shift fwd/back, we enter a hysteresis state.
//
//
// History
// 2000-01-10  M R Reynolds   Original
// 2002-02-19  C J Donlon  New routine to account for backlash in gearbox
//             screwing up the nudging routine
// 2002-03-13  C J Donlon  New approach using scaled power delays seems to be
// 	           spot on.  Tolerance should be 0.05 degrees in isarconf.icf and
//			   assumes an encoder resolution of 14400
// 2002-04-05  C J Donlon.  Adjusted the SCAN_THRESHOLD so that the set position
//			   can actually be achieved within tolerance without error !!
// 2002-04-15  C J Donlon  Changed MULT1 from 15 to 9 and MULT2 from 10 to 5 and
//			   increased the delay between scanpos reads to 250 ms to help positioning.
// 2002-04-16  C J Donlon  Initial move to within 8 degrees not 3.
// 2003-04-15  C J Donlon  Added input error check to make sure all is OK and tightened code up
// 2003-04-24  C J Donlon  Checked for -999 return on readEncoder() routine
// 2003-05-02  C J Donlon  Added Enable/Disable Encoder statements to stop RS485 BUS
//			   contention
// 2003-05-03  C J Donlon  Changed delays to LMDelay to work in uSec time steps
//                         1ms ~4 degrees rotation at full pelt.  Using uSec delays
//                         means that we can have tiny motor movements.  Due to
//                         new 485 system with Rev D/C of ISAR PCB, complete
//                         rewrite
// 2003-05-05 C J Donlon   Added timeout function to protect against hysteresis
// 2004-01-20 C J Donlon   Mike Reynolds corrections for problem with negative differences
//			   and tightened up timing.
// 2004-01-21 C J Donlon   Worked hard on ISAR_02 and ISAR-03 to get the timing correct and minimise the
//                         hysteresis jitters.  Opened up timing up by an order of magnitude in the nudging sections leaving the 
//			   'landing zone' at 0.075 degrees.  This now works very well with #2 and #3
//			   We will not get rid of the jitters completely as this is a limitation of the
//			   system requirements.  As suggested by Mike Reynolds, tightening the initial 
//			   stop to within 5 degrees speeds the routine up.
// 2005-04-28 W Wimmer 		Minor changes in the routine to speed up positioning 
//
**************************************************************************************/
{
	float pos=0, diff=0,lmmult=1000;
	int direction=REV, iOldDirection=REV;
	float fStartPos, fDelay;
	unsigned long ulTime;
	
	if(ee->testmode) printf("PointScanDrum input angle = %.1f,  reference = %.1f\n",requestpos,refpos);
	// If Request is out of range return
	// MISSING as an error flag
	if(requestpos > 359.999 || requestpos < 0){
		if(ee->testmode)printf("Warning) Encoder request position is bad: %f",requestpos);
		return MISSING;
	}
	
	// Enable the Encoder
	EnableEncoder();
	
	// Get the initial encoder position
	pos = readEncoder(refpos);
	fStartPos = pos;
	// Compute the smallest arc between pos and
	// the requestpos
	diff = DiffAngle(requestpos,pos);
	
	// Only move the encoder if necessary
	if( fabs(diff) > ee->SCAN_TOLERANCE )
	{
		ulTime = MilliSecs(); // The beginning of the positioning.
		// Decide on the correct direction to start
		// the scan motor
		if( sign(diff) > 0 )
			direction = FWD;
		else
			direction = REV;
		
		// If the encoder is +/-5 degrees from requestpos
		// start the motors continuously until we are within
		// ~5 degrees
		if(fabs(diff) > 5) {
			//StopWatchStart();
			ScanMotor(direction);
			while (fabs(diff) > 5.0) {
				pos = readEncoder(refpos);
				diff = DiffAngle(requestpos,pos);
				if( MilliSecs() - ulTime > 10000 ) {
					puts("PointScanDrum timeout");
					DisableEncoder();
					return pos;
				}
			}
			ScanMotor(STOP);
		}
		
		// Now we are within +/-3 degrees of the requestpos
		// and we need to nudge the scan drum with smaller and
		// smaller increments in order to land within
		// +/- SCAN_TOLERANCE
		//printf("Scandrum pos %f direction %d\n",pos,direction);
		// Add a timeout to protect in case of hysteresis
		
		// -------------------------------- fine positioning ----------------------------------
		DelayMilliSecs(80);
		pos = readEncoder(refpos);
		diff=DiffAngle(requestpos,pos);
		while (fabs(diff) >= ee->SCAN_TOLERANCE) {
			iOldDirection = direction;			
			// Check the direction required and set the
			// delay time.  Use unequal time to prevent
			// a hysteresis loop.
			if( sign(diff) > 0 ){
				direction = FWD;
				// Tested with ISAR 01, 02 and 03
				// based on in situ timings
				// and will not work well
				// when the system is on the bench
				//if(diff > 0.5)  	// 1
				//lmmult=15000;	// 10000
				//else
				//   lmmult=3000;     
				// if not the same direction as before allow bigger step for hystereseis
				if (iOldDirection != FWD) 
					lmmult=20000;
				else
					lmmult=15000;
			}
			else {
				direction = REV;
				// Tested with ISAR 01,02 and 03
				// based on in situ timings
				// and will not work well
				// when the system is on the bench
				// Reynolds v103 bugfix here: need to use fabs() as the result will always be -tive
				//if(fabs(diff) > 0.5)	// 1
				 //lmmult=15000;// 10000
				//else
				//   lmmult=3000;
				if (iOldDirection != REV)
					lmmult=20000;
				else
					lmmult=15000;
			}
			
			// Start the motor for a small duration and check
			// the result at the while statement
			
			fDelay = fabs(diff)*lmmult;
			if (fDelay < 1500)
				fDelay=1500;
			
			ScanMotor(direction);
			LMDelay(fDelay);
			ScanMotor(STOP);
			
			// Encoder settle time
			DelayMilliSecs(120);
			pos = readEncoder(refpos);
			diff=DiffAngle(requestpos,pos);
			if( MilliSecs() - ulTime > 10000 ) {
				puts("PointScanDrum timeout"); 
				DisableEncoder(); 
				return pos;
			}
		}
	}
	
	// Read the final position
	pos =  readEncoder(refpos);
	
	// Disable the encoder to prevent RS485 bus ciontention with 
	// ADAM 4017 module
	DisableEncoder();
	
	return pos;
}

float readEncoder (float ref)
/***************************************************************
// Read the USDIGITAL encoder
// Absolute position is computed using output = encoder angle - ref
// NOTE: bracket this call with EnableEncoder() and DisableEncoder()
//
// History:
// 08-01-2000 M R Reynolds original
// 2002-04-02 C J Donlon.  A2 Encoders should be set to a resolution of
//                         14400 (0.025 deg resolution), 9600 baud, scale factor
//                         of 1 and no mode setting.
// 2003-04-17 C J Donlon   Added saftey timeout on encoder and then 10 tries
//			   to get a good result
// 2003-04-29 C J Donlon  & R Edwards revised for new ISAR
// 2003-05-01 C J Donlon  Returns a MISSING.  The Encoder is contending with other
// 			  485 devices.  Assign an address E, the idea being that other 
//			  485 modules shouldn't spit out a hex E char.  Also returns a
//			  MISSING if it fails. 
//
*****************************************************************/
{
	int ok=0,count=0,i;
	unsigned e[2];
	float enc;
	
	// Enable the Encoder
	EnableEncoder();
	
	while( count < 10) {
		TSerInFlush(RX485);
		TSerInFlush(TX485);
	
		// turn 485 port to tx
		PSet(E,4);			// RE-  High for TX
		LMDelay(1000);
		TSerPutByte (TX485,0x1E);        // query encoder for angle
		
		// turn off 485 transmitter, return to receive
		LMDelay(2350);  		// required delay for encoder
		PClear(E,4);			// RE-  Low for RX
		
		// WAIT FOR 6 DELAY TIMES FOR CHARACTERS TO BE IN
		// DelayMilliSecs (3);
		
		// Wait for a character to come in and timeout after 1/2 second
		StopWatchStart();
		i=0;
		while((StopWatchTime() < 2000) && i<2) {
			while(TSerByteAvail(RX485)) {
				e[i]=MISSING;
				LMDelay(2000);
				if(TSerByteAvail(RX485)) e[i]=TSerGetByte(RX485);
				else puts("(Warning) Encoder 2nd byte reply missing!");
				i++;
			}
			if(e[0] != MISSING && e[1] != MISSING) {
				// This next calc assumes an encoder setting of 14400 resolution
				enc = (float)(e[0]*256+e[1]) / 40.0; 
				return CheckAngle(enc-ref);
			}
		}
		count++;
	}
	puts("10 bad returns from A2 Encoder, scan drum position is undefined");
	return MISSING;
}


void DisableEncoder(void)
/*****************************************
//      2003-05-01:  R Edwards original
******************************************/
{
  TPUSetPin(1,1);
  TPUSetPin(15,0);
}


void EnableEncoder(void)
/***************************************
//      2003-05-01:  R Edwards original
***************************************/
{
  TPUSetPin(1,0);
  TPUSetPin(15,1);
}


// NOTE -- TIME FUNCTIONS


time_t ShowTime(void)
/***************************************

****************************************/
{
	char a[24];
	time_t now;

	now = GetTime(a);
	printf("Current time: %s\n",a);
	return now;
}


time_t	GetTime(char *a)
/**************************************
**************************************/
{
	struct tm *tt;
	time_t now;
	int len;

	// GET THE TIME
	now = time(NULL);
	tt = localtime(&now); // pointer to structure

	// PRINT THE TIME IN STD FORMAT
	sprintf(a,"%4d,%02d,%02d,%02d,%02d,%02d\0\n",
	tt->tm_year+1900, tt->tm_mon+1, tt->tm_mday, tt->tm_hour, tt->tm_min,
	tt->tm_sec);
	len = strlen(a);
	return now;
}


/***********************************************/
int Startup(void)
/***********************************************/
{

	// EXTERNAL WDOG SETUP
	TPUSetPin(0,0);		//XWDOG

	//RS-485 SETUP
	TPUSetPin(7,0);		//RO MAX485 RX
	TPUSetPin(8,0);     	//DI MAX485 TX

	// Encoder Setup: Start with Encoder OFF
	DisableEncoder();


	// HALL EFFECT SWITCH INPUTS
	PConfInp(E,6);  	// hall effect switch, pin 65
	PConfInp(E,7);		// hall effect switch, pin 64

	// SET THESE PINS TO OUTPUT FOR THE POWER SWITCHING
	PConfOutp(E,0);  	// A for SPARE
	PConfOutp(E,1);	// B for KT15
	PConfOutp(E,2);	// C for BB1
	PConfOutp(E,3);	// D for BB2
	PClear(E,0); PClear(E,1); PClear(E,2); PClear(E,3); 		// start w all power off
	SwPwrFlag[0]=SwPwrFlag[1]=SwPwrFlag[2]=SwPwrFlag[3]=OFF;  	// set flags

	// set up COM2 (KT15)
	if(TSerOpen(IRTRX,HighPrior,0,malloc(256+TSER_MIN_MEM),256,9600,'N',8,1) != 0)
			printf("ERROR,(Warning) Problem opening TPU:%d for KT15 RX\n",IRTRX);
	else
			printf("TPU:%d open for KT15 RX\n",IRTRX);
	if(TSerOpen(IRTTX,HighPrior,1,malloc(256+TSER_MIN_MEM),256,9600,'N',8,1) != 0)
			printf("ERROR,(Warning) Problem opening TPU:%d for KT15 TX\n",IRTTX);
	else
			printf("TPU:%d open for KT15 TX\n",IRTTX);

	// Set up Com3 (GPS)
	if(TSerOpen(GPSRX,HighPrior,0,malloc(256+TSER_MIN_MEM),256,4800,'N',8,1) != 0)
			printf("(Warning) Problem opening TPU:%d for GPS RX\n",GPSRX);
	else
			printf("TPU:%d open for GPS RX\n",GPSRX);
	if(TSerOpen(GPSTX,HighPrior,1,malloc(256+TSER_MIN_MEM),256,4800,'N',8,1) != 0)
			printf("(Warning) Problem opening TPU:%d for GPS TX\n",GPSTX);
	else
			printf("TPU:%d open for GPS TX\n",GPSTX);


	// Set up COM4 (PNI)
	if(TSerOpen(RXPNI,HighPrior,0,malloc(256+TSER_MIN_MEM),256,9600,'N',8,1) != 0)
			printf("(Warning) Problem opening TPU:%d for PNI RX\n",RXPNI);
	else
			printf("TPU:%d open for PNI RX\n",RXPNI);
	if(TSerOpen(TXPNI,HighPrior,1,malloc(256+TSER_MIN_MEM),256,9600,'N',8,1) != 0)
			printf("(Warning) Problem opening TPU:%d for PNI TX\n",TXPNI);
	else
			printf("TPU:%d open for PNI TX\n",TXPNI);


	// Set up Com5 (RS-485)  4017 and Encoder and External 485 devices
	if(TSerOpen(RX485,HighPrior,0,malloc(256+TSER_MIN_MEM),256,9600,'N',8,1) != 0)
			printf("(Warning) Problem opening TPU:%d for RS485 RX\n",RX485);
	else
			printf("TPU:%d open for RS485 RX\n",RX485);
	if(TSerOpen(TX485,HighPrior,1,malloc(256+TSER_MIN_MEM),256,9600,'N',8,1) != 0)
			printf("(Warning) Problem opening TPU:%d for RS485 TX\n",TX485);
	else
			printf("TPU:%d open for RS485 TX\n",TX485);

	// SETUP FOR 485 CHANNEL
	PConfOutp(E,4);   // RE- (Receive Enable active low)
	PClear(E,4);

	// MOTORS OFF
	TPUSetPin(2,0);	// scan motor
	TPUSetPin(3,0); // scan motor
	TPUSetPin(4,0);	// door motor
	TPUSetPin(5,0); // door motor

	// Turn thermistor power on continuously rather than switch it on and off
	// at measurement time.
	//BLACK BODY REF VOLTAGE SETUP
	TPUSetPin(6,0);               //ENABLES BB VREF
	return OK;
}

/************************************/
void PingWatchDog(void)
{
// Sets TPU port 0 high to stop watchdog
// timeout

	TPUSetPin(0,1);
	DelayMilliSecs(50);
	TPUSetPin(0,0);
}

