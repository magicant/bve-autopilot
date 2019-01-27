//=============================
// BVE ATS Plug-in Header File
//
//             Rock_On, mackoy
//=============================

#ifdef ATS_EXPORTS
#define ATS_API __declspec(dllexport)
#else
#define ATS_API __declspec(dllimport)
#endif

// ATS Plug-in Version
#define ATS_VERSION	0x00020000

// ATS Keys
#define ATS_KEY_S	0	// S Key
#define ATS_KEY_A1	1	// A1 Key
#define ATS_KEY_A2	2	// A2 Key
#define ATS_KEY_B1	3	// B1 Key
#define ATS_KEY_B2	4	// B2 Key
#define ATS_KEY_C1	5	// C1 Key
#define ATS_KEY_C2	6	// C2 Key
#define ATS_KEY_D	7	// D Key
#define ATS_KEY_E	8	// R Key
#define ATS_KEY_F	9	// F Key
#define ATS_KEY_G	10	// G Key
#define ATS_KEY_H	11	// H Key
#define ATS_KEY_I	12	// I Key
#define ATS_KEY_J	13	// J Key
#define ATS_KEY_K	14	// K Key
#define ATS_KEY_L	15	// L Key

// Initial Position of Handle
#define ATS_INIT_REMOVED	2	// Handle Removed
#define ATS_INIT_EMG		1	// Emergency Brake
#define ATS_INIT_SVC		0	// Service Brake

// Sound Control Instruction
#define ATS_SOUND_STOP			-10000	// Stop
#define ATS_SOUND_PLAY			1		// Play Once
#define ATS_SOUND_PLAYLOOPING	0		// Play Repeatedly
#define ATS_SOUND_CONTINUE		2		// Continue

// Type of Horn
#define ATS_HORN_PRIMARY	0	// Horn 1
#define ATS_HORN_SECONDARY	1	// Horn 2
#define ATS_HORN_MUSIC		2	// Music Horn

// Constant Speed Control Instruction
#define ATS_CONSTANTSPEED_CONTINUE	0	// Continue
#define ATS_CONSTANTSPEED_ENABLE	1	// Enable
#define ATS_CONSTANTSPEED_DISABLE	2	// Disable

// Vehicle Specification
struct ATS_VEHICLESPEC
{
	int BrakeNotches;	// Number of Brake Notches
	int PowerNotches;	// Number of Power Notches
	int AtsNotch;		// ATS Cancel Notch
	int B67Notch;		// 80% Brake (67 degree)
	int Cars;			// Number of Cars
};

// State Quantity of Vehicle
struct ATS_VEHICLESTATE
{
	double Location;	// Train Position (Z-axis) (m)
	float Speed;		// Train Speed (km/h)
	int Time;			// Time (ms)
	float BcPressure;	// Pressure of Brake Cylinder (Pa)
	float MrPressure;	// Pressure of MR (Pa)
	float ErPressure;	// Pressure of ER (Pa)
	float BpPressure;	// Pressure of BP (Pa)
	float SapPressure;	// Pressure of SAP (Pa)
	float Current;		// Current (A)
};

// Received Data from Beacon
struct ATS_BEACONDATA
{
	int Type;		// Type of Beacon
	int Signal;		// Signal of Connected Section
	float Distance;	// Distance to Connected Section (m)
	int Optional;	// Optional Data
};

// Train Operation Instruction
struct ATS_HANDLES
{
	int Brake;		// Brake Notch
	int Power;		// Power Notch
	int Reverser;	// Reverser Position
	int ConstantSpeed;		// Constant Speed Control
};

// Called when this plug-in is loaded
ATS_API void WINAPI Load(void);

// Called when this plug-in is unloaded
ATS_API void WINAPI Dispose(void);

// Returns the version numbers of ATS plug-in
ATS_API int WINAPI GetPluginVersion(void);

// Called when the train is loaded
ATS_API void WINAPI SetVehicleSpec(ATS_VEHICLESPEC);

// Called when the game is started
ATS_API void WINAPI Initialize(int);

// Called every frame
ATS_API ATS_HANDLES WINAPI Elapse(ATS_VEHICLESTATE, int *, int *);

// Called when the power is changed
ATS_API void WINAPI SetPower(int);

// Called when the brake is changed
ATS_API void WINAPI SetBrake(int);

// Called when the reverser is changed
ATS_API void WINAPI SetReverser(int);

// Called when any ATS key is pressed
ATS_API void WINAPI KeyDown(int);

// Called when any ATS key is released
ATS_API void WINAPI KeyUp(int);

// Called when the horn is used
ATS_API void WINAPI HornBlow(int);

// Called when the door is opened
ATS_API void WINAPI DoorOpen(void);

// Called when the door is closed
ATS_API void WINAPI DoorClose(void);

// Called when current signal is changed
ATS_API void WINAPI SetSignal(int);

// Called when the beacon data is received
ATS_API void WINAPI SetBeaconData(ATS_BEACONDATA);