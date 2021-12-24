// Implementation of the multistatic interference radar concept: any object moving inside the field between two (or more) wifi radios modifies the propagation paths and therefore induces a variation in the RSSI values of each. 
// The purpose of this library is to filter, enhance, detect and report such variations. 

// this library requires the presence of one or more access point in range, but no connection between STA and AP is actually required. 
// in order to work correctly, this library requires the WiFi.h library, also, some of the function calls require ESP32 specific functionality, therefore the "esp_wifi.h" library is another requirement. 
// finally, the diagnostic messages use the Arduino.h library, but these can be easily converted to standard println calls. 

// Please note that the core function of this library, int multistatic_interference_radar_process(int sample, transmitterData *transmitterX) { ...
// does NOT require any hardware-specifc library to work, since it just makes use of a pretty standard data struct and processes RSSI samples in the form of signed integers. 
// The WiFi and esp_wifi libraries are only used by higher level functions that gather RSSI samples, do WiFi scans AND do the more complex job of sorting out the data from wifi scans 
// for you rather than having you do that externally on your own.




// WARNING: YOU'RE SUPPOSED TO HAVE ALREADY INITIALIZED WIFI BEFORE USING THIS LIBRARY (i.e. WiFi.begin() and mode (WIFI_STA or WIFI_STA_AP) have already been set, and WiFi is already connected to an external AP or has one client already connected

// Note: the library internally uses the standard wifi.h library but can also be made to work without such library if the rssi value is passed as a parameter. 



// standard includes
#include <stdint.h>


//// there are no initialization functions, the structures and arrays are already declared, initialized and accessible 




////// multistatic_interference_radar library
//
// library core definitions, declarations and functions
//


// WARNING: THESE LIMITS CANNOT BE CHANGED RUNTIME>\: AT THE MOMENT THE BUFFER SIZES ARE HARDWIRED. CHANGE THEM DEFORE COMPILING.
#define MAX_SAMPLEBUFFERSIZE_MULTI 32

#define MAX_AVERAGEBUFFERSIZE_MULTI 16

#define MAX_VARIANCEBUFFERSIZE_MULTI 16  // keep it equal to, or lower than MAX_SAMPLEBUFFERSIZE_MULTI

#define MAX_VARIANCE_MULTI 65535 // in dBm^2

#define VARIANCE_INTEGRATOR_LIMIT 3 

#define VARIANCE_THRESHOLD 16 // this may or may not be a reasonable value depending of too many factors, it has to be tuned according to your specific situation. 

#define MINIMUM_RSSI -80  // in dBm   // minimum acceptable RSSI to consider a transmitter valid for processing (too low the RSSI, the results might become meaningless)

#define ABSOLUTE_RSSI_LIMIT -128  // in dBm  // normaly you should never need to touch this value, it's already lower than the physical limits of most radios.


#define ABSOLUTE_MAX_SCAN_RESULTS 64  //

#define ENABLE_FIR_IIR_SECOND_ORDER 1 // 0 = disable; 1 or above = enable // enable filtering the variance signal to cancel offsets in case of very weak signals

#define ENABLE_SERIAL_CSV_DATA 0 // [ 0 = disabled, >=1 = enabled (default is 0) ] completely disable the verbose output and only print the variance foreach transmitter in s CSV format over the serial console, so that any program listening to the serial can graph the relevant data. 

#define ENABLE_RSSI_CLEANER 0 // aggressively remove transmitters with subpar signals



// ERROR LEVELS 

#define WIFI_UNINITIALIZED -8
#define WIFI_MODEINVALID -7
#define RADAR_INOPERABLE -6
#define RADAR_UNINITIALIZED -5
#define RADAR_BOOTING -4  // from -1 to -4 anything is RADAR_BOOTING


// STRUCTS
//
// plase note: for the moment I see no harm in esposing the library's internals. 
// you will be able to access the variance data directly, I'm ok with this. 
// Just, BE aware not to do out-of-bound reads if you're accessing the sample, variance and mobile average arrays. 






typedef struct  transmitterDataStruct {

int sampleBuffer[MAX_SAMPLEBUFFERSIZE_MULTI] = {0};

int sampleBufferSize = MAX_SAMPLEBUFFERSIZE_MULTI;  // DO NOT EXCEED THIS VALUE

int sampleBufferIndex = 0;

int sampleBufferValid = 0;

int latestReceivedSample = ABSOLUTE_RSSI_LIMIT;

int mobileAverageBufferSize = MAX_AVERAGEBUFFERSIZE_MULTI;  // DO NOT EXCEED THIS VALUE

int mobileAverageBuffer[MAX_AVERAGEBUFFERSIZE_MULTI] = {0};

int mobileAverageTemp = 0;

int mobileAverageFilterSize = MAX_SAMPLEBUFFERSIZE_MULTI; // you cannot calculate a mobile average past the maximum numer of samples in your sample buffer... that would exceed the array size and cause a crash  // DO NOT EXCEED THIS VALUE

int mobileAverage = 0;

int mobileAverageBufferIndex = 0;

int mobileAverageBufferValid = 0;

int varianceBufferSize = MAX_VARIANCEBUFFERSIZE_MULTI;  // DO NOT EXCEED THIS VALUE

int varianceBuffer[MAX_VARIANCEBUFFERSIZE_MULTI] = {0}; // holds the variance values

//// PLEASE NOTE: the init value is -1, meaning the variance is invalid, it will stay invalid until it can be effectively calculated
// DO NOT CHANGE THE -1 INITIALIZATION VALUE
int variance = RADAR_BOOTING; // this value is calculated from the current sample and the average calculated from the mobileAverageBuffer values

int variancePrev = 0;

int varianceSample = 0; // deviation of the current sample from the average

int varianceBufferIndex = 0;

int varianceBufferValid = 0;

int varianceIntegral = 0;

int varianceIntegratorLimitMax = MAX_SAMPLEBUFFERSIZE_MULTI;  // DO NOT EXCEED THIS VALUE

int varianceIntegratorLimit = VARIANCE_INTEGRATOR_LIMIT;  // DO NOT EXCEED varianceIntegratorLimitMax 

int varianceAR = 0;  // autoregressive version 


int bufferIndex = 0; // index used for all buffers. // currently unused lol


int enableThreshold = 0; // 0 = disable threshold; 1 = enable...

int varianceThreshold = VARIANCE_THRESHOLD; // in dBm

int FIRvarianceAvg = 0;

int detectionLevel = 0; // holds the detected level integrated from the varianceBuffer

bool enableAutoRegressive = false; // true to enable

int minimum_RSSI = MINIMUM_RSSI; // the define is -128dBm, but you should set it to a more reasonable -75 or -80 dBm

int currentRSSI = MINIMUM_RSSI;

int initComplete = 0; // 0 = buffers never initialized, 1 = buffers initialized with live data  // substantially unused at the moment.

int resetRequest = 0; // when 1, the pointers are reset to zero and this variable is toggled back to zero. 

int alarmStatus = 0; // 0 = no alarm; >1 triggered (above the varianceThreshold value)

} transmitterData;


// we'll allow UP TO 4 transmitters in the multistatic system. Basically: the strongest and nearest, one per non-overlapping channel plus one spare.


#define MAX_ALLOWED_TRANSMITTERS_NUMBER 4  // YOU MUST CHANGE THIS IF YOU NEED MORE TRANSMITTERS

static transmitterData transmitterA; //  I'm not gonna use an init function. These structs are small enough to let me keep the matters simple...
static transmitterData transmitterB; //  
static transmitterData transmitterC; //  
static transmitterData transmitterD; //  


// YOU MUST UNCOMMENT THIS IF YOU NEED MORE TRANSMITTERS
//transmitterData transmitterE; //  uncomment these to add more points, if you're adventurous. You'll have to modify the library code as well, or this will not work
//transmitterData transmitterF; //  
//transmitterData transmitterG; //  
//transmitterData transmitterH; //  




#define AP_SLOT_STATUS_VALID 0

#define AP_SLOT_STATUS_FREE 1

#define AP_SLOT_STATUS_INIT 2

#define AP_SLOT_STATUS_INVALID 3




typedef struct  multistaticDataStruct {

transmitterData transmittersData[MAX_ALLOWED_TRANSMITTERS_NUMBER] = {transmitterA, transmitterB, transmitterC, transmitterD};  // of course there are 4 transmitters as default value. // YOU MUST CHANGE THIS IF YOU NEED MORE TRANSMITTERS

int transmittersListLen = MAX_ALLOWED_TRANSMITTERS_NUMBER; // see above, update it if you plan to use more than 4 transmitters)

uint8_t BSSIDs[MAX_ALLOWED_TRANSMITTERS_NUMBER][6] = {{0}}; //holds the BSSID list, in 6 bytes format

uint8_t netItemNumbers[MAX_ALLOWED_TRANSMITTERS_NUMBER] = {}; //holds the netItem numbers for the current cycle // all of these are updated during the current cycle

// uint8_t BSSIDs[MAX_ALLOWED_TRANSMITTERS_NUMBER][6] = {{0}}; // placeholder, remove this line when cleaning

uint8_t APslotStatus[MAX_ALLOWED_TRANSMITTERS_NUMBER] = {0}; // 0 Valid; 1 free; init 2; invalid 3; // this will be updated runtime by the library, DO NOT TOUCH

char SSIDs[MAX_ALLOWED_TRANSMITTERS_NUMBER][34] = {{0}}; // the maximum allowed SSID length is 32 characters, (or 31 + NULL terminator), for extreme safety we will use 34 and accomodate 32 max (hardwired)

int currentTransmitterIndex = 0; // which of the MAX_ALLOWED_TRANSMITTERS_NUMBER we are currently processing. This will allow the code to runtime-pinpoint the relevant data in each array

int activeTransmittersNumber = 0; // updated per each roundrobin cycle from APslotStatus[], each valid slot increases this variable by one.

int discoveredNetworks = 0; // this is only used to parse the wifi scan results

int absoluteRSSIlimit = ABSOLUTE_RSSI_LIMIT; // in dBm

int initComplete = 0; // 0 = reinitialize the transmitters ranks, 1 = process the existing ranks

int scanIndexByPower[ABSOLUTE_MAX_SCAN_RESULTS] = {0}; // ABSOLUTE_MAX_SCAN_RESULTS bytes array used to sort the scan results by RSSI,  will only be used when and if we need to fill in empty transmitter slots in the data stucture. 

int scanIndexByPowerFirstFreeSpot = 0;

int latestVariances[MAX_ALLOWED_TRANSMITTERS_NUMBER] = {{0}}; // here you'll find the latest processing results, in the form of variance values, accordingto the transmitter index. 

int secondOrderFilter = ENABLE_FIR_IIR_SECOND_ORDER; // default enabled (1), reset to 0 to disable  // useful to stabilize the variance output in crowded environments with a lot of weak signals

int secondOrderAttenutationCoefficient = 16; // don't set it to 0 or 1, it needs to be at least 2 or above. If <= 1 very bad things might happen.

int RSSIcleanerEnable = ENABLE_RSSI_CLEANER; // aggressively remove transmitters with subpar signals

int serialCSVdataEnable = ENABLE_SERIAL_CSV_DATA; // output only data in CSV format, good for plotting the variance data

} multistaticData;


static multistaticData accessPoints;








// main functions
   
//// no init and deinit functions for the data arrays


// NOTE: there is no reconfiguration function. The data structures are exposed already, you may directly change every parameter, taking care to NEVER exceed the array boundaries and the maximum defined limits.

// current status: IMPLEMENTED // architecture-independent
 // receives the transmitter data structure (don't forget to check the default values in multistatic_interference_radar.h) and the related RSSI signal as parameters, 
 // returns the detection level in dBm^2 ( < 0 -> error (see ERROR LEVELS section), == 0 -> no detection, > 0 -> detection level in dBm^2)
int multistatic_interference_radar_process(int sample, transmitterData *transmitterX);

// current status: IMPLEMENTED // ESP32 and Arduino architecture-dependent
int multistatic_interference_radar(); // ESP32 specific version: does all the the scans, classification, and requests the RSSI level internally, then processes the signal and returns the detection level in dBm^2




// SERVICE / CONFIG FUNCTIONS

// current status: IMPLEMENTED
int multistatic_interference_radar_debug_via_serial(int);  // parameter is debug level, set it to at least >= 1; the highest the level, the more messages you enjoy

// current status: IMPLEMENTED
int multistatic_interference_radar_set_debug_level(int);  // parameter is debug level, set it to at least >= 1; the highest the level, the more messages you enjoy

// current status: IMPLEMENTED
int multistatic_interference_radar_set_txN_limit(int); // forcefully reduces the number of processed transmitters; CAVEAT: do not exceed MAX_ALLOWED_TRANSMITTERS_NUMBER

// current status: IMPLEMENTED
int multistatic_interference_radar_enable_second_order_variance_filtering(int); // [ 0 = disabled, >=1 = enabled (1 recommended) ] enhances the second order derivative for very slow drifting variances, nullifies long-term variance offsets in crowded environments. As a wanted side-effect, when a transmitter accidentally becomes to weak to measure reliably and data stops being gathered, the terminal variance offset is gracefully compensated by the finite impulse response filter. 

// current status: // current status: IMPLEMENTED
int multistatic_interference_radar_enable_aggressive_cleaning_low_RSSI(int); // 0 = disabled, 1 = enabled (0 recommended) aggressively clean the transmitters list in case of low signals. // I don't recommentd enabling it unless you debug your system

// current status: IMPLEMENTED
int multistatic_interference_radar_enable_serial_CSV_graph_data(int); // [ 0 = disabled, >=1 = enabled (default is 0) ] completely disable the verbose output and only print the variance foreach transmitter in CSV format over the serial console, so that any program listening to the serial can graph the relevant data. 


// current status: IMPLEMENTED
int multistatic_interference_radar_set_Second_Order_Attenutation_Coefficient(int); // don't set it to 0 or 1, it needs to be at least 2 or above. If <= 1 very bad things might happen.

// current status: IMPLEMENTED
int multistatic_interference_radar_set_minimum_RSSI(int);

// current status: IMPLEMENTED
int multistatic_interference_radar_enable_alarm(int);


// current status: IMPLEMENTED
int multistatic_interference_radar_set_alarm_threshold(int);

//
