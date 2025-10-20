#include <TimerThree.h>

// Pin definitions
#define VALVE_A 3
#define VALVE_B 4
#define VALVE_C 5
#define VALVE_D 6
#define LED_PIN 13

// Sensor pins
#define P_SYS 1  // System Pressure Sensor
#define P_RES 2  // Reservoir Pressure Sensor
#define OXY 0    // Oxygen Sensor

#define VALVE_LOW_POWER		2000		// Reduced Power
#define VALVE_FULL_POWER	4095		// Full Power
#define PWM_DELAY_TIME		20			// mS

#define 	ValveAclosed		analogWrite(VALVE_A, 0); ValveA = 0
#define 	ValveAopen			analogWrite(VALVE_A, VALVE_FULL_POWER); ValveA = PWM_DELAY_TIME;
#define 	ValveBclosed		analogWrite(VALVE_B, 0); ValveB = 0
#define 	ValveBopen			analogWrite(VALVE_B, VALVE_FULL_POWER); ValveB = PWM_DELAY_TIME; 
#define 	ValveCclosed		analogWrite(VALVE_C, 0); ValveC = 0; ValveCmimic = false
#define 	ValveCopen			analogWrite(VALVE_C, VALVE_FULL_POWER); ValveC = PWM_DELAY_TIME; ValveCmimic = true;
#define 	ValveDclosed		analogWrite(VALVE_D, 0); ValveD = 0; ValveDmimic = false
#define 	ValveDopen			analogWrite(VALVE_D, VALVE_FULL_POWER); ValveD = PWM_DELAY_TIME; ValveDmimic = true;


// Variables for sensor readings
float ReservoirPressure = 0;
float SystemPressure = 0;
float OxygenLevel = 0;

// Variables for volumes and flow rate
float	static InhVolume, ExhVolume, Flow, Kerr, Sigma, PeakAirway, Pairway;

// Constants for flow calculation... from JV00004P4, Reference from previous arduino code 
#define A_VALVE_C 2.1256
#define N_VALVE_C 0.522
#define A_VALVE_D 5.1759
#define N_VALVE_D 0.5296

// *************************************************************************
// PRVC settings (default value)
float targetTidalVolume = 400; // Target tidal volume in milli-liters
float Ppeep = 15.0;             // Positive End-Expiratory Pressure (cmH2O)
uint16_t	FiO2 = 40 ;							// FiO2 as % (Range 30-100%)
float		IERatioSetting = 2 ;				// I:E Ratio 1:2 = 1 Inh + 2 Exh
uint16_t	RespiratoryRate = 20 ;				// Respiratory Rate  (BPM)
uint16_t	RequiredCycleTime = (1000 * 60)/RespiratoryRate ;			// Cycle time = 1/Respiratory Rate in milli Seconds
float		SysKv = 1.25 ;						// Kv value used to calulate Airway Poressure from System Pressure. This is a calibration value.
// *************************************************************************

// Timing variables
unsigned long lastDataTime = 0;
const unsigned long dataInterval = 20;   // Send data every 20 ms
const unsigned long inhalationTime = (float) RequiredCycleTime / (1.0 + IERatioSetting); // Inhalation duration in ms

// Mode selection
bool isManualMode = false; // If in manual mode, user controls valves directly
bool systemStarted = false; // whether the system has been started by "START" command

// *************************************************************************
// Values calculated from Control Values
uint16_t	Toxy = 150 ; 			// Duration for oxygen valve open in mS
uint16_t	Tair = 410 ; 			// Duration for air valve open in ms 
// *************************************************************************

// *************************************************************************
// Values use in phase transition
#define _SWITCH_		true					// Rem out to not use the switch.

#define VALVE_A_PULLIN		8
#define VALVE_B_PULLIN		8
#define VALVE_C_PULLIN		9
#define VALVE_D_PULLIN		8

#define VALVE_DELAY			32			// Optimized for no pressure glitch.

#define VALVE_A_DROPOUT		34
#define VALVE_B_DROPOUT		34
#define VALVE_C_DROPOUT		30
#define VALVE_D_DROPOUT		74			// mS

#define	WINDOW_TIME			50			// Window to find PEEP peak after valve D closes in mS 

#define ALARM_PIN			2
#define LED_PIN				13
#define ENABLE_PIN			32

#define FILTER_SIZE		50              // Adjustable filter size for moving average, original is 1000

boolean		Tick = false ;
uint16_t	TimerTick = 0 ;
uint16_t	CycleTimer = 0 ;

uint16_t	Phase1Time = inhalationTime-VALVE_C_DROPOUT;		// mS (Inhalation)
uint16_t	Phase2Time = inhalationTime;						// mS (Pause/Hold)
uint16_t	Phase6Time = 100;					// mS (End of cycle)

boolean		ValveCmimic = false ;
boolean		ValveDmimic = false ;


static int currentPhase = 0;   // Start from phase 4 since, the first time running the reservior pressure is 0
boolean		static ChangePhase = true;
float		static PeepMax, PeepError = 0 , PeepOffset;
float		Peep = 0.0;
float		PiP = 0.0;
uint16_t	ValveA = 0; 
uint16_t	ValveB = 0; 
uint16_t	ValveC = 0; 
uint16_t	ValveD = 0; 

uint16_t	Temp16, Res16, VolumeCalcDelay;
uint16_t static OpenDelay = VALVE_DELAY, Tfill, ValveDDropout, ValveCDropout;
float	static Error, FlowSwitchValue, PeepMin, FlowAtSwitch;
int16_t	static WindowValid = 0;
// *************************************************************************


// *************************************************************************
// Alarm setting 
int highPressureThreshold = 35; // Default 35 cmH2O
int lowPressureThreshold = -5;  // Default -5 cmH2O
unsigned long lastPressureCheck = 0;
int blockageThreshold = 5;  // Default 5 cmH2O above PEEP
int breathCount = 0;
int hyperventilationThreshold = 30;  // Default 30% of set VT
int hypoventilationThreshold = -30;  // Default -30% of set VT
int leakageThreshold = 35;  // Default 35% of VT
// *************************************************************************

// *************************************************************************
// PEEP Control
float LungPressure = 0.0;
// --- For capturing pre-/post-close values to measure R_lung
static float  preDFlowLpm       = 0.0f;
float ePEEP = 1.5f;    // initial offset (Eqn.20 suggests ePEEP=1.5)
float delta_ePEEP = 0.1f; // step size for updating offset each breath
#define PRE_MEASUREMENT_DELAY_MS 0  // Reduced from 100 ms for more precise pre-closure measurement
#define POST_MEASUREMENT_DELAY_MS 500  // Reduced from 1000 ms for faster post-closure measurement

static float preDPress = 0.0f;
static float postDPress = 0.0f;
static unsigned long valveDCloseTime = 0;
static bool awaitingPostClose = false;
// *************************************************************************

//*****************************************************************
// Read system pressure
float ReadSysPressure(void)
{
    // Define constants for configuration
    const float CONVERSION_FACTOR = 0.08568179;  // Conversion from ADC to cmH2O
    float SysPressureOffset = 2.60;             

    // Static variables for moving average
    static uint16_t Buffer[FILTER_SIZE];  // Circular buffer for storing ADC values
    static uint16_t i = 0;                      // Index for circular buffer
    static uint32_t Total = 0;                  // Running total of ADC values

    // Update moving average
    Total -= Buffer[i];                         // Subtract the oldest value from the total
    uint16_t Temp16 = analogRead(P_SYS);        // Read new ADC value
    Buffer[i] = Temp16;                         // Store the new value in the buffer
    Total += Temp16;                            // Add the new value to the total

    // Wrap index for circular buffer
    if (++i > FILTER_SIZE-1){
        i = 0;
    }

    float Result = (Total * 0.08568179 / FILTER_SIZE) - SysPressureOffset; // Calculate pressure in cmH2O
    return Result;  // Return the calculated pressure
}

// Read reservoir pressure
float ReadResPressure(void) {
    // Variables for the rolling average filter
    float ResPressureOffset = 0.0;     
    static uint16_t Buffer[FILTER_SIZE]; // Rolling buffer to store ADC readings
    static uint16_t i = 0;               // Current buffer index
    static uint32_t Total = 0;           // Running total for averaging
    uint16_t Temp16;                     // Temporary variable for the current ADC reading

    // Update the rolling buffer
    Total -= Buffer[i];                  // Remove the oldest ADC value from the total
    Temp16 = analogRead(P_RES);          // Read the current ADC value from the reservoir pressure sensor
    Buffer[i] = Temp16;                  // Store the new ADC reading in the buffer
    Total += Temp16;                     // Add the new ADC reading to the running total

    // Update the buffer index (circular buffer logic)
    if (++i >= FILTER_SIZE) {            // If index exceeds the buffer size
        i = 0;                           // Wrap it back to the start
    }
    // Use external voltage reference for ADC conversion and convert to cmH₂O
    // Multiplier 0.244873 is derived for cmH₂O with external reference
    float Result = (Total * 0.244873 / FILTER_SIZE) - ResPressureOffset; // Final pressure in cmH₂O
    return Result; 
}


// Read oxygen level
float ReadOxygenLevel() {
    float rawValue = analogRead(OXY);
    return ((rawValue - 1489) * 100.0) / (2150 - 1489);
}

//*****************************************************************
// This interrupt will fire every 1mS
//*****************************************************************

void TickInterrupt(void)
{
	Tick = true;
	TimerTick++;
	CycleTimer++;
}

// Send sensor data
void sendSensorData() {
    Serial.print("ReservoirPressure:");
    Serial.print(ReservoirPressure, 2);
    Serial.print(", SystemPressure:");
    Serial.print(SystemPressure, 2);
    Serial.print(", OxygenLevel:");
    Serial.print(OxygenLevel, 2);
    Serial.print(", InhVolume:");
    Serial.print(InhVolume / 60, 2);
    Serial.print(", ExhVolume:");
    Serial.print(ExhVolume / 60, 2);
    Serial.print(", Flow:");
    Serial.print(Flow, 2);
    Serial.print(", ValveDState:");
    Serial.print(ValveDmimic ? "OPEN" : "CLOSED");
    Serial.println();
}


void setup() {
    // Initialize serial communication
    Serial.begin(115200);

    // Set valve and LED pins as output
    pinMode(VALVE_A, OUTPUT);
    pinMode(VALVE_B, OUTPUT);
    pinMode(VALVE_C, OUTPUT);
    pinMode(VALVE_D, OUTPUT);
    pinMode(LED_PIN, OUTPUT);

    // Turn off all valves initially
    digitalWrite(VALVE_A, LOW);
    digitalWrite(VALVE_B, LOW);
    digitalWrite(VALVE_C, LOW);
    digitalWrite(VALVE_D, LOW);

    Timer3.initialize(1000);				// 1mS Tick Timer
	  Timer3.attachInterrupt(TickInterrupt);	// Run every 1 mS

    // light LED to indicate system start
    digitalWrite(LED_PIN, HIGH);

    pinMode(LED_PIN, OUTPUT);
	  pinMode(ENABLE_PIN, INPUT_PULLUP);

    analogReference(0);
	  analogReadResolution(12);
    analogWriteResolution(12);

    Serial.println("System powered. Waiting for 'START' command...");
}

void loop() {
    while (Tick==false){ 
      ReservoirPressure = ReadResPressure();
      SystemPressure = ReadSysPressure();
      OxygenLevel = ReadOxygenLevel();
    } 
    Tick = false;

    unsigned long currentMillis = millis();
    if (currentMillis - lastDataTime >= dataInterval) {
        ReservoirPressure = ReadResPressure();
        SystemPressure = ReadSysPressure();
        OxygenLevel = ReadOxygenLevel();
        sendSensorData();
        lastDataTime = currentMillis;
    }

    // Handle incoming commands for mode and valve control
    if (Serial.available()) {
        String command = Serial.readStringUntil('\n');
        handleCommand(command.trim());
    }

    // Only run the automatic control logic if "systemStarted" is true
    if (systemStarted && !isManualMode) {
        autoControlLogic();
    }
}

// Handle mode selection and manual valve control commands
void handleCommand(String command) {
    if (command.startsWith("TIDAL_VOLUME:")) {
        targetTidalVolume = command.substring(13).toFloat();
        Serial.println("ACK: Tidal Volume Updated");
    } else if (command.startsWith("PPEEP:")) {
        Ppeep = command.substring(6).toFloat();
        Serial.println("ACK: PPEEP Updated");
    } else if (command.startsWith("RESP_RATE:")) {
        RespiratoryRate = command.substring(10).toInt();
        RequiredCycleTime = (1000 * 60) / RespiratoryRate; // Recalculate cycle time
        Serial.println("ACK: Respiratory Rate Updated");
    } else if (command.startsWith("FIO2:")) {
        FiO2 = command.substring(5).toInt(); // Assuming FiO2 is an integer percentage
        Serial.println("ACK: FiO2 Updated");
    } else if (command.startsWith("IE_RATIO:")) {
        String ratio = command.substring(9); // Format: "1:2"
        int colonIndex = ratio.indexOf(':');
        if (colonIndex > 0) {
            float inhale = ratio.substring(0, colonIndex).toFloat();
            float exhale = ratio.substring(colonIndex + 1).toFloat();
            IERatioSetting = exhale / inhale; // Calculate ratio as a fraction (e.g., 1:2 = 2.0)
            Serial.println("ACK: I:E Ratio Updated");
        }
    } else if (command.startsWith("HIGH_PRESSURE_ALARM:")) {
        highPressureThreshold = command.substring(20).toInt();
        Serial.println("ACK: High Pressure Alarm Threshold Updated");
    } else if (command.startsWith("LOW_PRESSURE_ALARM:")) {
        lowPressureThreshold = command.substring(19).toInt();
        Serial.println("ACK: Low Pressure Alarm Threshold Updated");
    } else if (command.startsWith("BLOCKAGE_ALARM:")) {
        blockageThreshold = command.substring(15).toInt();
        Serial.println("ACK: Blockage Alarm Threshold Updated");
    } else if (command.startsWith("HYPERVENTILATION_ALARM:")) {
        hyperventilationThreshold = command.substring(24).toInt();
        Serial.println("ACK: Hyperventilation Alarm Threshold Updated");
    } else if (command.startsWith("HYPOVENTILATION_ALARM:")) {
        hypoventilationThreshold = command.substring(23).toInt();
        Serial.println("ACK: Hypoventilation Alarm Threshold Updated");
    } else if (command.startsWith("LEAKAGE_ALARM:")) {
        leakageThreshold = command.substring(14).toInt();
        Serial.println("ACK: Leakage Alarm Threshold Updated");
    } else if (command == "START") {
        systemStarted = true;
        Serial.println("ACK: System Started. Automatic logic is now enabled (if in AUTO mode).");
    } else if (command == "STOP") {
        systemStarted = false;  // Set the system to stopped
        stopAllProcesses();   // Call a function to reset processes
        Serial.println("ACK: System Stopped. All processes halted.");
    } else if (command == "MODE_MANUAL") {
        isManualMode = true;
        Serial.println("ACK: Switched to Manual Mode");
    } else if (command == "MODE_AUTO") {
        isManualMode = false;
        Serial.println("ACK: Switched to Automatic Mode");
    } else if (isManualMode) {
        // Handle manual valve control commands
        if (command == "VALVE_A_OPEN") {
            ValveAopen;
            Serial.println("ACK: Valve A Opened");
        } else if (command == "VALVE_A_CLOSE") {
            ValveAclosed;
            Serial.println("ACK: Valve A Closed");
        } else if (command == "VALVE_B_OPEN") {
            ValveBopen;
            Serial.println("ACK: Valve B Opened");
        } else if (command == "VALVE_B_CLOSE") {
            ValveBclosed;
            Serial.println("ACK: Valve B Closed");
        } else if (command == "VALVE_C_OPEN") {
            ValveCopen;
            Serial.println("ACK: Valve C Opened");
        } else if (command == "VALVE_C_CLOSE") {
            ValveCclosed;
            Serial.println("ACK: Valve C Closed");
        } else if (command == "VALVE_D_OPEN") {
            ValveDopen;
            Serial.println("ACK: Valve D Opened");
        } else if (command == "VALVE_D_CLOSE") {
            ValveDclosed;
            Serial.println("DEBUG: Valve D Close Command Received from Python (Manual Mode)");
            Serial.println("ACK: Valve D Closed");
        } else {
            Serial.println("ERR: Unknown Command");
        }
    } else {
        Serial.println("ERR: Manual mode required for this command");
    }
}

void stopAllProcesses() {
    // Turn off all valves
    digitalWrite(VALVE_A, LOW);
    digitalWrite(VALVE_B, LOW);
    digitalWrite(VALVE_C, LOW);
    digitalWrite(VALVE_D, LOW);

    // Reset logic variables
    InhVolume = 0.0;
    ExhVolume = 0.0;
    Flow = 0.0;

    // Optionally turn off any status LEDs or alarms
    digitalWrite(LED_PIN, LOW);

    // Log the stop event
    Serial.println("All processes stopped.");
}


// Automatic mode logic
void autoControlLogic() {
    static unsigned long phaseStartTime = 0;
    unsigned long currentTime = millis(); // millis() function counts time in ms
    

    float pressureDiffInh = 0.0;
    float pressureDiffExh = 0.0;
    // Check for serial commands from Python
    if (Serial.available()) {
        String command = Serial.readStringUntil('\n').trim();
        if (command == "VALVE_D_CLOSE") {
            Serial.println("DEBUG: Valve D Close Command Received from Python (Auto Mode)");
            ValveDclosed;
            ValveDDropout = VALVE_D_DROPOUT; // Set dropout time (74ms)
            awaitingPostClose = true;
            valveDCloseTime = currentTime;
            Serial.println("ACK: Valve D Closed by PEEP Control");
        }
    }

    switch (currentPhase) {
        case 0: // Initialization Phase
            if (ChangePhase){
              ChangePhase=false;
              digitalWrite(VALVE_A, LOW);
              digitalWrite(VALVE_B, LOW);
              digitalWrite(VALVE_C, LOW);
              InhVolume = 0.0;  // Reset inhalation volume
              ExhVolume = 0.0;  // Reset exhalation volume
            }
#ifdef _SWITCH_ 	
      if (digitalRead(ENABLE_PIN)==LOW)
#endif
      {			
        ChangePhase = true;
        phaseStartTime = currentTime;
        TimerTick = 0;
        currentPhase = 1;
        PeepOffset = Peep + (Peep * 0.1) ;  
      }
        break;

       case 1: // Inhalation Phase
            if (ChangePhase) {
                TimerTick = 0;
                CycleTimer = 0;
                ValveDclosed; // Close Valve D
                Serial.println("DEBUG: Valve D Triggered Locally (Phase 1 - Inhalation Start)");
                ValveCopen; // Open Valve C for inhalation
                ChangePhase = false;
                PiP = 0.0;                // Reset Peak Inspiratory Pressure
                PeakAirway = 0.0;         // Reset Peak Airway Pressure
                Peep = SystemPressure;
                InhVolume = 0.0;
                VolumeCalcDelay = VALVE_C_PULLIN; // Pull-in delay for Valve C
            } else {
                if (VolumeCalcDelay) { // Don't accumulate flow if valve is shut
                    VolumeCalcDelay--;
                    Flow = 0.0;
                    LungPressure = SystemPressure;
                } else {
                    pressureDiffInh = ReservoirPressure - SystemPressure;
                    if (pressureDiffInh < 0.0) {
                        pressureDiffInh = 0;
                    }
                    Flow = A_VALVE_C * (pow(pressureDiffInh, N_VALVE_C)); // Flow in L/min
                    InhVolume += Flow;
                    float Q_in_L_s = Flow / 60.0; // Convert L/min to L/s
                }
                if (SystemPressure > PiP) {
                    PiP = SystemPressure;
                }
                Pairway = SystemPressure - ((0.0707 * Flow / SysKv) * (0.0707 * Flow / SysKv));
                if (Pairway > PeakAirway) {
                    PeakAirway = Pairway;
                }
                if (InhVolume / 60 >= targetTidalVolume || TimerTick >= inhalationTime) {
                    ValveCclosed; // Close Valve C
                    ChangePhase = true;
                    currentPhase = 2;
                }
            }
            break;

        case 2: // Pause Phase
            if (ChangePhase) {
                ChangePhase = false;
                ValveCDropout = VALVE_C_DROPOUT;
            } else if (TimerTick > Phase2Time) {
                ChangePhase = true;
                currentPhase = 3;
                TimerTick = 0;
            }
            break;

        case 3: // Exhalation Phase
            if (ChangePhase) {
                ValveAopen; // Open Valve A for exhalation
                ValveDopen; // Open Valve D for exhalation
                VolumeCalcDelay = VALVE_D_PULLIN; // Pull-in delay for Valve D
                ExhVolume = 0.0;
                ChangePhase = false;
                TimerTick = 0;
                PeepMax = 0.0; // Reset PeepMax
                WindowValid = WINDOW_TIME; // Search window for PEEP peak
                awaitingPostClose = false;
            } else {
                if (ValveDmimic) { // Valve D open
                    if (VolumeCalcDelay) {
                        VolumeCalcDelay--;
                        Flow = 0.0;
                    } else {
                        pressureDiffExh = SystemPressure;
                        if (pressureDiffExh < 0.0) {
                            pressureDiffExh = 0;
                        }
                        Flow = A_VALVE_D * (pow(pressureDiffExh, N_VALVE_D)); // Flow in L/min
                        ExhVolume += Flow;
                        Flow *= -1;
                        float Q_ex_L_s = fabs(Flow) / 60.0;
                    }
                    // --------- NEW: Early PEEP protection in Phase 3 ----------
                    if (SystemPressure <= (Ppeep + ePEEP)) {
                        preDPress = SystemPressure;          // snapshot before closing
                        Serial.print("Valve D Closure threslhold pressure at case 3: "); //////////////////////////////////////////////new added on 6/21/////////////////////////////////////////////////////////////////////
                        Serial.println(Ppeep + ePEEP);
                        ValveDclosed;                        // close NOW (no pre-wait)
                        ValveDDropout = VALVE_D_DROPOUT;     // start seating timer
                        valveDCloseTime = millis();          // for post-closure sample
                        awaitingPostClose = true;            // enable post sample
                    }
                } else if (ValveDDropout) { // Valve D closing
                    ValveDDropout--;
                    pressureDiffExh = SystemPressure;
                    if (pressureDiffExh < 0.0) {
                        pressureDiffExh = 0;
                    }
                    Flow = A_VALVE_D * (pow(pressureDiffExh, N_VALVE_D));
                    ExhVolume += Flow;
                    Flow *= -1;
                    float Q_ex_L_s = fabs(Flow) / 60.0;
                } else { // Valve D fully closed
                    Flow = 0;
                    LungPressure = SystemPressure;
                    if (WindowValid) {
                        WindowValid--;
                        if (SystemPressure > PeepMax) {
                            PeepMax = SystemPressure;
                        }
                    }
                }
              if (TimerTick > (Toxy + VALVE_A_PULLIN - VALVE_A_DROPOUT)) {
                  ValveAclosed; // Close Valve A
                  ChangePhase = true;
                  OpenDelay = VALVE_DELAY;
                  currentPhase = 4;
              }
            }
            break;

        case 4: // Air Mixing Phase
          static unsigned long preCloseTime = 0;
          if (ChangePhase) {
              if (--OpenDelay == 0) {
                  ValveBopen;
                  ChangePhase = false;
                  TimerTick = 0;
                  if (VolumeCalcDelay) {
                      VolumeCalcDelay--;
                  } else {
                      pressureDiffExh = SystemPressure;
                      if (pressureDiffExh < 0.0) {
                          pressureDiffExh = 0;
                      }
                      Flow = A_VALVE_D * (pow(pressureDiffExh, N_VALVE_D));
                      ExhVolume += Flow;
                      Flow *= -1;
                  }
              }
          } else {
              if (ValveDmimic) {
                  if (VolumeCalcDelay) {
                      VolumeCalcDelay--;
                  } else {
                      pressureDiffExh = SystemPressure;
                      if (pressureDiffExh < 0.0) {
                          pressureDiffExh = 0;
                      }
                      Flow = A_VALVE_D * (pow(pressureDiffExh, N_VALVE_D));
                      ExhVolume += Flow;
                      Flow *= -1;

                      if (SystemPressure <= (Ppeep + ePEEP)) {  //////////////////////////////////////////////new added on 7/11 (this is wrong, should be lower than plung=PEEP+ePEEP) /////////////////////////////////////////////////////////////////////
                          Serial.print("Valve D Closure threslhold pressure: "); //////////////////////////////////////////////new added on 6/21/////////////////////////////////////////////////////////////////////
                          Serial.println(Ppeep + ePEEP); //////////////////////////////////////////////new added on 6/21/////////////////////////////////////////////////////////////////////

                          preDPress = SystemPressure;  // take it now, no wait
                          ValveDclosed;
                          ValveDDropout = VALVE_D_DROPOUT;
                          valveDCloseTime = millis();
                          awaitingPostClose = true;
                      }
                  }
              } else if (ValveDDropout) {
                  ValveDDropout--;
                  pressureDiffExh = SystemPressure;
                  if (pressureDiffExh < 0.0) {
                      pressureDiffExh = 0;
                  }
                  Flow = A_VALVE_D * (pow(pressureDiffExh, N_VALVE_D));
                  ExhVolume += Flow;
                  Flow *= -1;
              } else {
                  Flow = 0;
                  if (WindowValid) {
                      WindowValid--;
                      if (SystemPressure > PeepMax) {
                          PeepMax = SystemPressure;
                      }
                  }
              }

              if (TimerTick > (Tair + VALVE_B_PULLIN - VALVE_B_DROPOUT)) {
                  ValveBclosed;
                  ChangePhase = true;
                  OpenDelay = VALVE_DELAY;
                  currentPhase = 5;
              }
          }
          break;

        case 5: // Check if pressure is released or not, if not continue ValveD opens
          postDPress = SystemPressure; // Assign to existing static variable
          if (ValveDmimic) { // Valve D open
              if (VolumeCalcDelay) {
                  VolumeCalcDelay--;
              } else {
                  pressureDiffExh = SystemPressure;
                  if (pressureDiffExh < 0.0) {
                      pressureDiffExh = 0;
                  }
                  Flow = A_VALVE_D * (pow(pressureDiffExh, N_VALVE_D));
                  ExhVolume += Flow;
                  Flow *= -1;
              }
          } else if (ValveDDropout) { // Valve D closing
              ValveDDropout--;
              pressureDiffExh = SystemPressure;
              if (pressureDiffExh < 0.0) {
                  pressureDiffExh = 0;
              }
              Flow = A_VALVE_D * (pow(pressureDiffExh, N_VALVE_D));
              ExhVolume += Flow;
              Flow *= -1;
          } else {
              Flow = 0;
              if (WindowValid) {
                  WindowValid--;
                  if (SystemPressure > PeepMax) {
                      PeepMax = SystemPressure;
                  }
              }
          }

          if (CycleTimer > (RequiredCycleTime - Phase6Time)) {
              if (ValveDmimic) {
                  ValveDclosed; // Close Valve D
                  Serial.println("DEBUG: Valve D Triggered Locally (Phase 5 - Cycle End Fallback)");
              }
              ChangePhase = true;
              currentPhase = 6;
          }
          break;

        case 6: // End of Cycle Phase, all valves closes and maintain lung pressure until a cycle time is reached
            if (CycleTimer > RequiredCycleTime) {
                float finalPEEP = SystemPressure; //////////////////////////////////////////////new added on 7/11 (should not be directing equal to current system pressure, should find the lowest point when valve D closes)/////////////////////////////////////////////////////////////////////
                float peepError = Ppeep - finalPEEP; 
                ePEEP = ePEEP + (delta_ePEEP * (peepError/abs(peepError))); //////////////////////////////////////////////new added on 7/11 (this is also wrong, should follow the equation from design document) /////////////////////////////////////////////////////////////////////

                ChangePhase = true;
                currentPhase = 0;
                TimerTick = 0;
                ExhVolume /= 60;
                LungPressure = SystemPressure;

                Serial.print("Achieve PEEP:"); //////////////////////////////////////////////new added on 7/11 (should be the lowest point of the pressure)/////////////////////////////////////////////////////////////////////
                Serial.print( finalPEEP, 2);   //////////////////////////////////////////////new added on 7/11 (should be the lowest point of the pressure)/////////////////////////////////////////////////////////////////////
                Serial.print("PEEP error:");
                Serial.println(peepError);
                Serial.print("ePeep:"); //////////////////////////////////////////////new added on 6/21/////////////////////////////////////////////////////////////////////
                Serial.println(ePEEP);  //////////////////////////////////////////////new added on 6/21/////////////////////////////////////////////////////////////////////

            }
          break;
    }

    if (awaitingPostClose && (millis() - valveDCloseTime >= POST_MEASUREMENT_DELAY_MS)) {
        postDPress = SystemPressure;
        Serial.print("POST_DPRESS:");
        Serial.println(postDPress, 2);
        awaitingPostClose = false;
    }
}