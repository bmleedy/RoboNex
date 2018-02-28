/*reference:
 * https://www.arduino.cc/reference/en/language/functions/digital-io/digitalwrite/
 * our board:
 * https://smile.amazon.com/gp/product/B074WMHLQ4/ref=oh_aui_search_detailpage?ie=UTF8&psc=1
 * language reference:
 * https://www.arduino.cc/reference/en/
 * 
 * IDEAS: 
 *  Remove absurd amount of wait time in SR04 reading - Fire and check multiple ultrasonics simultaneously? Set timeout to much lower value
 *  34.3 cm/ms speed of sound @ maximum range of 1m * 2 trips = 200/34.3 = 5.83 ms maximum wait time (call it 6ms)
 *  Add steering
 *  Add a rubber band launcher: http://www.instructables.com/id/Rubber-Band-Gun-Using-an-Arduino/
 *  Add police Lights / reverse lights for the bus
 *  Allow control of lights from the remote
 *  Add control via wifi / webpage (maybe using raspberry pi?)
 *  Make wiring more solid (create a whitewire board for relays and power supply)
 *  Get some unit tests
 *  
 *  Expected time for pings to run: 
*/

#include "NewTone.h"
#include "IRremote.h"
#include "SR04.h"


/////////////////////////////
////////DEFINITIONS//////////
/////////////////////////////

//Debugging flag handled at compile-time to reduce the compiled size of non=verbose code
#define VERBOSE false

#define BEEP_LENGTH_MILLIS 500

#define DESIRED_LOOP_DURATION 500 //todo: this loop duration should be less than 100 ms

////////////// Desired State //////////////

struct {
  int velocity = 0;        // mm/s
                           // If set to zero, go forever.
  bool beeper_on = false;  // true = on
  void dump()
  {
    Serial.print("velocity:  "); Serial.println(velocity);
    Serial.print("beeper_on: "); Serial.println(beeper_on);
  }
} desired;


////////////// Current Mesured State //////////////
struct {
  // Ultrasonics
  long front_range_cm = 0;  // measured range, cm
  long rear_range_cm  = 0;  // measured range, cm
  bool front_rangefinder_inhibit = false;
  bool rear_rangefinder_inhibit = false;
  // IR Receiver
  decode_results results;   // IR receiver results 
  // Velocity command
  int command_velocity = 0; // mm/s
  void dump()
  {
    Serial.print("front_range_cm:            "); Serial.println(front_range_cm);
    Serial.print("rear_range_cm:             "); Serial.println(rear_range_cm);
    Serial.print("front_rangefinder_inhibit: "); Serial.println(front_rangefinder_inhibit);
    Serial.print("rear_rangefinder_inhibit:  "); Serial.println(rear_rangefinder_inhibit);
    Serial.print("results.value:           0x"); Serial.println(results.value, HEX);
  };
} measured;

////////////// Ultrasonics Definitions //////////////

// Front Ultrasonic Receiver
#define FRONT_TRIG_PIN 7  // from schematic
#define FRONT_ECHO_PIN 6  // from schematic
SR04 front_ultrasonic = SR04(FRONT_ECHO_PIN, FRONT_TRIG_PIN);                   

// Rear Ultrasonic Receiver
#define REAR_TRIG_PIN 5
#define REAR_ECHO_PIN 4
SR04 rear_ultrasonic = SR04(REAR_ECHO_PIN, REAR_TRIG_PIN);

// One range for all ultrasonics
#define ULTRASONIC_RANGE_THRESHOLD 40  // stop if measured range is below this, cm

////////////// Motor Control Definitions //////////////
#define MOTOR_POWER_PIN  10            // Green
#define H_BRIDGE_PIN1    12            // Yellow - Relay board IN1
#define H_BRIDGE_PIN2    11            // Orange - Relay board IN2

////////////// IR Receiver Control Definitions //////////////
#define IR_RECEIVER_PIN  2     // Signal Pin of IR receiver to Arduino Digital Pin 11
IRrecv irrecv(IR_RECEIVER_PIN);       // create instance of 'irrecv'

////////////// Buzzer Definitions //////////////
// Buzzer tones
#define DO  262  // C
#define RE  294  // D
#define ME  330  // E
#define FA  349  // F
#define SO  392  // G
#define LA  440  // A
#define SI  494  // B
#define DO2 524  // C
#define BUZZER_TONE RE //set this to your desired tone
#define BUZZER_PIN 8 //Not necessarily a PWM pin, any digital pin will work!!! 
#define TONE_WAIT  0 //Duration for tone to run.  Zero = Don't stop

/////////////////////////////
//////// FUNCTIONS //////////
/////////////////////////////

/*
 * set_desired_velocity()
 * 
 * This function is the only place where velocity_command is set.
 *   It accepts an integer velocity command in mm/s.  This desired
 *   velocity is applied in each main program loop.
 * 
 * Arguments:
 *   newVel - mm/s - Set desired velocity to this.  
 */
void set_desired_velocity(int newVel)
{
  #if VERBOSE
  Serial.print("RoboNex.set_desired_velocity :: Setting Desired Velocity from ");
         Serial.print(desired.velocity); Serial.print(" to "); Serial.println(newVel);
  #endif
  desired.velocity = newVel;
}

/*
 * set_backup_beep(bool beep)
 * 
 * Set the status of backup beeper.  This is applied in every program cycle.
 */
void set_backup_beep(bool setting)
{
  #if VERBOSE
  Serial.print("RoboNex.set_backup_beep :: Setting Backup Beep to "); Serial.println(setting);
  #endif
  desired.beeper_on = setting;
}

/*
 *  apply_backup_beep_io()
 *  
 *  Based on the beeper global variable state, apply the digital IO for the backup beeper.
 *  
 */
void apply_backup_beep_io(bool beeper_on)
{
  static int last_xition_millis = 0;
  static bool beep_on = true;

  // Every n Milliseconds, switch the state of the beeper
  if( (millis() - last_xition_millis) > BEEP_LENGTH_MILLIS )
  {
    if(beep_on)
      beep_on=false; 
    else
      beep_on=true;

    last_xition_millis = millis();
  }
 
  // Apply the correct IO
  if(beeper_on && beep_on)
    NewTone(BUZZER_PIN, BUZZER_TONE);  // Turn on backup beeper
  else
    noNewTone(BUZZER_PIN);    // Turn off backup beeper
}

/*
 * stop()
 * 
 * Power the motor OFF.  This is used when applying the velocity IO.
 * 
 */
void stop()
{
  digitalWrite(MOTOR_POWER_PIN, HIGH);  // Motor power off
}

/*
 * go()
 * 
 * Power the motor ON. This is used in either goForward() or goBack()
 */
void go()
{
  digitalWrite(MOTOR_POWER_PIN, LOW);            //motor power on
}

/* 
 *  goBack()
 *  
 *  Switch the motor H-bridge into backward direction and turn on the motor.
 */
void goBack()
{
  //set relays to forward mode
  digitalWrite(H_BRIDGE_PIN1, HIGH);   // Orange - Relay board IN1
  digitalWrite(H_BRIDGE_PIN2, HIGH);   // Yellow - Relay board IN2
  go();   //motor power on

}

/* 
 *  goForward()
 *  
 *  Switch the motor H-bridge into forward direction and turn on the motor.
 */
void goForward()
{
  // Set relays to forward mode
  digitalWrite(H_BRIDGE_PIN1, LOW);   //sets digital pin 12 off
  digitalWrite(H_BRIDGE_PIN2, LOW);   //sets digital pin 13 off
  // Turn Power on for duration milliseconds
  go();   //motor power on

}

/*
 * set_velocity_io()
 * 
 * Apply the commanded velocity IO to the robot. For the Knex vehicle, 
 *   that's just switching the h-bridge on in that direction.
 *   
 *   Arguments:
 *     v_command - mm/s - signed desired velocity.  
 *                        (Currently only really -1,0,1 is used.)
 */
void set_velocity_io(int v_command)
{
  measured.command_velocity = v_command;
  if(v_command > 0)
    goForward();
  else if (v_command < 0)
    goBack();
  else
    stop();
}

/*
 * translateIR()
 * 
 * Take action on IR remote control inputs.
 * 
 * This function makes changes to global variables, which are applied 
 *   to the digital IO on the regular program cycle.
 *   
 */
void translateIR(decode_results results) // takes action based on IR code received
{

  switch(results.value)

  {
  case 0xFFA25D: Serial.println("POWER"); break;
  case 0xFFE21D: Serial.println("VOL STOP"); break;
  case 0xFF629D: Serial.println("MODE"); break;
  case 0xFF22DD: Serial.println("PAUSE");
                  set_desired_velocity(0);
                  break;
  case 0xFF02FD: Serial.println("FAST BACK");    break;
  case 0xFFC23D: Serial.println("FAST FORWARD");   break;
  case 0xFFE01F: Serial.println("EQ");    break;
  case 0xFFA857: Serial.println("VOL-");
                 set_desired_velocity(-1);
                 break;
  case 0xFF906F: Serial.println("VOL+");
                 set_desired_velocity(1);
                 break;
  case 0xFF9867: Serial.println("RETURN");    break;
  case 0xFFB04F: Serial.println("USB SCAN");    break;
  case 0xFF6897: Serial.println("0");
                 break;
  case 0xFF30CF: Serial.println("1");
                 break;
  case 0xFF18E7: Serial.println("2");
                 break;
  case 0xFF7A85: Serial.println("3");
                 break;
  case 0xFF10EF: Serial.println("4");
                 break;
  case 0xFF38C7: Serial.println("5");
                 break;
  case 0xFF5AA5: Serial.println("6");
                 break;
  case 0xFF42BD: Serial.println("7");
                 break;
  case 0xFF4AB5: Serial.println("8");
                 break;
  case 0xFF52AD: Serial.println("9");
                 break;
  case 0xFFFFFFFF: Serial.println(" REPEAT");break;  

  default: 
    Serial.println(" other button   ");

  }// End Case
} 


/////////////////////////////
////////   SETUP   //////////
/////////////////////////////

void setup() {
  // Setup the debug serial port
  Serial.begin(9600);

  // Put your setup code here, to run once:
  pinMode(H_BRIDGE_PIN1, OUTPUT);          // sets the digital pin 12 as output
  pinMode(H_BRIDGE_PIN2, OUTPUT);          // sets the digital pin 13 as output
  pinMode(MOTOR_POWER_PIN, OUTPUT);        // sets the digital pin 10 as output

  // Enable IR Remote Control Input
  irrecv.enableIRIn(); // Start the receiver

  // Setup the buzzer pin
  pinMode(BUZZER_PIN, OUTPUT);

  // Make sure we are stopped
  stop();

  // Init built-in LED
  pinMode(LED_BUILTIN, OUTPUT);

  Serial.println("Setup Complete.");

}


/////////////////////////////
//////// MAIN LOOP //////////
/////////////////////////////

void loop() {
  unsigned long loop_start_millis = millis();

  static int command_velocity = 0;  // mm/s
  static unsigned long prev_time = 0;  //microseconds


  // Check for remote control commands
  if (irrecv.decode(&measured.results)) // have we received an IR signal?
  {
    translateIR(measured.results); 
    irrecv.resume(); // receive the next value
  }  

  // Read the front ultrasonic sensor
  // todo: look into replacing with non-blocking calls.
  measured.front_range_cm = front_ultrasonic.Distance();

  // read the rear ultrasonic
  measured.rear_range_cm=rear_ultrasonic.Distance();


  // Start with desired velocity, prior to sensor limits
  command_velocity = desired.velocity;

  // Limit Forward Velocity based on front ultrasonic
  if( (measured.front_range_cm < ULTRASONIC_RANGE_THRESHOLD) &&
      (command_velocity > 0))
  {
    command_velocity = 0;
    measured.front_rangefinder_inhibit = true;
  }
  else
  {
    measured.front_rangefinder_inhibit = false;
  }
  
  // Limit reverse velocity based on rear ultrasonic
  if( (measured.rear_range_cm < ULTRASONIC_RANGE_THRESHOLD) &&
           (command_velocity < 0))
  {
    command_velocity = 0;
    measured.rear_rangefinder_inhibit = true;
  }
  else
  {
    measured.rear_rangefinder_inhibit = false;
  }

  // Finally, apply command velocity
  set_velocity_io(command_velocity);

  // Decide whether the backup beep state needs to changed
  if( (command_velocity >= 0) && desired.beeper_on )
    set_backup_beep(false);
  else if( (command_velocity < 0) && !desired.beeper_on )
    set_backup_beep(true);

  // Apply the desired backup beeper state (after evaluating our command velocity)
  apply_backup_beep_io(desired.beeper_on);

  // Indicate whether I'm being blocked by the US with the builtin LED
  if(measured.rear_rangefinder_inhibit || measured.front_rangefinder_inhibit)
    digitalWrite(LED_BUILTIN, HIGH);  // LED on
  else
    digitalWrite(LED_BUILTIN, LOW);   // LED off

  // Dump measured values at end of the program cycle
  #if VERBOSE
  Serial.println("----------------------------------------");
  measured.dump();
  Serial.println("----------------------------------------");
  unsigned long delta_time = micros() - prev_time;
  prev_time = micros();
  Serial.print("Cycle Duration (micros) : "); Serial.println(delta_time);
  #endif

  // Loop Timing
  unsigned long loop_duration = millis() - loop_start_millis;
  int loop_wait = DESIRED_LOOP_DURATION - loop_duration;
  if(loop_wait >= 0)
  {
    delay(loop_wait);
    unsigned int utilization = 100 - ((loop_wait * 100) / DESIRED_LOOP_DURATION);
    Serial.print("% Utilization: "); Serial.println(utilization);
  }
  else
  {
    Serial.println("WARNING: LOOP DURATION EXCEEDED!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
  }
}



