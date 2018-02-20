/*reference:
 * https://www.arduino.cc/reference/en/language/functions/digital-io/digitalwrite/
 * our board:
 * https://smile.amazon.com/gp/product/B074WMHLQ4/ref=oh_aui_search_detailpage?ie=UTF8&psc=1
 * language reference:
 * https://www.arduino.cc/reference/en/
 * 
 * IDEAS: 
 *  Use the onboard LED to blink when we're stopped due to the ultrasonics
 *  Add steering
 *  Beep the backup beeper
 *  Add a rubber band launcher: http://www.instructables.com/id/Rubber-Band-Gun-Using-an-Arduino/
 *  Unify the desired state into one context data structure
 *  Unify the sensor inputs into one status data structure
 *  Allow dumping of the contexts for debugging
 *  Add police Lights / reverse lights for the bus
 *  Allow control of lights from the remote
 *  Add control via wifi / webpage (maybe using raspberry pi?)
 *  Make wiring more solid (create a whitewire board for relays and power supply)
 *  Put this code up on GitHub ASAP!!!
 *  Get some unit tests
 *  Create a data structure of ultrasonics so I can add them easily
 *  Factor out the velocity control logic from the beeper behavior from the velocity IO function.
*/

#include "NewTone.h"
#include "IRremote.h"
#include "SR04.h"


/////////////////////////////
////////DEFINITIONS//////////
/////////////////////////////

////////////// Ultrasonics Definitions //////////////

// Front Ultrasonic Receiver
#define FRONT_TRIG_PIN 7
#define FRONT_ECHO_PIN 6
SR04 front_ultrasonic = SR04(FRONT_ECHO_PIN, FRONT_TRIG_PIN);
long front_range_cm;                     // measured range, cm

// Rear Ultrasonic Receiver
#define REAR_TRIG_PIN 5
#define REAR_ECHO_PIN 4
SR04 rear_ultrasonic = SR04(REAR_ECHO_PIN, REAR_TRIG_PIN);
long rear_range_cm;                      // measured range, cm

// One range for all ultrasonics
#define RANGE_THRESHOLD 40               // stop if measured range is below this, cm

////////////// Motor Control Definitions //////////////

int go_duration = 0;           // Number of millisections to go when I press + or -.
                               // If set to zero, go forever.
int velocity_command = 0;      // Negative = backward; positive=forward

#define MOTOR_POWER_PIN  10            // Green
#define H_BRIDGE_PIN1    12            // Orange - Relay board IN1
#define H_BRIDGE_PIN2    13            // Yellow - Relay board IN2


////////////// IR Receiver Control Definitions //////////////
#define IR_RECEIVER_PIN 11     // Signal Pin of IR receiver to Arduino Digital Pin 11
IRrecv irrecv(IR_RECEIVER_PIN);       // create instance of 'irrecv'
decode_results results;        // create instance of 'decode_results'

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
#define BUZZER_PIN 8 //Not necessarily a PWM pin, any digital pin will work!!! 
#define TONE_WAIT  0 //Duration for tone to run.  Zero = Don't stop
bool backup_beeper_on = false;

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
  velocity_command = newVel;
}

/*
 * set_backup_beep(bool beep)
 * 
 * Set the status of backup beeper.  This is applied in every program cycle.
 */
void set_backup_beep(bool setting)
{
  backup_beeper_on = setting;
}

/*
 *  apply_backup_beep_io()
 *  
 *  Based on the beeper global variable state, apply the digital IO for the backup beeper.
 *  
 */
void apply_backup_beep_io()
{
 if(backup_beeper_on)
 {
   NewTone(BUZZER_PIN, RE);  // Turn on backup beeper
 }
 else
 {
   noNewTone(BUZZER_PIN);    // Turn off backup beeper
 }
}

/*
 * stop()
 * 
 * Power the motor OFF.  This is used when applying the velocity IO.
 * 
 */
void stop()
{
  Serial.println("Stopping Motor");
  digitalWrite(MOTOR_POWER_PIN, HIGH);           // Motor power off
  set_backup_beep(false);            // Turn backup buzzer off when I stop
}

/*
 * go()
 * 
 * Power the motor ON. This is used in either goForward() or goBack()
 */
void go()
{
  Serial.println("starting Motor");
  digitalWrite(MOTOR_POWER_PIN, LOW);            //motor power on
}

/* 
 *  goBack()
 *  
 *  Switch the motor H-bridge into backward direction and turn on the motor.
 *    If a duration is provided, run for a certain duration.
 *    
 *  If global variable go_duration is zero, go forever.
 */
void goBack()
{
  //set relays to forward mode
  Serial.println("Setting relays to backward mode");
  digitalWrite(H_BRIDGE_PIN1, HIGH);   // Orange - Relay board IN1
  digitalWrite(H_BRIDGE_PIN2, HIGH);   // Yellow - Relay board IN2

  Serial.println("Setting motor to Reverse.");
  go();   //motor power on

  set_backup_beep(true);  // Turn on backup beeper

   //Turn Power on for duration milliseconds
  if(go_duration > 0)
  {
    delay(go_duration);      //wait for a certain amount of time
    stop();
  }
}

/* 
 *  goForward()
 *  
 *  Switch the motor H-bridge into forward direction and turn on the motor.
 *    If a duration is provided, run for a certain duration.
 *    
 *  If global variable go_duration is zero, go forever.
 */
void goForward()
{
  // Set relays to forward mode
  Serial.println("Setting relays to forward mode");
  digitalWrite(H_BRIDGE_PIN1, LOW);   //sets digital pin 12 off
  digitalWrite(H_BRIDGE_PIN2, LOW);   //sets digital pin 13 off

  // Turn Power on for duration milliseconds
  Serial.println("Turning on forward Motor");
  go();   //motor power on

  // Turn off backup beeper
  set_backup_beep(false);

  // Run for only a certain duration if set
  if(go_duration > 0)
  {     
    delay(go_duration);      //wait for a certain amount of time
    stop();
  }
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
  if(v_command > 0)
  {
    goForward();
  } 
  else if (v_command < 0)
  {
    goBack();
  }
  else
  {
    stop();
  }
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
void translateIR() // takes action based on IR code received
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
                 go_duration = 0;
                 break;
  case 0xFF30CF: Serial.println("1");
                 go_duration = 1000;
                 break;
  case 0xFF18E7: Serial.println("2");
                 go_duration = 2000;
                 break;
  case 0xFF7A85: Serial.println("3");
                 go_duration = 3000;
                 break;
  case 0xFF10EF: Serial.println("4");
                 go_duration = 4000;
                 break;
  case 0xFF38C7: Serial.println("5");
                 go_duration = 5000;
                 break;
  case 0xFF5AA5: Serial.println("6");
                 go_duration = 6000;
                 break;
  case 0xFF42BD: Serial.println("7");
                 go_duration = 7000;
                 break;
  case 0xFF4AB5: Serial.println("8");
                 go_duration = 8000;
                 break;
  case 0xFF52AD: Serial.println("9");
                 go_duration = 9000;
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
  pinMode(H_BRIDGE_PIN1, OUTPUT);          // sets the digital pin 13 as output
  pinMode(H_BRIDGE_PIN2, OUTPUT);          // sets the digital pin 12 as output
  pinMode(MOTOR_POWER_PIN, OUTPUT);          // sets the digital pin 11 as output

  // Enable IR Remote Control Input
  Serial.println("IR Receiver Button Decode"); 
  irrecv.enableIRIn(); // Start the receiver

  // Setup the buzzer pin
  pinMode(BUZZER_PIN, OUTPUT);

  // Make sure we are stopped
  stop();

  // Wait a half-second to start
  delay(500);
}


/////////////////////////////
//////// MAIN LOOP //////////
/////////////////////////////

void loop() {

  // Check for remote control Commands
  if (irrecv.decode(&results)) // have we received an IR signal?
  {
    translateIR(); 
    irrecv.resume(); // receive the next value
  }  

  // read the front ultrasonic
  front_range_cm = front_ultrasonic.Distance();
  Serial.print("Front range: ");
  Serial.print(front_range_cm);
  Serial.println("cm");

  // read the rear ultrasonic
  rear_range_cm=rear_ultrasonic.Distance();
  Serial.print("Rear range: ");
  Serial.print(rear_range_cm);
  Serial.println("cm");


  // Apply the desired backup beeper state
  apply_backup_beep_io();

  // Apply speeds, using ultrasonics to stop
  if( (front_range_cm < RANGE_THRESHOLD) &&
      (velocity_command > 0))
  {
    Serial.println("forward stopped due to ultrasonics");
    set_velocity_io(0);
  }
  else if( (rear_range_cm < RANGE_THRESHOLD) &&
           (velocity_command < 0))
  {
    Serial.println("reverse stopped due to ultrasonics");
    set_velocity_io(0);
  }
  else
  {
    set_velocity_io(velocity_command);
  }

  //Not adding a delay here, but todo: make rate consistent.
  delay(1000);
  
}



