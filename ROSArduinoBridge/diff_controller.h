/* Functions and type-defs for PID control.

   Taken mostly from Mike Ferguson's ArbotiX code which lives at:
   
   http://vanadium-ros-pkg.googlecode.com/svn/trunk/arbotix/
*/
#include "Arduino.h"
/* PID setpoint info For a Motor */
typedef struct {
  double TargetTicksPerFrame;    // target speed in ticks per frame

  /*
  * Using previous input (PrevVelocityInTicksPerFrame) instead of PrevError to avoid derivative kick,
  * see http://brettbeauregard.com/blog/2011/04/improving-the-beginner%E2%80%99s-pid-derivative-kick/
  */
  int CurrentVelocityInTicksPerFrame;                // last input
  int PrevVelocityInTicksPerFrame;                // last input
  //int PrevErr;                   // last error

  /*
  * Using integrated term (ITerm) instead of integrated error (Ierror),
  * to allow tuning changes,
  * see http://brettbeauregard.com/blog/2011/04/improving-the-beginner%E2%80%99s-pid-tuning-changes/
  */
  //int Ierror;
  int ITerm;                    //integrated term

  long output;                    // last motor setting
}
SetPointInfo;

SetPointInfo leftPID, rightPID;

/* PID Parameters */
int Kp = 20;
int Kd = 12;
int Ki = 0;
int Ko = 50;

unsigned char moving = 0; // is the base in motion?

/*
* Initialize PID variables to zero to prevent startup spikes
* when turning PID on to start moving
* In particular, assign both Encoder and PrevEnc the current encoder value
* See http://brettbeauregard.com/blog/2011/04/improving-the-beginner%E2%80%99s-pid-initialization/
* Note that the assumption here is that PID is only turned on
* when going from stop to moving, that's why we can init everything on zero.
*/
void resetPID(){
  Serial.print("Reset PID: ");
  leftPID.TargetTicksPerFrame = 0.0;
  leftPID.output = 0;
  leftPID.CurrentVelocityInTicksPerFrame = 0;
  leftPID.PrevVelocityInTicksPerFrame = 0;
  leftPID.ITerm = 0;

  rightPID.TargetTicksPerFrame = 0.0;
  rightPID.output = 0;
  leftPID.CurrentVelocityInTicksPerFrame = 0;
  rightPID.PrevVelocityInTicksPerFrame = 0;
  rightPID.ITerm = 0;
}

/* PID routine to compute the next motor commands */
void doPID(SetPointInfo * p) {
  long Perror;
  long output;

  //Perror = p->TargetTicksPerFrame - (p->Encoder - p->PrevEnc);
  Perror = p->TargetTicksPerFrame - p->CurrentVelocityInTicksPerFrame;

  Serial.print("Target: ");
  Serial.println(p->TargetTicksPerFrame);
  Serial.print("error: ");
  Serial.println(Perror);
  Serial.print("Velocity: ");
  Serial.println(p->CurrentVelocityInTicksPerFrame);
  Serial.println("============================");

  /*
  * Avoid derivative kick and allow tuning changes,
  * see http://brettbeauregard.com/blog/2011/04/improving-the-beginner%E2%80%99s-pid-derivative-kick/
  * see http://brettbeauregard.com/blog/2011/04/improving-the-beginner%E2%80%99s-pid-tuning-changes/
  */
  //output = (Kp * Perror + Kd * (Perror - p->PrevErr) + Ki * p->Ierror) / Ko;
  // p->PrevErr = Perror;
  output = (Kp * Perror - Kd * (p->CurrentVelocityInTicksPerFrame - p->PrevVelocityInTicksPerFrame) + p->ITerm) / Ko;

  output += p->output;
  // Accumulate Integral error *or* Limit output.
  // Stop accumulating when output saturates
  if (output >= MAX_PWM)
    output = MAX_PWM;
  else if (output <= -MAX_PWM)
    output = -MAX_PWM;
  else
  /*
  * allow turning changes, see http://brettbeauregard.com/blog/2011/04/improving-the-beginner%E2%80%99s-pid-tuning-changes/
  */
  p->ITerm += Ki * Perror;

  p->output = output;
  p->PrevVelocityInTicksPerFrame = p->CurrentVelocityInTicksPerFrame;
}

/* Read the encoder values and call the PID routine */
void updatePID() {
  /* Read the encoders */
  leftPID.CurrentVelocityInTicksPerFrame = readEncoder(LEFT);
  rightPID.CurrentVelocityInTicksPerFrame = readEncoder(RIGHT);
  
  /* If we're not moving there is nothing more to do */
  if (!moving){
    /*
    * Reset PIDs once, to prevent startup spikes,
    * see http://brettbeauregard.com/blog/2011/04/improving-the-beginner%E2%80%99s-pid-initialization/
    * PrevVelocityInTicksPerFrame is considered a good proxy to detect
    * whether reset has already happened
    */
    if (leftPID.PrevVelocityInTicksPerFrame != 0 || rightPID.PrevVelocityInTicksPerFrame != 0) resetPID();
    return;
  }

  /* Compute PID update for each motor */
  doPID(&rightPID);
  doPID(&leftPID);

  /* Set the motor speeds accordingly */
  setMotorSpeeds(leftPID.output, rightPID.output);
}

