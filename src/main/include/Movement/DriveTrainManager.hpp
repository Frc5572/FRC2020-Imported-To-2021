#ifndef DRIVE_TRAIN_HPP
#define DRIVE_TRAIN_HPP

#include <iostream>
#include <ctre/Phoenix.h>
#include "Vision/VisionManager.hpp"
#include "Movement/ControllerManager.hpp"
#include <frc/SpeedControllerGroup.h>
#include "rev/CANSparkMax.h"
#include "AHRS.h"
#include <frc/PIDController.h>
#include <frc/SmartDashboard/SmartDashboard.h>


/* CAN ID layout for drive train from a top view

          Front of Robot
         
         |--------------|
         |              |
      1  | M1        M2 | 2
      3  | M3        M4 | 3
      5  | M5        M6 | 6
         |              |
         |              |
         |--------------|

          Back Of Robot
*/

class DriveTrain
{
public:

DriveTrain(
  rev::CANSparkMax &TopLeftMotor     ,
  rev::CANSparkMax &TopRightMotor    ,
  rev::CANSparkMax &MiddleLeft       ,
  rev::CANSparkMax &MiddleRight      ,
  rev::CANSparkMax &BottomLeftMotor  , 
  rev::CANSparkMax &BottomRightMotor ,
  FRC5572Controller &Driver          ,
  VisionManager &VisionManager      ,
  AHRS &ahrs                         
  );

~DriveTrain();
void Calucate();
void TestRPM();
void Test();
void AutoPID();
void RunPID();
void InitPID();
void Drive();
void LowerAmps();
void Aim();

  int AutoSelection = 0;

  double Power = 0, Distance = 0, rpm = 0, leftRPM = 0, rightRPM = 0, SP = 0;

  double kP = 6e-5, kI = 1e-6, kD = 0, kMaxOutput = 1.0, kMinOutput = -1.0 ,SetP = 700;

  // kIz = 0, kFF = 0.000015,

  VisionManager* LimeLight;

  double disX, L, R;

  frc::PIDController *m_pidControllerLeft;
  frc::PIDController *m_pidControllerRight;

  frc::SpeedControllerGroup* LeftMotors;
  frc::SpeedControllerGroup* RightMotors;
  frc::SpeedControllerGroup* TempRightMotors;
  frc::SpeedControllerGroup* TempLeftMotors;
  
  FRC5572Controller* Driver;

  rev::CANSparkMax* TopLeftMotor;
  rev::CANSparkMax* TopRightMotor;

  rev::CANSparkMax* MiddleLeft;
  rev::CANSparkMax* MiddleRight;  
  
  rev::CANSparkMax* BottomLeftMotor; 
  rev::CANSparkMax* BottomRightMotor;

  rev::CANEncoder* leftMotorEncoder    ;
  rev::CANEncoder* rightMotorEncoder   ;
  // rev::CANEncoder* MiddleLeftMotorEncoder ;
  // rev::CANEncoder* MiddleRightMotorEncoder;
  // rev::CANEncoder* BottomLeftMotorEncoder ;
  // rev::CANEncoder* BottomRightMotorEncoder;

  AHRS* ahrs;
  };  
#endif