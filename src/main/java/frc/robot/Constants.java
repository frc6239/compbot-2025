// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import swervelib.math.Matter;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean constants. This
 * class should not be used for any other purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants
{


  public static class ClimberConstants {
    // Units are shows between <>
    // Gearbox ratio of arm motor <unitless>
    public static final int kArmGearBoxRatio = 144;

    public static final double kP = 0.03;

    // The arm rotates from 0 upto 120 degrees
    // Calculate arm rotation distance in rotations <rotations> 
    public static final double kArmRotationDistance = 125.0/360.0;

    // Set position ranges to rotate arm
    // Maximum we can go is 20 degrees above the rotation distance
    public static final double kMaxPosition =(kArmRotationDistance + 20.0/360.0) * kArmGearBoxRatio;

    // Location where arm is deployed
    public static final double kDeployPosition = kArmRotationDistance * kArmGearBoxRatio;

    // Arm initially in bucket.
    // We cannot retract the arm back to zero position once deployed
    // We can only move to safe distance above the bucket as it is a hard stop
    public static final double kMinSafePosition = 40.0/360.0 * kArmGearBoxRatio;

    // Locaiton where frame lifts off the floor on the lift
    public static final double kLiftPosition = 45.0/360.0 * kArmGearBoxRatio;

    // Set initial arm increment
    public static final double kArmIncrement = 10.0 / 360.0 * kArmGearBoxRatio;
    
    // Time takes arm to rotate through range <seconds> 
    public static final double kArmRotationTime = 3.0;
    // Arm revolutions per minute = rotation time / ( 60 * rotation range) <rpm>
    // Note:  even though arm does not rotate one full rotation we need to
    //         include it in calculation since we are calculating revolutions per minute
    public static final double kArmRpm = kArmRotationDistance/kArmRotationTime * 60; //2000.0/kArmGearBoxRatio; // kArmRotationTime / ( 60 * kArmRotationDistance) ;
    // The motor is on the other side of the gear box and runs faster than the arm
    // To calculate motor rpm multiple the arm rpm by the gearbox ratios <rpm>
    public static final double kMotorRpm = kArmRpm * kArmGearBoxRatio;
    public static final double kMotorRpmAcc = kMotorRpm/60;

    public static final int kCANidMotor = 15;
    
  }

  public static final double ROBOT_MASS = (148 - 20.3) * 0.453592; // 32lbs * kg per pound
  public static final Matter CHASSIS    = new Matter(new Translation3d(0, 0, Units.inchesToMeters(8)), ROBOT_MASS);
  public static final double LOOP_TIME  = 0.13; //s, 20ms + 110ms sprk max velocity lag
  public static final double MAX_SPEED  = Units.feetToMeters(11.0);
  // Maximum speed of the robot in meters per second, used to limit acceleration.

//  public static final class AutonConstants
//  {
//
//    public static final PIDConstants TRANSLATION_PID = new PIDConstants(0.7, 0, 0);
//    public static final PIDConstants ANGLE_PID       = new PIDConstants(0.4, 0, 0.01);
//  }

  public static final class DrivebaseConstants
  {

    // Hold time on motor brakes when disabled
    public static final double WHEEL_LOCK_TIME = 10; // seconds
  }

  public static class OperatorConstants
  {
    // Joystick Deadband
    public static final double DEADBAND        = 0.1;
    public static final double LEFT_Y_DEADBAND = 0.1;
    public static final double RIGHT_X_DEADBAND = 0.1;
    public static final double TURN_CONSTANT    = 6;
  }

  public static final class ElevatorConstants {
    // Units are shows between <>
    // Gearbox ratio of arm motor <unitless>
    public static final int kCANIdLeftMotor = 9;
    public static final int kCANIdRightMotor = 10;
    public static final double kGearboxRatio = 15.0;

    public static final int kElevatorGearBoxRatio = 15;
    public static final double kDt = 0.02;
    public static final double kMaxVelocity = 7.5;
    public static final double kMaxAcceleration = 4.0;
    public static final double kP = 0.85;
    public static final double kI = 0.0;
    public static final double kD = 0.0;//0.7;
    public static final double kS = 0.0;//1.1;
    public static final double kG = 0.18;
    public static final double kV = 0.4;//1.3;

    //height of coarl in inches
    public static final double L_2 = 18.5; 
    public static final double L_3 = 33.5;


    //DO NOT CHANGE pOffset
    public static final double pOffset= 8.0;

    //distance traveled
    public static final double position_Home = 0.0;
    public static final double position_L2 = (L_2-pOffset);
    public static final double position_L3 = (L_3-pOffset);
    

    
}


public static final class OuttakeConstants {
  // Units are shows between <>
  // Gearbox ratio of arm motor <unitless>
  public static final int kCANIdLeftMotor = 11;
  public static final int kCANIdRightMotor = 12;
  public static final double kCoralShootSpeed = 0.75;
  public static final double kCoralFeedSpeed = 0.06;

}

}