// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Constants;

import java.util.List;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.CustomTypes.PID;
// import frc.robot.CustomTypes.SwerveCanIDs;
// import frc.robot.CustomTypes.SwerveModuleLocations;
import frc.robot.CustomTypes.Math.Vector2;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants 
{

  public static final double TELESCOPE_PIVOT_GEAR_RATIO = 81;
  public static final double INTAKE_PIVOT_ROTATIONS_PER_DEGREE = (270.0/360.0)/*(270/(double)360)*/;
  // 300*(36/24) = gear ratio
  public static final double ALGAE_INTAKE_PIVOT_ROTATIONS_PER_DEGREE =1;//((double)(150*36/(double)24)/360);
  public static final double CLIMBER_ROTATIONS_PER_DEGREE = 125/(double)360;


  //Operator Constants
  public static final class OperatorConstants {
    // Port constants for driver and operator controllers. These should match the
    // values in the Joystick tab of the Driver Station software
    public static final int DRIVER_CONTROLLER_PORT = 0;
    public static final int kDriverControllerPort = 0;

    public static final int OPERATOR_CONTROLLER_PORT = 1;

    // This value is multiplied by the joystick value when rotating the robot to
    // help avoid turning too fast and beign difficult to control
    public static final double DRIVE_SCALING = .7;
    public static final double ROTATION_SCALING = .8;
  }
    
  

  
//guh
  public static class ButtonConstants
  {
    //9,10,11,12
    public static final int INCREASE_LEFT_MOTOR_BUTTON_ID = 10;
    public static final int INCREASE_RIGHT_MOTOR_BUTTON_ID = 12;
    public static final int DECREASE_LEFT_MOTOR_BUTTON_ID = 9;
    public static final int DECREASE_RIGHT_MOTOR_BUTTON_ID = 11;

    public static final int FLIP_FRONT_BUTTON_ID = 2;
    public static final int SHOOT_LIMELIGHT_BUTTON_ID = 1;
    public static final int SHOOT_NO_LIMELIGHT_BUTTON_ID = 8;
    public static final int INTAKE_BUTTON_ID = 5;
    public static final int OUTTAKE_BUTTON_ID = 7;
    public static final int STOP_SHOOTER_BUTTON_ID = 3;
    public static final int SHAKE_ROBOT_BUTTON_ID = 12;
    public static final int STOP_INTAKE_BUTTON_ID = 4; 
    public static final int FEED_INTAKE_BUTTON_ID = 9;
  }

    public static final class DriveConstants {

      public static final double DEAD_BAND_DRIVE = 0.02;
       public static final double DEAD_BAND_STEER = 0.05;

      // Motor controller IDs for drivetrain motors
      public static final int LEFT_LEADER_ID = 5  ;
      public static final int LEFT_FOLLOWER_ID = 2;
      public static final int RIGHT_LEADER_ID = 3;
      public static final int RIGHT_FOLLOWER_ID = 4;

      // Current limit for drivetrain motors. 60A is a reasonable maximum to reduce
      // likelihood of tripping breakers or damaging CIM motors
      public static final int DRIVE_MOTOR_STALL_LIMIT = 60;
      public static final int DRIVE_MOTOR_FREE_LIMIT = 35;
      public static final int DRIVE_MOTOR_LIMIT_RPM = 5000;
      public static final int DRIVE_MOTOR_RAMP_RATE = 10;

      public static final double WHEEL_RADIUS = 2.0 * 0.0254; // meters * 0.98
      public static final double WHEEL_CIRCUMFERENCE = 2.0 * Math.PI * WHEEL_RADIUS; // meters/revolution

      public static final double MAX_DRIVE_SPEED = 3.5; // meters/second
      public static final double MAX_STEER_RATE = .5; // rotations/second of a wheel for steer.
      public static final double MAX_TWIST_RATE = .6 * 2.0 * Math.PI; // radians/second of the robot rotation.
      public static final double CONTROLLER_TWIST_RATE = 2; // constant turn rate for using controller

      //DistanceDrive
      public static final double GEAR_RATIO = 10.71;

      public static final double METERS_PER_MOTOR_ROTATION = WHEEL_CIRCUMFERENCE / GEAR_RATIO;

    // #region <Misc CAN IDs>
      public static final int PIGEON_IMU_ID = 9;
      public static final int PIGEON_2_ID = 9;
    // #endregion

    public static final double TRACK_WIDTH = 21.5 * MathConstants.INCH_TO_METER;

    public static final double CHASSILENGTH = 26.5 * MathConstants.INCH_TO_METER ; 
  }
  public static final class MathConstants
  {
    public static final double INCH_TO_METER = 0.0254;
  }

    public static final class JoystickConstants
    {
      public static final double JOY_X_RATE_LIMIT = 3;
      public static final double JOY_TURN_RATE_LIMIT = 5;

      public static final double JOY_INPUT_VELOCITY_MULT = 1;
      public static final double JOY_INPUT_ROTATION_VELOCITY_MULT = .25;
    }

    public static final class ControllerConstants 
    {
      public static final int DRIVER_STICK_CHANNEL = 0;
      public static final int AUX_STICK_CHANNEL    = 1;
      public static final double DEADZONE_DRIVE    = 0.1;
      public static final double DEADZONE_STEER    = 0.3;
    }

    public static final class FuelShooterConstants{
      public static final int INTAKE_MOTOR_1_ID = 10;
      public static final int INTAKE_MOTOR_2_ID = 11; 
      public static final int SHOOTER_MOTOR_ID = 12;
      public static final int SHOOTER_MOTOR_ID2 = 13;

      public static final int SHOOTER_FREE_LIMIT = 35;
      public static final int SHOOTER_STALL_LIMIT = 60;

      public static final int INTAKE_FREE_LIMIT1 = 35;
      public static final int INTAKE_STALL_LIMIT1 = 60;

      public static final int INTAKE_FREE_LIMIT2 = 35;
      public static final int INTAKE_STALL_LIMIT2 = 60;

      public static final int MOTOR_SPEED_NONE = 0;

      public static final PID INTAKE_MOTOR_2_PID = new PID(0.0001,0.0,0.0, 0.00, 0.0015, 0, 0);
      public static final PID INTAKE_MOTOR_1_PID = new PID(0.00015,0.0,0.0, 0.00, 0.002, 0, 0);
      public static final PID SHOOTER_MOTOR_PID = new PID(0.00052,0.0000001,0.0, 0.00, 0.0015, 0, 0);
      public static final int INTAKESPEED1 = 2500;
      public static final int INTAKESPEED2 = 5000;
      public static final double FEEDSPEED = 2000; 
      public static final int DEFAULT_SHOOTER_RPM = 3500;
    }

    public static final class CalculateShooterRpmConstants{
      public static final double GRAVITY = 9.8;
      public static final double WHEEL_RADIUS = 0.1016; //4 inch wheel radius 
      public static final double CAMERA_HEIGHT = 20.5 * 0.0254; //.75
      public static final double CAMERA_OFFSET = 0.5588;
      public static final double TARGET_HEIGHT = 1.8542;
      public static final double MOUNTING_ANGLE = 20;
      public static final double SHOOTER_ANGLE = 75;
      public static final double RPM_FUDGE_FACTOR = 1.0;
    } 

    public static final class ClimbConstants {

        public static final int MOTOR1_ID = 20;
        public static final int MOTOR2_ID = 21;

        public static final PID CLIMB_MOTOR_PID = new PID(0.0,0.0 , 0.0); //two motors controlled the same
        public static final int STALL_LIMIT = 30;
        public static final int FREE_LIMIT = 25;

        public static final double DRIVE = 0;
        public static final double L1 = 0;
        public static final double RESET = 3;

    }

}
