// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.CustomTypes.PID;
import frc.robot.CustomTypes.SwerveCanIDs;
import frc.robot.CustomTypes.SwerveModuleLocations;
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
  public static class OperatorConstants 
  {
    public static final int kDriverControllerPort = 0;
  }

  
//guh
  public static class ButtonConstants
  {
    //elevator
    // public static final int ELEVATOR_L1 = 1;
    public static final int ELEVATOR_RESET = 1;
    public static final int ELEVATOR_INTAKE = 6;
    public static final int ELEVATOR_L2 = 7;
    public static final int ELEVATOR_L3 = 8;
    public static final int ELEVATOR_L4 = 3;
    // public static final int ELEVATOR_DRIVE = 1;

    //coral intake
    public static final int CORAL_NONE = 6;
    public static final int CORAL_INTAKE = 5;
    public static final int CORAL_OUTTAKE_L1 = 3;
    public static final int CORAL_OUTTAKE_L2_TO_L3 = 4;
    public static final int CORAL_OUTTAKE_L4 = 9;

    //algae intake
    public static final int ALGAE_INTAKE_INTAKE = 1;
    public static final int ALGAE_INTAKE_OUTTAKE = 1;
    public static final int ALGAE_INTAKE_NONE = 1;

    //algae pivot
    public static final int ALGAE_PIVOT_DRIVE = 1;
    public static final int ALGAE_PIVOT_GROUND = 1;
    public static final int ALGAE_PIVOT_REEF = 1;
    public static final int ALGAE_PIVOT_SHOOT = 1;
  }

  

 

    public static final class DriveConstants {
      // Final Robot Constants
      // 11.875 for 29" side (front)
      // 12.375 for 30" side (side)
        public static final SwerveModuleLocations Robot2025SwerveLocations = new SwerveModuleLocations(
            11.875  * MathConstants.INCH_TO_METER, // LEFT_FRONT_WHEEL_X
            12.375  * MathConstants.INCH_TO_METER,   // LEFT_FRONT_WHEEL_Y
            11.875   * MathConstants.INCH_TO_METER, // RIGHT_FRONT_WHEEL_X
            -12.375 * MathConstants.INCH_TO_METER,   // RIGHT_FRONT_WHEEL_Y
            -11.875  * MathConstants.INCH_TO_METER, // RIGHT_REAR_WHEEL_X
            -12.375 * MathConstants.INCH_TO_METER,   // RIGHT_REAR_WHEEL_Y
            -11.875  * MathConstants.INCH_TO_METER, // LEFT_REAR_WHEEL_X
            11.375  * MathConstants.INCH_TO_METER    // LEFT_REAR_WHEEL_Y
        ); 
        // in case the autofill doesnt show, the can ids go as follows.
        // L/R F/B D/S M for left/right front/back drive/steer motor. it goes in order of lf,rf,lr,rr with drive first 
    
        public static final SwerveCanIDs Robot2025SwerveCAN = new SwerveCanIDs(
          10, 
            20, 
            11, 
            21, 
            13, 
            23, 
            12, 
            22, 
            30, 
            31, 
            33, 
            32
           ); 


          // Test Robot Constants
          public static final SwerveCanIDs AdrianBotSwerveCAN = new SwerveCanIDs(
            10, 
            20, 
            11, 
            21, 
            13, 
            23, 
            12, 
            22, 
            30, 
            31, 
            33, 
            32
           ); 

        public static final SwerveModuleLocations AdrianBotSwerveLocations = new SwerveModuleLocations(
            12.375   * MathConstants.INCH_TO_METER, // LEFT_FRONT_WHEEL_X
            9.375  * MathConstants.INCH_TO_METER,   // LEFT_FRONT_WHEEL_Y
            12.375   * MathConstants.INCH_TO_METER, // RIGHT_FRONT_WHEEL_X
            -9.375 * MathConstants.INCH_TO_METER,   // RIGHT_FRONT_WHEEL_Y
            -12.375  * MathConstants.INCH_TO_METER, // RIGHT_REAR_WHEEL_X
            -9.375 * MathConstants.INCH_TO_METER,   // RIGHT_REAR_WHEEL_Y
            -12.375  * MathConstants.INCH_TO_METER, // LEFT_REAR_WHEEL_X
            9.375  * MathConstants.INCH_TO_METER    // LEFT_REAR_WHEEL_Y
          ); 
      public static final SwerveCanIDs ROBOT_SWERVE_CAN = Robot2025SwerveCAN;
      public static final SwerveModuleLocations ROBOT_SWERVE_LOCATIONS = Robot2025SwerveLocations;
      public static final double WHEEL_RADIUS = 2.0 * MathConstants.INCH_TO_METER; // meters * 0.98
      public static final double WHEEL_CIRCUMFERENCE = 2.0 * Math.PI * WHEEL_RADIUS; // meters/revolution
      public static final double MAX_DRIVE_SPEED = 5.5; // meters/second
      public static final double MAX_STEER_RATE = .5; // rotations/second of a wheel for steer.
      public static final double MAX_TWIST_RATE = .6 * 2.0 * Math.PI; // radians/second of the robot rotation.
      public static final double CONTROLLER_TWIST_RATE = 2; // constant turn rate for using controller
      public static final int PIGEON_2_ID = 9;
      public static final double DRIVE_GEAR_RATIO = 1.0/5.9;//.169;
      public static final double STEER_GEAR_RATIO = .05333333333;
      public static final PID PID_SparkMax_Steer = new PID(0.0004,0,0.0001,0,0.0001);
      //public static final PID PID_SparkMax_Steer = new PID(0.0005,0.00000018,0.001,0,0.0001);
      public static final PID PID_Encoder_Steer = new PID(15, 10, .1);
      public static final PID PID_SparkFlex_Drive = new PID(0.0003,0.0000001,0.005,0,0.0002);
      public static final double AUTO_ODOMETRY_DRIVE_MIN_SPEED = .1;
      public static final double AUTO_ODOMETRY_DRIVE_MAX_SPEED = 2;

      public static final double AUTO_ODOMETRY_DRIVE_TARGET_ALLOWED_ERROR = .1; // in meters
      public static final double AUTO_ODOMETRY_DRIVE_SLOWDOWN_DISTANCE = .6; // in meters
  }
  public static final class MathConstants
  {
    public static final double INCH_TO_METER = 0.0254;
  }

  public static enum AprilTagData
    {
      ba_source_left(1, "Source left", DriverStation.Alliance.Blue),
      ba_source_right(2, "Source right", DriverStation.Alliance.Blue),
      ra_speaker_aux(3, "Speaker auxillary", DriverStation.Alliance.Red),
      ra_speaker_main(4, "Speaker main", DriverStation.Alliance.Red, 0, 16.6193978),
      ra_amplifier(5, "Amplifier", DriverStation.Alliance.Red, -2.7389074, 14.778355),
      ba_amplifier(6, "Amplifier", DriverStation.Alliance.Blue, -2.7389074, 1.858645),
      ba_speaker_main(7, "Speaker main", DriverStation.Alliance.Blue, 0, 0),
      ba_speaker_aux(8, "Speaker auxillary", DriverStation.Alliance.Blue),
      ra_source_right(9, "Source right", DriverStation.Alliance.Red),
      ra_source_left(10, "Source left", DriverStation.Alliance.Red),
      ra_core_scoring_table(11, "Core scoring table side", DriverStation.Alliance.Red),
      ra_core_opp_scoring_table(12, "Core opposite scoring table side", DriverStation.Alliance.Red),
      ra_core_mid(13, "Core middle side", DriverStation.Alliance.Red),
      ba_core_mid(14, "Core middle side", DriverStation.Alliance.Blue),
      ba_core_opp_scoring_table(15, "Core opposite scoring table side", DriverStation.Alliance.Blue),
      ba_core_scoring_table(16, "Core scoring table side", DriverStation.Alliance.Blue);

      public final int id;
      public final String name;
      public final DriverStation.Alliance alliance;
      private AprilTagData(int id, String name, DriverStation.Alliance alliance)
      {
        this.id = id;
        this.name = name;
        this.alliance = alliance;
      }

      private AprilTagData(int id, String name, DriverStation.Alliance alliance, double x, double y)
      {
        this.id = id;
        this.name = name;
        this.alliance = alliance;
      }

      @Override
      public String toString()
      {
        return (alliance == DriverStation.Alliance.Blue ? "Blue " : "Red ") + name + "[" + id + "]";
      }

      public static AprilTagData getTag(int id) {
        switch(id){
          case 1: return ba_source_left;
          case 2: return ba_source_right;
          case 3: return ra_speaker_aux;
          case 4: return ra_speaker_main;
          case 5: return ra_amplifier;
          case 6: return ba_amplifier;
          case 7: return ba_speaker_main;
          case 8: return ba_speaker_aux;
          case 9: return ra_source_right;
          case 10: return ra_source_left;
          case 11: return ra_core_scoring_table;
          case 12: return ra_core_opp_scoring_table;
          case 13: return ra_core_mid;
          case 14: return ba_core_mid;
          case 15: return ba_core_opp_scoring_table;
          case 16: return ba_core_scoring_table;
          default: return null;
        }
      }

      public static boolean isSpeakerTag(AprilTagData tag) { return tag == AprilTagData.ra_speaker_main || tag == AprilTagData.ba_speaker_main; }
      public static boolean isAmpTag(AprilTagData tag) { return tag == AprilTagData.ra_amplifier || tag == AprilTagData.ba_amplifier; }
    }

    public static final class ControllerConstants 
    {
      public static final int DRIVER_STICK_CHANNEL = 0;
      public static final int AUX_STICK_CHANNEL    = 1;
      public static final double DEADZONE_DRIVE    = 0.1;
      public static final double DEADZONE_STEER    = 0.3;
    }

    


  }
