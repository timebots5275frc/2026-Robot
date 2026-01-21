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
   
  }

    public static final class DriveConstants {
    // Motor controller IDs for drivetrain motors
    public static final int LEFT_LEADER_ID = 5  ;
    public static final int LEFT_FOLLOWER_ID = 2;
    public static final int RIGHT_LEADER_ID = 3;
    public static final int RIGHT_FOLLOWER_ID = 4;

    // Current limit for drivetrain motors. 60A is a reasonable maximum to reduce
    // likelihood of tripping breakers or damaging CIM motors
    public static final int DRIVE_MOTOR_CURRENT_LIMIT = 60;

    public static final double WHEEL_RADIUS = 2.0 * 0.0254; // meters * 0.98
    public static final double WHEEL_CIRCUMFERENCE = 2.0 * Math.PI * WHEEL_RADIUS; // meters/revolution

    public static final double MAX_DRIVE_SPEED = 3.5; // meters/second
    public static final double MAX_STEER_RATE = .5; // rotations/second of a wheel for steer.
    public static final double MAX_TWIST_RATE = .6 * 2.0 * Math.PI; // radians/second of the robot rotation.
    public static final double CONTROLLER_TWIST_RATE = 2; // constant turn rate for using controller

    // #region <Misc CAN IDs>
      public static final int PIGEON_IMU_ID = 9;
      public static final int PIGEON_2_ID = 9;
    // #endregion
  }
  public static final class MathConstants
  {
    public static final double INCH_TO_METER = 0.0254;
  }

  public static final class VisionConstants {
    public static final boolean ENABLE_LIMELIGHT_LIGHT_ON_ENABLE = true;
    public static final int VALUES_TO_AVERAGE = 3;
    public static final double TARGET_POSITION_ALLOWED_ERROR = .1; // meters
    public static final double LIMELIGHT_X_OFFSET = 0.31773; // meters

    public static final double LIMELIGHT_DATA_WAIT_TIME = .5; // seconds

    public static final double MAX_AMP_TARGET_DISTANCE = 3;
    public static final Vector2 AMP_VISION_DRIVE_TARGET = new Vector2(.07, .47);

    public static enum AprilTagData
    {
      //Red Hub
      Hub_Right_Far_Red(2, "Hub Right Far", DriverStation.Alliance.Red),
      Hub_Back_Right_Red(3, "Hub Back Right", DriverStation.Alliance.Red),
      Hub_Back_Left_Red(4, "Hub Back Left", DriverStation.Alliance.Red, 0, 16.6193978),
      Hub_Left_Far_Red(5, "Hub Left Far", DriverStation.Alliance.Red, -2.7389074, 14.778355),
      Hub_Left_Close_Red(8, "Hub Left Close", DriverStation.Alliance.Red),
      Hub_Front_Left_Red(9, "Hub Front Left", DriverStation.Alliance.Red),
      Hub_Front_Right_Red(10, "Hub Front Right", DriverStation.Alliance.Red),
      Hub_Right_Close_Red(11, "Hub Right Close", DriverStation.Alliance.Red),
      //Blue Hub
      Hub_Right_Far_Blue(18, "Hub Right Far", DriverStation.Alliance.Blue),
      Hub_Back_Right_Blue(19, "Hub Back Right", DriverStation.Alliance.Blue),
      Hub_Back_Left_Blue(20, "Hub Back Left", DriverStation.Alliance.Blue, 0, 16.6193978),
      Hub_Left_Far_Blue(21, "Hub Left Far", DriverStation.Alliance.Blue, -2.7389074, 14.778355),
      Hub_Left_Close_Blue(24, "Hub Left Close", DriverStation.Alliance.Blue),
      Hub_Front_Left_Blue(25, "Hub Front Left", DriverStation.Alliance.Blue),
      Hub_Front_Right_Blue(26, "Hub Front Right", DriverStation.Alliance.Blue),
      Hub_Right_Close_Blue(27, "Hub Right Close", DriverStation.Alliance.Blue);

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
          // case 1: return Trench_Right_Neutral_Zone;
          case 2: return Hub_Right_Far_Red;
          case 3: return Hub_Back_Right_Red;
          case 4: return Hub_Back_Left_Red;
          case 5: return Hub_Left_Far_Red;
          // case 6: return Trench_Left_Neutral_Zone;
          // case 7: return Trench_Left_Alliance_Zone;
          case 8: return Hub_Left_Close_Red;
          case 9: return Hub_Front_Left_Red;
          case 10: return Hub_Front_Right_Red;
          case 11: return Hub_Right_Close_Red;
          // case 12: return Trench_Right_Alliance_Zone;
          // case 13: return Loading_Station_Right;
          // case 14: return Loading_Station_Left;
          // case 15: return Tower_Right;
          // case 16: return Tower_Left;
          // case 17: return Trench_Right_Neutral_Zone;
          case 18: return Hub_Right_Far_Blue;
          case 19: return Hub_Back_Right_Blue;
          case 20: return Hub_Back_Left_Blue;
          case 21: return Hub_Left_Far_Blue;
          // case 22: return ba_amplifier;
          // case 23: return ba_speaker_main;
          case 24: return Hub_Left_Close_Blue;
          case 25: return Hub_Front_Left_Blue;
          case 26: return Hub_Front_Right_Blue;
          case 27: return Hub_Right_Close_Blue;
          // case 28: return ra_core_opp_scoring_table;
          // case 29: return ra_core_mid;
          // case 30: return ba_core_mid;
          // case 31: return ba_core_opp_scoring_table;
          // case 32: return ba_core_scoring_table;
          default: return null;
        }
      }

      // public static boolean isSpeakerTag(AprilTagData tag) { return tag == AprilTagData.Hub_Back_Left || tag == AprilTagData.Hub_Back_Right; }
      // public static boolean isAmpTag(AprilTagData tag) { return tag == AprilTagData.Hub_Front_Left || tag == AprilTagData.Hub_Front_Right; }
    }
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

      public static final PID motor1PID = new PID(0,0,0);
      public static final PID motor2PID = new PID(0,0,0);
      public static final PID motor3PID = new PID(0,0,0);

      public static final int IntakeSpeed = 2000;
    }

    public static final class ClimberConstants{
      public static final int CLIMBER_MOTOR_1_ID = 20;
      public static final int CLIMBER_MOTOR_2_ID = 21;
    }

}
