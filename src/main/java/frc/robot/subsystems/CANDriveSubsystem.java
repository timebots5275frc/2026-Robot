 // Copyright (c) FIRST and other WPILib contributors.
 // Open Source Software; you can modify and/or share it under the terms of
 // the WPILib BSD license file in the root directory of this project.

 package frc.robot.subsystems;

 import com.revrobotics.spark.SparkBase.PersistMode;
 import com.revrobotics.spark.SparkBase.ResetMode;
 import com.revrobotics.spark.SparkLowLevel.MotorType;

import static frc.robot.Constants.Constants.DriveConstants.*;

import java.util.Optional;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.revrobotics.spark.SparkFlex;
 import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Constants;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.Vision.LimelightHelpers;
import frc.robot.subsystems.Vision.Vision;

 public class CANDriveSubsystem extends SubsystemBase {
   public static SparkFlex leftLeader;
   private SparkFlex leftFollower;
   public static  SparkFlex rightLeader;
   private  SparkFlex rightFollower;
   private Vision vision;
   private Pigeon2 gyro;
   private DifferentialDriveKinematics ddk;
   private DifferentialDrivePoseEstimator ddpe;
   private Translation3d targetPose;

   private final DifferentialDrive drive;

  public CANDriveSubsystem(Vision vision) {
    this.vision = vision;

    // create brushed motors for drive
    leftLeader = new SparkFlex(LEFT_LEADER_ID, MotorType.kBrushless);
    leftFollower = new SparkFlex(LEFT_FOLLOWER_ID, MotorType.kBrushless);
    rightLeader = new SparkFlex(RIGHT_LEADER_ID, MotorType.kBrushless);
    rightFollower = new SparkFlex(RIGHT_FOLLOWER_ID, MotorType.kBrushless);

    // set up differential drive class
     drive = new DifferentialDrive(leftLeader, rightLeader);

     // Set can timeout. Because this project only sets parameters once on
     // construction, the timeout can be long without blocking robot operation. Code
     // which sets or gets parameters during operation may need a shorter timeout.
    //  leftLeader.setCANTimeout(250);
    //  rightLeader.setCANTimeout(250);
    //  leftFollower.setCANTimeout(250);
    //  rightFollower.setCANTimeout(250);

     // Create the configuration to apply to motors. Voltage compensation
     // helps the robot perform more similarly on different
     // battery voltages (at the cost of a little bit of top speed on a fully charged
     // battery). The current limit helps prevent tripping
     // breakers.
     SparkFlexConfig config = new SparkFlexConfig();
     config.idleMode(IdleMode.kBrake);
     config.voltageCompensation(12);
     config.smartCurrentLimit(Constants.DriveConstants.DRIVE_MOTOR_STALL_LIMIT, 
                              Constants.DriveConstants.DRIVE_MOTOR_FREE_LIMIT
                              ); //Constants.DriveConstants.DRIVE_MOTOR_LIMIT_RPM

   config.closedLoopRampRate(Constants.DriveConstants.DRIVE_MOTOR_RAMP_RATE);

     // Set configuration to follow each leader and then apply it to corresponding
     // follower. Resetting in case a new controller is swapped
     // in and persisting in case of a controller reset due to breaker trip
     config.follow(leftLeader);
     leftFollower.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
     config.follow(rightLeader);
     rightFollower.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

     // Remove following, then apply config to right leader
     config.disableFollowerMode();
     rightLeader.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
     // Set config to inverted and then apply to left leader. Set Left side inverted
     // so that postive values drive both sides forward
     config.inverted(true);
     leftLeader.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

     ddk = new DifferentialDriveKinematics(.6);

   ddpe = new DifferentialDrivePoseEstimator(
    ddk,
    vision.gyro.getRotation2d(),
    (CANDriveSubsystem.leftLeader.getEncoder().getPosition() * 2 * Math.PI * .0508), 
    (CANDriveSubsystem.rightLeader.getEncoder().getPosition() * 2 * Math.PI * .0508),
    new Pose2d()
);
   }

   public void driveArcade(double xSpeed, double zRotation) {
     drive.arcadeDrive(xSpeed, zRotation);
   }
    public void curvatureDrive(double xSpeed, double zRotation, boolean allowTurnInPlace) {
     drive.curvatureDrive(xSpeed, zRotation, allowTurnInPlace);
   }

    public void resetEncoders() {
      leftLeader.getEncoder().setPosition(0);
      rightLeader.getEncoder().setPosition(0);
    }

public Pose2d getPose() {
    return ddpe.getEstimatedPosition();
}

public void addVisionMeasurement(Pose2d visionPose, double timestamp) {
    ddpe.addVisionMeasurement(visionPose, timestamp);
}

public double getLeftDistance() {
    return leftLeader.getEncoder().getPosition() * 2 * Math.PI * 0.0508;
}

public double getRightDistance() {
    return rightLeader.getEncoder().getPosition() * 2 * Math.PI * 0.0508;
}

    public double getAverageDistanceMeters() {
      double leftMeters = leftLeader.getEncoder().getPosition() * METERS_PER_MOTOR_ROTATION;
      double rightMeters = rightLeader.getEncoder().getPosition() * METERS_PER_MOTOR_ROTATION;
      return (leftMeters + rightMeters) / 2.0;

    }

    public double degreesTurned() {

      double leftRotations = leftLeader.getEncoder().getPosition();

      double rightRotations = rightLeader.getEncoder().getPosition();

      double leftMeters = leftRotations * Constants.DriveConstants.WHEEL_CIRCUMFERENCE;
      double rightMeters = rightRotations * Constants.DriveConstants.WHEEL_CIRCUMFERENCE;

      double angleRad =(rightMeters - leftMeters)/ Constants.DriveConstants.TRACK_WIDTH;

      return Math.toDegrees(angleRad);
    }
    public DifferentialDrive getDifferentialDrive() {
      return drive;
    }

    @Override
   public void periodic() {

    Optional<Alliance> alliance = DriverStation.getAlliance();
        targetPose =
            alliance.isPresent() && alliance.get() == Alliance.Blue
                ? VisionConstants.BLUE_HUB_POSE
                : VisionConstants.RED_HUB_POSE;

    int aTag = vision.AprilTagID();

    if (vision.hasValidData()) {
      double latencyMS = (LimelightHelpers.getLatency_Pipeline("limelight")+LimelightHelpers.getLatency_Capture("limelight"));
      double timeStamp = Timer.getFPGATimestamp() - (latencyMS/1000.0);

      ddpe.update(vision.gyro.getRotation2d(), (getLeftDistance()), (getRightDistance()));
      ddpe.addVisionMeasurement(new Pose2d(vision.RobotPosInFieldSpace().x + VisionConstants.AprilTagFieldConstants.TAGS.get(aTag-1).pose.getX(),vision.RobotPosInFieldSpace().y + VisionConstants.AprilTagFieldConstants.TAGS.get(aTag-1).pose.getY(), vision.gyro.getRotation2d()), timeStamp);
      return;
    }

    ddpe.update(
        vision.gyro.getRotation2d(),
        (getLeftDistance()), 
        (getRightDistance())
    );

    SmartDashboard.putNumber("robotX", ddpe.getEstimatedPosition().getX());
    SmartDashboard.putNumber("robotX", ddpe.getEstimatedPosition().getY());
   }

}
