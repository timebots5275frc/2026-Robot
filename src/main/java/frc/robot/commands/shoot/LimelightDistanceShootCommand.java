// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shoot;

import java.util.Optional;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.revrobotics.spark.SparkBase.ControlType;

import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.estimator.PoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.Constants;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.CANDriveSubsystem;
import frc.robot.subsystems.FuelShooter;
import frc.robot.subsystems.FuelShooter.FuelShooterState;
import frc.robot.subsystems.Vision.Vision;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class LimelightDistanceShootCommand extends Command {

  private Boolean end = false;

  private Vision vision;
  private FuelShooter fuelShooter;
  private double LLDist = 0;
  private double targetRPM = 0;
  private double allowedError = 25;
  private double robotX = 0;
  private double robotY = 0;
  private double robotHeading = 0;
  private Translation3d targetPose;
  private DifferentialDriveKinematics ddk;
  private DifferentialDrivePoseEstimator ddpe;

    /** Creates a new LimelightDistanceShootCommand. */
    public LimelightDistanceShootCommand(Vision vision, FuelShooter fuelShooter) {
      this.vision = vision;
      this.fuelShooter = fuelShooter;

      addRequirements(vision, fuelShooter);
    }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
     ddk = new DifferentialDriveKinematics(.025);//not correct track width
     ddpe = new DifferentialDrivePoseEstimator(ddk, vision.gyro.getRotation2d(), (CANDriveSubsystem.leftLeader.getEncoder().getPosition() * 2 * Math.PI * 4) , (CANDriveSubsystem.rightLeader.getEncoder().getPosition() * 2 * Math.PI * 4), new Pose2d());
    vision.gyro.reset();
    vision.setUsingLimelight(true);

    Optional<Alliance> alliance = DriverStation.getAlliance();
        targetPose =
            alliance.isPresent() && alliance.get() == Alliance.Blue
                ? VisionConstants.BLUE_HUB_POSE
                : VisionConstants.RED_HUB_POSE;

    if(!vision.hasValidData()) {

      fuelShooter.calculateRPMFromLimelight(Math.hypot(robotX - targetPose.getX(), robotY - targetPose.getY()));
      // fuelShooter.shooterMotorPID.setReference(Constants.FuelShooterConstants.DEFAULT_SHOOTER_RPM ,ControlType.kVelocity);
      end = true;
    }

    LLDist = vision.AprilTagPosInRobotSpace().magnitude();
    targetRPM = fuelShooter.calculateRPMFromLimelight(LLDist);

    fuelShooter.shooterMotorPID.setReference(targetRPM, ControlType.kVelocity);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    if(!vision.hasValidData()){
      if(robotX == 0) return; //so robot doesnt start calculating its pose until it creates and origin from tag
      ddpe.update(vision.gyro.getRotation2d(), (CANDriveSubsystem.leftLeader.getEncoder().getPosition() * 2 * Math.PI * 4), (CANDriveSubsystem.rightLeader.getEncoder().getPosition() * 2 * Math.PI * 4));
      robotX = ddpe.getEstimatedPosition().getX();
      robotY = ddpe.getEstimatedPosition().getY();
    }

    int aTag = vision.AprilTagID();

    if(vision.hasValidData()) {
      ddpe.resetPosition(new Rotation2d(0,0), 0, 0, new Pose2d(0,0,new Rotation2d(0,0)));
      vision.CalculateRobotPositionInFieldSpace();
      //averages vision and gyro data
      robotX = vision.RobotPosInFieldSpace().x + VisionConstants.AprilTagFieldConstants.TAGS.get(aTag).pose.getX();
      robotY = vision.RobotPosInFieldSpace().y + VisionConstants.AprilTagFieldConstants.TAGS.get(aTag).pose.getY();
      robotHeading = vision.gyro.getYaw().getValueAsDouble();
    }

    SmartDashboard.putNumber("aTag", aTag);
    SmartDashboard.putNumber("RobotX", robotX);
    SmartDashboard.putNumber("RobotY", robotY);
    SmartDashboard.putNumber("RobotHeading", robotHeading);
    SmartDashboard.putNumber("targetRPM", targetRPM);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    vision.setUsingLimelight(false);
    vision.ClearAprilTagData();
    // fuelShooter.SetShooterState(FuelShooterState.NONE);
    end = false;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(Math.abs(fuelShooter.shooterMotor1.getEncoder().getVelocity() - targetRPM) < allowedError) return end = true;
    return end;
  }
}
