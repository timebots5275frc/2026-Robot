// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shoot;

import java.util.Optional;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.revrobotics.spark.SparkBase.ControlType;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.VisionConstants;
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
  private double allowedError = 50;
  private double robotX = 0;
  private double robotY = 0;
  // private double dx = 0;
  // private double dy = 0;
  private double robotHeading = 0;
  // private Pigeon2 gyro;
  private Translation3d targetPose;
  
    /** Creates a new LimelightDistanceShootCommand. */
    public LimelightDistanceShootCommand(Vision vision, FuelShooter fuelShooter) {
      this.vision = vision;
      this.fuelShooter = fuelShooter;

      addRequirements(vision, fuelShooter);
    }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    vision.setUsingLimelight(true);

    Optional<Alliance> alliance = DriverStation.getAlliance();
        targetPose =
            alliance.isPresent() && alliance.get() == Alliance.Blue
                ? VisionConstants.BLUE_HUB_POSE
                : VisionConstants.RED_HUB_POSE;

    if(!vision.hasValidData()) {
      // potentially make it so LL dist is calcualted based off of the robots last known location on the field
      // pros
      //  -if being defended and we saw the aTag before they cut off our vision distance is still accurate
      // cons
      //  - if the last known tag was the tunnel or something else its going to be way off
      //  - potentially not enough time to develop and test
      // solutions to cons
      //  - could just press shoot no limelight button for 3500 rpm const
      //  - rest of monday, all of tuesday, all of wednesday

      //make relative to hub
      // LLDist = Math.hypot(robotX, robotY);
      // fuelShooter.calculateRPMFromLimelight(Math.hypot(targetPose.getX() - robotX, targetPose.getY() - robotY));

      fuelShooter.shooterMotorPID.setReference(-3500,ControlType.kVelocity);
      end = true;
    }

    LLDist = vision.AprilTagPosInRobotSpace().magnitude();
    targetRPM = fuelShooter.calculateRPMFromLimelight(LLDist);

    fuelShooter.shooterMotorPID.setReference(-targetRPM, ControlType.kVelocity);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    //use gyro to help with tracking robot pose

    int aTag = vision.AprilTagID();

    if(vision.hasValidData()) {
      vision.CalculateRobotPositionInFieldSpace();
      robotX = vision.RobotPosInFieldSpace().x + VisionConstants.AprilTagFieldConstants.TAGS.get(aTag).pose.getX();
      robotY = vision.RobotPosInFieldSpace().y + VisionConstants.AprilTagFieldConstants.TAGS.get(aTag).pose.getY();
      robotHeading = vision.gyro.getYaw().getValueAsDouble();
    }

    SmartDashboard.putNumber("aTag", aTag);
    SmartDashboard.putNumber("RobotX", robotX);
    SmartDashboard.putNumber("RobotY", robotY);
    SmartDashboard.putNumber("RobotHeading", robotHeading);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    vision.setUsingLimelight(false);
    vision.ClearAprilTagData();
    fuelShooter.SetShooterState(FuelShooterState.NONE);
    end = false;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(Math.abs(fuelShooter.shooterMotor1.getEncoder().getVelocity() - targetRPM) < allowedError) return end = true;
    return end;
  }
}
