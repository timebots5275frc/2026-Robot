// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shoot;

import java.util.Optional;

import com.revrobotics.spark.SparkBase.ControlType;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.Constants;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.CANDriveSubsystem;
import frc.robot.subsystems.FuelShooter;
import frc.robot.subsystems.Vision.Vision;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class LimelightDistanceShootCommand extends Command {

  private Boolean end = false;

  private Vision vision;
  private FuelShooter fuelShooter;
  private double LLDist = 0;
  private double targetRPM = 0;
  private double testRPM = 0;
  private double allowedError = 25;
  private double robotX = 0;
  private double robotY = 0;
  private Translation3d targetPose;
  private CANDriveSubsystem cds;
  private Boolean commandisFinished = false;

    /** Creates a new LimelightDistanceShootCommand. */
    public LimelightDistanceShootCommand(Vision vision, FuelShooter fuelShooter, CANDriveSubsystem cds) {
      this.vision = vision;
      this.fuelShooter = fuelShooter;
      this.cds = cds;

      addRequirements(vision, fuelShooter);
    }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    Optional<Alliance> alliance = DriverStation.getAlliance();
        targetPose =
            alliance.isPresent() && alliance.get() == Alliance.Blue
                ? VisionConstants.BLUE_HUB_POSE
                : VisionConstants.RED_HUB_POSE;

    if(!vision.hasValidData()) {
      robotX = cds.getPose().getX();
      robotY = cds.getPose().getY();
      testRPM = fuelShooter.calculateRPMFromLimelight(Math.hypot(robotX - targetPose.getX(), robotY - targetPose.getY()));
      fuelShooter.shooterMotorPID.setReference(testRPM ,ControlType.kVelocity);
      return;
    }

    LLDist = vision.AprilTagPosInRobotSpace().magnitude();
    targetRPM = fuelShooter.calculateRPMFromLimelight(LLDist);
    fuelShooter.shooterMotorPID.setReference(targetRPM, ControlType.kVelocity);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    commandisFinished = true;
    // SmartDashboard.putBoolean("LL Shoot is Finished", commandisFinished);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(Math.abs(fuelShooter.shooterMotor1.getEncoder().getVelocity() - targetRPM) < allowedError) return end = true;
    return end;
  }
}
