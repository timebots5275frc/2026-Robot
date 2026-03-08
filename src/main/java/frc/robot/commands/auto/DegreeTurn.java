// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CANDriveSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class DegreeTurn extends Command {
  CANDriveSubsystem drive;
  double turnDegrees;
  boolean clockwise;
  double speed;
  /** Creates a new DegreeTurn. */
  public DegreeTurn(CANDriveSubsystem drive, double turnDegrees, boolean clockwise, double speed) {
    this.drive = drive;
    this.turnDegrees = turnDegrees;
    this.clockwise = clockwise;
    this.speed = speed;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    drive.resetEncoders();
    
     
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    drive.driveArcade(0, speed * (clockwise ? -1:1)); 
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drive.driveArcade(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Math.abs(drive.degreesTurned()) >= Math.abs(turnDegrees);
  }
}
