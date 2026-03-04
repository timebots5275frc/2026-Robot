// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CANDriveSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class DistanceDrive extends Command {
    private  CANDriveSubsystem drive;
    private  double distanceMeters;
    private  double speed;

    public DistanceDrive(CANDriveSubsystem drive, double distanceMeters, double speed) {
        this.drive = drive;
        this.distanceMeters = distanceMeters;
        this.speed = speed;
        addRequirements(drive);
    }

    @Override
    public void initialize() {
        drive.resetEncoders();
        
    }

    @Override
    public void execute() {
        

         double direction = Math.signum(distanceMeters);
         drive.driveArcade(Math.abs(speed) * direction, 0);
        
    }

    @Override
    public boolean isFinished() {
        return Math.abs(drive.getAverageDistanceMeters()) >= Math.abs(distanceMeters);
    }

    @Override
    public void end(boolean interrupted) {
        drive.driveArcade(0, 0);
    }
}