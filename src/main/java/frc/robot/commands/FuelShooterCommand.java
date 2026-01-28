// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.revrobotics.spark.SparkBase.ControlType;

import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.FuelShooter;
import frc.robot.subsystems.FuelShooter.FuelShooterState;
import frc.robot.subsystems.Vision.Vision;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class FuelShooterCommand extends Command {

  
  private Vision vision;

  private boolean end = false;

  

  private FuelShooter shooter;
  private FuelShooterState shooterState;
  

  public FuelShooterCommand(FuelShooter fs, Vision vision, FuelShooterState shooterState) {
    
    
    // Use addRequirements() here to declare subsystem dependencies.

    this.vision = vision;
    this.shooterState = shooterState;
    this.shooter = fs;

    addRequirements(shooter);
     addRequirements(vision);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    shooter.SetShooterState(shooterState);
  }

  
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() { 
    switch(shooterState){
      case CHARGEMOTOR: 
      break;
      case NONE:
      break;
      case FEEDBALL: // move feeding motors
        break;
      case LOCKTOHUB: // lock on
        break;
    }
      
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

    // shooter.setShooterRPM(0);
    // shooter.setState(Shooter.NONE);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return end;
  }
    
}
