// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shoot;

import com.revrobotics.spark.SparkBase.ControlType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.FuelShooter;
import frc.robot.subsystems.FuelShooter.FuelShooterState;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ChargeMotor extends Command {
  /** Creates a new ChargeMotor. */


  FuelShooter shooter;
  double targetRPM;

  

  int allowedError = 20;


  public ChargeMotor(FuelShooter shooter, double RPM) {
   // this.vision = vision;
    this.shooter = shooter;
    this.targetRPM = -RPM /* shooter.getShooterRPMMult()*/;
    

   // addRequirements(vision);
    addRequirements(shooter);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
      shooter.SetShooterState(FuelShooterState.CHARGEMOTOR);
        
      shooter.shooterMotorPID.setReference(targetRPM, ControlType.kVelocity);
       
      
      SmartDashboard.putBoolean("Charging Motor", true);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // SmartDashboard.putNumber("Encoder Velocity", shooter.getMotor().getEncoder().getVelocity());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
   
    if(interrupted) {
      shooter.SetShooterState(FuelShooterState.NONE);
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() { 
    if (targetRPM == 0) {
      return false;
    }
     SmartDashboard.putBoolean("Charging Motor", true);
    
    if (shooter.getMotor().getEncoder().getVelocity() >= targetRPM - allowedError && shooter.getMotor().getEncoder().getVelocity() <= targetRPM + allowedError) {
      SmartDashboard.putBoolean("Charging Motor", false);
      return true;
    } 
    return false;
    
  }
}
