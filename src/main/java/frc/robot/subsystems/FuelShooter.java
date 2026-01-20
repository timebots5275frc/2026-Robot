// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;

import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class FuelShooter extends SubsystemBase {
  /** Creates a new FuelErector. */

  SparkMax IntakeMotor1;
  private SparkClosedLoopController IntakePIDOne;
  SparkMax IntakeMotor2;
  private SparkClosedLoopController IntakePIDTwo;
  SparkMax shooterMotor;
  private SparkClosedLoopController ShooterPID;
  private AbsoluteEncoder ShooterEncoder;
  
  private Shooter shooterState = Shooter.NONE;

  private double shooterRPM = 0;

  private boolean feedingFuel = false;

  public enum Shooter{
    INTAKE,
    OUTTAKE,
    SHOOT,
    NONE;
  }

  public FuelShooter() {
    
    IntakeMotor1 = new SparkMax(Constants.FuelShooterConstants.motor1ID, SparkLowLevel.MotorType.kBrushless);
    Constants.FuelShooterConstants.motor1PID.setSparkMaxPID(IntakeMotor1, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    IntakePIDOne = IntakeMotor1.getClosedLoopController();

    IntakeMotor2 = new SparkMax(Constants.FuelShooterConstants.motor2ID, SparkLowLevel.MotorType.kBrushless);
    Constants.FuelShooterConstants.motor2PID.setSparkMaxPID(IntakeMotor2, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    IntakePIDTwo = IntakeMotor2.getClosedLoopController();

    shooterMotor = new SparkMax(Constants.FuelShooterConstants.motor3ID, SparkLowLevel.MotorType.kBrushless);
    Constants.FuelShooterConstants.motor3PID.setSparkMaxPID(shooterMotor, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    ShooterPID = shooterMotor.getClosedLoopController();
    
  }

  public void Shooter(){

    switch (shooterState) {
      case INTAKE: IntakePIDOne.setReference(Constants.FuelShooterConstants.IntakeSpeed, ControlType.kVelocity);
                   IntakePIDTwo.setReference(Constants.FuelShooterConstants.IntakeSpeed, ControlType.kVelocity);
                   ShooterPID.setReference(0, ControlType.kVelocity);

      break;
      case NONE: IntakePIDOne.setReference(0, ControlType.kVelocity);
                 IntakePIDTwo.setReference(0, ControlType.kVelocity);
                 ShooterPID.setReference(0, ControlType.kVelocity);
      break;
      case OUTTAKE: IntakePIDOne.setReference(-Constants.FuelShooterConstants.IntakeSpeed, ControlType.kVelocity);
                    IntakePIDTwo.setReference(-Constants.FuelShooterConstants.IntakeSpeed, ControlType.kVelocity);
                    ShooterPID.setReference(-Constants.FuelShooterConstants.IntakeSpeed, ControlType.kVelocity);
      break;
      case SHOOT: IntakePIDOne.setReference(0, ControlType.kVelocity);
                  IntakePIDTwo.setReference(0, ControlType.kVelocity);
                  ShooterPID.setReference(shooterRPM, ControlType.kVelocity);
        break;
      
    }

  }

  public void feedFuel() {
    if (feedingFuel == true) return;
      IntakePIDOne.setReference(Constants.FuelShooterConstants.IntakeSpeed, ControlType.kVelocity);
      IntakePIDTwo.setReference(-Constants.FuelShooterConstants.IntakeSpeed, ControlType.kVelocity);
      feedingFuel = true;
    }

  public void stopFeedingFuel() {
    if (feedingFuel == false) return;
      IntakePIDOne.setReference(0, ControlType.kVelocity);
      IntakePIDTwo.setReference(0, ControlType.kVelocity);
      feedingFuel = false;
    }

  

  public void setState(Shooter state) {
    shooterState = state;
    Shooter();
  }

  public void setShooterRPM(double rpm) {
    shooterRPM = rpm;
    Shooter();
  } 

  public boolean shooterAtTargetRPM() {
      if(shooterMotor.getEncoder().getVelocity() == shooterRPM && shooterRPM != 0) {
        return true;
      }
      return false;
  }

  @Override
  public void periodic() {
    // ShooterEncoder.getVelocity();
    // SmartDashboard.putNumber("Shooter vel.",ShooterEncoder);
  }
}
