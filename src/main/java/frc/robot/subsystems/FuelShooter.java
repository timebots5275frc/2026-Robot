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
  private double shooterRPM = 4000;
  private boolean feedingFuel = false;
  public double dx;

  public enum Shooter{
    INTAKE,
    OUTTAKE,
    SHOOT,
    NONE;
  }

  public FuelShooter() {
    
    IntakeMotor1 = new SparkMax(Constants.FuelShooterConstants.INTAKE_MOTOR_1_ID, SparkLowLevel.MotorType.kBrushless);
    Constants.FuelShooterConstants.motor1PID.setSparkMaxPID(IntakeMotor1, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    IntakePIDOne = IntakeMotor1.getClosedLoopController();

    IntakeMotor2 = new SparkMax(Constants.FuelShooterConstants.INTAKE_MOTOR_2_ID, SparkLowLevel.MotorType.kBrushless);
    Constants.FuelShooterConstants.motor2PID.setSparkMaxPID(IntakeMotor2, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    IntakePIDTwo = IntakeMotor2.getClosedLoopController();

    shooterMotor = new SparkMax(Constants.FuelShooterConstants.SHOOTER_MOTOR_ID, SparkLowLevel.MotorType.kBrushless);
    Constants.FuelShooterConstants.motor3PID.setSparkMaxPID(shooterMotor, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    ShooterPID = shooterMotor.getClosedLoopController();
    
  }

  public void Shooter(){

    switch (shooterState) {
      case NONE: feedingFuel = false;
                 ShooterPID.setReference(0, ControlType.kCurrent);
      break;
      case INTAKE: feedingFuel = true;

      break;
      case OUTTAKE: IntakePIDOne.setReference(-Constants.FuelShooterConstants.IntakeSpeed, ControlType.kVelocity);
                    IntakePIDTwo.setReference(-Constants.FuelShooterConstants.IntakeSpeed, ControlType.kVelocity);
                    ShooterPID.setReference(0, ControlType.kVelocity);
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
      IntakePIDOne.setReference(0, ControlType.kCurrent);
      IntakePIDTwo.setReference(0, ControlType.kCurrent);
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

  //does the calculations in HubAimCommand
  public double calculateRPMFromLimelight(
        double ty,
        double cameraHeight,
        double targetHeight,
        double mountingAngle,
        double thetaDeg,
        double wheelRadius,
        double g) {

    // Convert angles to radians
    double totalAngleRad = Math.toRadians(ty + mountingAngle);
    double thetaRad = Math.toRadians(thetaDeg);

    // Height difference
    double deltaH = targetHeight - cameraHeight;

    // Horizontal distance to target
    double dx = deltaH / Math.tan(totalAngleRad);

    // Projectile motion denominator
    double denominator =
            2.0 * Math.pow(Math.cos(thetaRad), 2.0)
            * (dx * Math.tan(thetaRad) - deltaH);

    if (denominator <= 0.0) {
        throw new IllegalArgumentException("Target unreachable at this angle.");
    }

    // Required exit velocity
    double v0 = Math.sqrt(g * dx * dx / denominator);

    // Convert linear velocity to wheel RPM
    return (60.0 / (2.0 * Math.PI * wheelRadius)) * v0;
}

  @Override
  public void periodic() {

  }
}
