// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Constants;
import frc.robot.CustomTypes.PID;

//intake/outtake(not a shooter) mechanism that pivots on a single wrist and is a single flywheel mechanism
/*
 * neo motor to pivit the wrist and for the flywheel
*/

public class LearnSubsystem extends SubsystemBase {
  /** Creates a new LearnSubsystem. */
  wristState wState;
  flywheelState fState;
  
  SparkMax wristMotor;
  SparkClosedLoopController wristMotorPID;
  SparkMax flywheelMotor;
  SparkClosedLoopController flywheelMotorPID;

  //constants
  public final PID WRIST_MOTOR_PID = new PID(0, 0, 0);
  public final int INTAKE_POS = 0;
  public final int OUTTAKE_POS = 0;
  public final int DRIVE_POS = 0;
  public final PID FLYWHEEL_MOTOR_PID = new PID(0, 0, 0);
  public final int INTAKE = 0;
  public final int OUTTAKE = 0;
  public final int NONE = 0;
  


  public LearnSubsystem() {
    wristMotor = new SparkMax(0, MotorType.kBrushless);
    WRIST_MOTOR_PID.setSparkMaxPID(wristMotor, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    flywheelMotor = new SparkMax(0, MotorType.kBrushless);
    FLYWHEEL_MOTOR_PID.setSparkMaxPID(flywheelMotor, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  public enum wristState{
    INTAKE_POS,
    OUTTAKE_POS,
    DRIVE_POS,
  }

  public void updateWristState(wristState wState){
    this.wState = wState;
    setWristState(wState);
  }

  public void setWristState(wristState wState){
    switch(wState){
      case INTAKE_POS: wristMotorPID.setReference(INTAKE_POS, ControlType.kPosition);
      break;
      case OUTTAKE_POS: wristMotorPID.setReference(OUTTAKE_POS, ControlType.kPosition);
      break;
      case DRIVE_POS: wristMotorPID.setReference(DRIVE_POS, ControlType.kPosition);
      break;
    }
  }

  public enum flywheelState{
    INTAKE, 
    OUTTAKE,
    NONE,
  }

  public void updateFlywheelState(flywheelState fState){
    this.fState = fState;
    setFlywheelState(fState);
  }

  public void setFlywheelState(flywheelState fState){
    switch(fState){
      case INTAKE: flywheelMotorPID.setReference(INTAKE, ControlType.kVelocity);
      break;
      case OUTTAKE: flywheelMotorPID.setReference(OUTTAKE, ControlType.kVelocity);
      break;
      case NONE: flywheelMotorPID.setReference(NONE, ControlType.kVelocity);
      break;
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  //understand what you want this program to do
  //instantiate motors (set pids, set current limit, etc...)(persist peramiters)
  //*enum
  //*set mode function (logic goes here)
  //*update mode functions
}
