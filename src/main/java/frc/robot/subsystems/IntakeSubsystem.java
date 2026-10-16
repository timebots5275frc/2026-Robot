// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Constants;




public class IntakeSubsystem extends SubsystemBase {
  /** Creates a new IntakeSubsystem. */

  //Declare the variables like motors and other things

  public SparkFlex intakeMotor1;
  public SparkClosedLoopController intakeMotorPID1;

  public SparkMax intakeMotor2;
  public SparkClosedLoopController intakeMotorPID2;

  public IntakeState state;

  //instantiate the motors and other things and do the limits

  public IntakeSubsystem() {

    intakeMotor2 = new SparkMax(Constants.FuelShooterConstants.INTAKE_MOTOR_2_ID,SparkLowLevel.MotorType.kBrushless);
    Constants.FuelShooterConstants.INTAKE_MOTOR_2_PID.setFreeLimit(Constants.FuelShooterConstants.INTAKE_FREE_LIMIT2);
    Constants.FuelShooterConstants.INTAKE_MOTOR_2_PID.setStallLimit(Constants.FuelShooterConstants.INTAKE_STALL_LIMIT2);
    Constants.FuelShooterConstants.INTAKE_MOTOR_2_PID.setSparkMaxPID(intakeMotor2,ResetMode.kResetSafeParameters,PersistMode.kPersistParameters);
    intakeMotorPID2 = intakeMotor2.getClosedLoopController();

    intakeMotor1 = new SparkFlex(Constants.FuelShooterConstants.INTAKE_MOTOR_1_ID,SparkLowLevel.MotorType.kBrushless);
    Constants.FuelShooterConstants.INTAKE_MOTOR_1_PID.setFreeLimit(Constants.FuelShooterConstants.INTAKE_FREE_LIMIT1);
    Constants.FuelShooterConstants.INTAKE_MOTOR_1_PID.setStallLimit(Constants.FuelShooterConstants.INTAKE_STALL_LIMIT1);
    Constants.FuelShooterConstants.INTAKE_MOTOR_1_PID.setSparkFlexPID(intakeMotor1,ResetMode.kResetSafeParameters,PersistMode.kPersistParameters);
    intakeMotorPID2 = intakeMotor1.getClosedLoopController();
  }

  //this is needed to keep track of what state the intake is in. 
  //you need this to have a switch case.

  public enum IntakeState {
    INTAKE, 
    OUTAKE, 
    NONE,
    SUCK,
    FEED, 
    BLOW;
  }

  //you need a getter and setter because Nathan said you do

  public void SetIntakeState(IntakeState state){
    this.state = state;
    UpdateIntakeState(state);
  }

  //this is the switch case it is like a bunch of if statements but written in a cleaner way

  public void UpdateIntakeState(IntakeState state){
    switch(state){
      case INTAKE:intakeMotorPID1.setReference(-Constants.FuelShooterConstants.INTAKESPEED1, ControlType.kVelocity);
                  intakeMotorPID2.setReference(-Constants.FuelShooterConstants.INTAKESPEED2, ControlType.kVelocity);
      break;
      case OUTAKE:intakeMotorPID1.setReference(Constants.FuelShooterConstants.INTAKESPEED1, ControlType.kVelocity);
                  intakeMotorPID2.setReference(Constants.FuelShooterConstants.INTAKESPEED2, ControlType.kVelocity);
      break;
      case NONE:intakeMotorPID1.setReference(0, ControlType.kVelocity);
                intakeMotorPID2.setReference(0, ControlType.kVelocity);
      break;
      case SUCK:intakeMotorPID1.setReference(-Constants.FuelShooterConstants.INTAKESPEED1, ControlType.kVelocity);
                intakeMotorPID2.setReference(Constants.FuelShooterConstants.INTAKESPEED2, ControlType.kVelocity);
      break;
      case FEED:intakeMotorPID1.setReference(Constants.FuelShooterConstants.FEEDSPEED, ControlType.kVelocity);
                intakeMotorPID2.setReference(-Constants.FuelShooterConstants.FEEDSPEED, ControlType.kVelocity);
      break;
      case BLOW:intakeMotorPID1.setReference(-Constants.FuelShooterConstants.INTAKESPEED1, ControlType.kVelocity);
                intakeMotorPID2.setReference(Constants.FuelShooterConstants.INTAKESPEED2, ControlType.kVelocity);
      break;
    }
  }

  //this is the getter that goes with the setter

  public IntakeState getIntakeState(){
    return state;
  }

  //this is the code that runs every 20 milliseconds

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
