package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Constants;

public class IntakeSubsystem extends SubsystemBase {

  private SparkFlex intakeMotor1;
  private SparkClosedLoopController intakePID1;
  
  private SparkMax intakeMotor2; // this is spark max dont forget
  private SparkClosedLoopController intakePID2;

  private IntakeState state = IntakeState.NONE;
  
  // 2 big neos

  /** Creates a new IntakeSubsystem. */
  public IntakeSubsystem() {
    intakeMotor2 = new SparkMax(Constants.FuelShooterConstants.INTAKE_MOTOR_2_ID,SparkLowLevel.MotorType.kBrushless);
    Constants.FuelShooterConstants.INTAKE_MOTOR_2_PID.setFreeLimit(Constants.FuelShooterConstants.INTAKE_FREE_LIMIT2);
    Constants.FuelShooterConstants.INTAKE_MOTOR_2_PID.setStallLimit(Constants.FuelShooterConstants.INTAKE_STALL_LIMIT2);
    Constants.FuelShooterConstants.INTAKE_MOTOR_2_PID.setSparkMaxPID(intakeMotor2,ResetMode.kResetSafeParameters,PersistMode.kPersistParameters);
    intakePID2 = intakeMotor2.getClosedLoopController();

    intakeMotor1 = new SparkFlex(Constants.FuelShooterConstants.INTAKE_MOTOR_1_ID, MotorType.kBrushless);
    Constants.FuelShooterConstants.INTAKE_MOTOR_1_PID.setFreeLimit(Constants.FuelShooterConstants.INTAKE_FREE_LIMIT1);
    Constants.FuelShooterConstants.INTAKE_MOTOR_1_PID.setStallLimit(Constants.FuelShooterConstants.INTAKE_STALL_LIMIT1);
    Constants.FuelShooterConstants.INTAKE_MOTOR_1_PID.setSparkFlexPID(intakeMotor1,ResetMode.kResetSafeParameters,PersistMode.kPersistParameters);
    intakePID1 = intakeMotor1.getClosedLoopController();       
    
  }

  public enum IntakeState{
    INTAKE,
    OUTTAKE,
    SUCK,
    NONE,
    FEED,
    BLOW;

  }

  public void setIntakeState(IntakeState state){
    this.state = state;
    switch(state){
      case INTAKE:
        intakePID1.setReference(-Constants.FuelShooterConstants.INTAKESPEED1, ControlType.kVelocity);
        intakePID2.setReference(-Constants.FuelShooterConstants.INTAKESPEED2, ControlType.kVelocity);
        break;
      case OUTTAKE:
        intakePID1.setReference(Constants.FuelShooterConstants.INTAKESPEED1, ControlType.kVelocity);
        intakePID2.setReference(Constants.FuelShooterConstants.INTAKESPEED2, ControlType.kVelocity);
        break;
      case FEED:
        intakePID1.setReference(Constants.FuelShooterConstants.FEEDSPEED, ControlType.kVelocity);
        intakePID2.setReference(-Constants.FuelShooterConstants.FEEDSPEED, ControlType.kVelocity);
        break;
      case NONE:
        intakePID1.setReference(0, ControlType.kCurrent);
        intakePID2.setReference(0, ControlType.kCurrent);
        break;
      case SUCK: 
        intakePID1.setReference(-Constants.FuelShooterConstants.INTAKESPEED1, ControlType.kVelocity);
        intakePID2.setReference(Constants.FuelShooterConstants.INTAKESPEED2, ControlType.kVelocity);
        break;
      case BLOW:
      intakePID1.setReference(-Constants.FuelShooterConstants.INTAKESPEED1, ControlType.kVelocity);
        intakePID2.setReference(Constants.FuelShooterConstants.INTAKESPEED2, ControlType.kVelocity);
        break;
    
    }
  }

  public IntakeState getIntakeState(){
    return state;
  } 

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

}
