package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.Vision.Vision;

public class FuelShooter extends SubsystemBase {

  private SparkFlex shooterMotor;
  public SparkClosedLoopController shooterMotorPID;
  
  private SparkFlex intakeMotor1;
  private SparkClosedLoopController intakePID1;
  
  private SparkMax intakeMotor2; // this is spark max dont forget
  private SparkClosedLoopController intakePID2;

  private double shooterRPMMult = 1.0;
  private double intakeRPMMult = 1.0;

  private double tx, ty, shooterAngleDeg;

  private double shootRPM = 1.0;

  private FuelShooterState state;

  /** Distance to target (meters) for dashboard/debug */
  public double dx = 0.0;

  public enum FuelShooterState{
    CHARGEMOTOR,
    NONE,
    FEEDBALL,
    LOCKTOHUB,
    INTAKE, OUTTAKE;
  }

  public FuelShooter() {
    

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



    shooterMotor = new SparkFlex(Constants.FuelShooterConstants.SHOOTER_MOTOR_ID, MotorType.kBrushless);
    Constants.FuelShooterConstants.SHOOTER_MOTOR_PID.setFreeLimit(Constants.FuelShooterConstants.SHOOTER_FREE_LIMIT);
    Constants.FuelShooterConstants.SHOOTER_MOTOR_PID.setStallLimit(Constants.FuelShooterConstants.SHOOTER_STALL_LIMIT);
    Constants.FuelShooterConstants.SHOOTER_MOTOR_PID.setSparkFlexPID(shooterMotor,ResetMode.kResetSafeParameters,PersistMode.kPersistParameters, IdleMode.kCoast);
    shooterMotorPID = shooterMotor.getClosedLoopController(); 

    // SparkFlexConfig configFlexIntake = new SparkFlexConfig();
    // configFlexIntake.voltageCompensation(12);
    // configFlexIntake.smartCurrentLimit(Constants.FuelShooterConstants.INTAKE_STALL_LIMIT, 
    //                           Constants.FuelShooterConstants.INTAKE_FREE_LIMIT );
    // intakeMotor2.configure(configFlexIntake, null, null);
              



   }

  public void SetShooterState(FuelShooterState state){
    this.state = state;
    UpdateShooterState(state);
  }

  public void UpdateShooterState(FuelShooterState state){
    switch(state){
      case CHARGEMOTOR: 
      break;
      case NONE: shooterMotorPID.setReference(0, ControlType.kCurrent);
                 intakePID1.setReference(0, ControlType.kCurrent);
                 intakePID2.setReference(0, ControlType.kCurrent);
      break;
      case FEEDBALL: intakePID1.setReference(-Constants.FuelShooterConstants.FEEDSPEED * intakeRPMMult, ControlType.kVelocity);
                    intakePID2.setReference(Constants.FuelShooterConstants.FEEDSPEED* intakeRPMMult, ControlType.kVelocity);
      break;
      case LOCKTOHUB: Vision.usingLimelight = true;
      break;
      case INTAKE: intakePID1.setReference(Constants.FuelShooterConstants.INTAKESPEED1* intakeRPMMult, ControlType.kVelocity);
                    intakePID2.setReference(Constants.FuelShooterConstants.INTAKESPEED2* intakeRPMMult, ControlType.kVelocity);
        break;
      case OUTTAKE:
                    intakePID1.setReference(Constants.FuelShooterConstants.INTAKESPEED1* intakeRPMMult, ControlType.kVelocity);
                    intakePID2.setReference(Constants.FuelShooterConstants.INTAKESPEED2* intakeRPMMult, ControlType.kVelocity);
        break;
    }
  }

  /** Set shooter velocity in RPM */
  public void setShooterRPMMult(double rpm) {
    shooterRPMMult = rpm;
    shooterMotorPID.setReference(rpm, ControlType.kVelocity);
  }

  /** Stop shooter safely */
  public void stopShooter() {
    shooterRPMMult = 0.0;
    shooterMotorPID.setReference(0.0, ControlType.kVelocity);
  }

  public double calculateRPMFromLimelight(double tx, double ty, double dx) {
    this.tx = tx;
    this.ty = ty;
    this.dx = dx + Constants.CalculateShooterRpmConstants.CAMERA_OFFSET;
    
    double rpm = 0;
    
    double deltaH = Constants.CalculateShooterRpmConstants.TARGET_HEIGHT - Constants.CalculateShooterRpmConstants.CAMERA_HEIGHT;
    double thetaRad = Math.toRadians(Constants.CalculateShooterRpmConstants.SHOOTER_ANGLE);
    double cosTheta = Math.cos(thetaRad);
    double denominator = (2.0 * cosTheta * cosTheta * (dx * Math.tan(thetaRad) - deltaH));
     if (denominator <= 0.0) {return rpm+1000;}
    double v0 = Math.sqrt(Constants.CalculateShooterRpmConstants.GRAVITY * dx * dx / denominator);
    rpm = (v0 * 60)/2*Math.PI*4.5;
    rpm *= Constants.CalculateShooterRpmConstants.RPM_FUDGE_FACTOR;
    // SmartDashboard.putNumber("rpm", rpm);
    // SmartDashboard.putNumber("den", denominator);

    return rpm;
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("rpm", shooterRPMMult);
    // SmartDashboard.putNumber("tx", tx);
    // SmartDashboard.putNumber("ty", ty);
    // SmartDashboard.putNumber("shooter angle degree", shooterAngleDeg);
    SmartDashboard.putNumber("dx", dx);

    SmartDashboard.putNumber("Intake 1 rpm", intakeMotor1.getEncoder().getVelocity());
    SmartDashboard.putNumber("Intake 2 rpm", intakeMotor2.getEncoder().getVelocity());

    SmartDashboard.putNumber("Intake 1 Current", intakeMotor1.getOutputCurrent());
    SmartDashboard.putNumber("Intake 2 Current", intakeMotor2.getOutputCurrent());

    SmartDashboard.putNumber("shooter Current", shooterMotor.getOutputCurrent());
    SmartDashboard.putNumber("shooter rpm ", shooterMotor.getEncoder().getVelocity());
    
  }

  public double getShooterRPMMult() {
    return shooterRPMMult;
  }

  public void increaseShooterRPM() {
    shooterRPMMult *= 1.05;
  }

  public void decreaseShooterRPM() {
    shooterRPMMult *= 0.95;
  }

  public double getIntakeRPMMult() {
    return intakeRPMMult;
  }

  public void increaseIntakeRPM() {
    intakeRPMMult *= 1.05;
  }

  public void decreaseIntakeRPM() {
    intakeRPMMult *= 0.95;
  }

  public FuelShooterState getShooterState() {
    return state;
  }

  public SparkFlex getMotor() {
    return shooterMotor;
  }
}
