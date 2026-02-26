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

  // private double iS = 0;
  // CANDriveSubsystem cs;

  public enum FuelShooterState{
    CHARGEMOTOR,
    NONE,
    
  }

  public FuelShooter() {
    



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
      case CHARGEMOTOR: //intakePID1.setReference(500, ControlType.kVelocity);
                   

      break;
      case NONE: shooterMotorPID.setReference(Constants.FuelShooterConstants.MOTOR_SPEED_NONE, ControlType.kVelocity);
                 intakePID1.setReference(Constants.FuelShooterConstants.MOTOR_SPEED_NONE, ControlType.kVelocity);
                 intakePID2.setReference(Constants.FuelShooterConstants.MOTOR_SPEED_NONE, ControlType.kVelocity);
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
    
    double rpm = 3500;
    
    double deltaH = Constants.CalculateShooterRpmConstants.TARGET_HEIGHT - Constants.CalculateShooterRpmConstants.CAMERA_HEIGHT;
    double thetaRad = Math.toRadians(Constants.CalculateShooterRpmConstants.SHOOTER_ANGLE);
    double cosTheta = Math.cos(thetaRad);
    double denominator = (2.0 * cosTheta * cosTheta * (dx * Math.tan(thetaRad) - deltaH));
     if (denominator <= 0.0) {return rpm;}
    double v0 = Math.sqrt(Constants.CalculateShooterRpmConstants.GRAVITY * dx * dx / denominator);
    rpm = (v0 * 60)/2*Math.PI*4.5;
    rpm *= Constants.CalculateShooterRpmConstants.RPM_FUDGE_FACTOR;
    // SmartDashboard.putNumber("rpm", rpm);
    // SmartDashboard.putNumber("den", denominator);

    return rpm;
  }

  @Override
  public void periodic() {

  }

  public FuelShooterState getShooterState() {
    return state;
  }

  public SparkFlex getMotor() {
    return shooterMotor;
  }
}
