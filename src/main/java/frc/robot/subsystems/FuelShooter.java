package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
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

  private double shooterRPM = 0.0;

  private double tx, ty, shooterAngleDeg;

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
    Constants.FuelShooterConstants.INTAKE_MOTOR_2_PID.setSparkMaxPID(intakeMotor2,ResetMode.kResetSafeParameters,PersistMode.kPersistParameters);
    intakePID2 = intakeMotor2.getClosedLoopController();

    intakeMotor1 = new SparkFlex(Constants.FuelShooterConstants.INTAKE_MOTOR_1_ID, MotorType.kBrushless);
    Constants.FuelShooterConstants.INTAKE_MOTOR_1_PID.setSparkFlexPID(intakeMotor1,ResetMode.kResetSafeParameters,PersistMode.kPersistParameters);
    intakePID1 = intakeMotor1.getClosedLoopController();

    shooterMotor = new SparkFlex(Constants.FuelShooterConstants.SHOOTER_MOTOR_ID, MotorType.kBrushless);
    Constants.FuelShooterConstants.SHOOTER_MOTOR_PID.setSparkFlexPID(shooterMotor,ResetMode.kResetSafeParameters,PersistMode.kPersistParameters, IdleMode.kCoast);
    shooterMotorPID = shooterMotor.getClosedLoopController(); 

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
      case FEEDBALL: intakePID1.setReference(-Constants.FuelShooterConstants.FEEDSPEED, ControlType.kVelocity);
                    intakePID2.setReference(Constants.FuelShooterConstants.FEEDSPEED, ControlType.kVelocity);
      break;
      case LOCKTOHUB: Vision.usingLimelight = true;
      break;
      case INTAKE: intakePID1.setReference(Constants.FuelShooterConstants.INTAKESPEED, ControlType.kVelocity);
                    intakePID2.setReference(Constants.FuelShooterConstants.INTAKESPEED, ControlType.kVelocity);
        break;
      case OUTTAKE:
                    //intakePID1.setReference(-Constants.FuelShooterConstants.INTAKESPEED + 1000, ControlType.kVelocity);
                    intakePID2.setReference(Constants.FuelShooterConstants.INTAKESPEED, ControlType.kVelocity);
        break;
    }
  }

  /** Set shooter velocity in RPM */
  public void setShooterRPM(double rpm) {
    shooterRPM = rpm;
    shooterMotorPID.setReference(rpm, ControlType.kVelocity);
  }

  /** Stop shooter safely */
  public void stopShooter() {
    shooterRPM = 0.0;
    shooterMotorPID.setReference(0.0, ControlType.kVelocity);
  }

  public double calculateRPMFromLimelight(double tx, double ty, double shooterAngleDeg, double dx) {
    this.tx = tx;
    this.ty = ty;
    this.shooterAngleDeg = shooterAngleDeg;
    

    // --- Camera geometry ---
    //double cameraAngleRad = Math.toRadians(ty+ Constants.CalculateShooterRpmConstants.MOUNTING_ANGLE);
    //change in height from camera to target
    double deltaH = Constants.CalculateShooterRpmConstants.TARGET_HEIGHT - Constants.CalculateShooterRpmConstants.CAMERA_HEIGHT;

    this.dx = dx;
    // --- Ballistics ---
    double thetaRad = Math.toRadians(shooterAngleDeg);
    double cosTheta = Math.cos(thetaRad);
    //denominator
    double denominator =
    2.0 * cosTheta * cosTheta * (dx * Math.tan(thetaRad) - deltaH);

    SmartDashboard.putNumber("dx * tan(theta)", dx * Math.tan(thetaRad));
    SmartDashboard.putNumber("deltaH", deltaH);
     if (denominator <= 0.0) {return shooterRPM;} // If unreachable, return last RPM instead of crashing
    //solves for velocity initial
    double v0 = Math.sqrt(Constants.CalculateShooterRpmConstants.GRAVITY * dx * dx / denominator);
    // --- Convert to wheel RPM ---
    double rpm = (60.0 / (2.0 * Math.PI * Constants.CalculateShooterRpmConstants.WHEEL_RADIUS)) * v0;
    // Empirical tuning
    rpm *= Constants.CalculateShooterRpmConstants.RPM_FUDGE_FACTOR;
    //returns final rpm
    SmartDashboard.putNumber("rpm", rpm);
    return rpm;
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("rpm", shooterRPM);
    SmartDashboard.putNumber("tx", tx);
    SmartDashboard.putNumber("ty", ty);
    SmartDashboard.putNumber("shooter angle degree", shooterAngleDeg);
    SmartDashboard.putNumber("dx", dx);

    SmartDashboard.putNumber("Intake 1 rpm", intakeMotor1.getEncoder().getVelocity());
    SmartDashboard.putNumber("Intake 2 rpm", intakeMotor2.getEncoder().getVelocity());

    SmartDashboard.putNumber("Intake 1 Current", intakeMotor1.getOutputCurrent());
    SmartDashboard.putNumber("Intake 2 Current", intakeMotor2.getOutputCurrent());

    SmartDashboard.putNumber("shooter Current", shooterMotor.getOutputCurrent());
    SmartDashboard.putNumber("shooter rpm ", shooterMotor.getEncoder().getVelocity());
    
  }

  public FuelShooterState getShooterState() {
    return state;
  }

  public SparkFlex getMotor() {
    return shooterMotor;
  }
}
