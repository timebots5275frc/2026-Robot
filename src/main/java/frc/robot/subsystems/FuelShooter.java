package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.Vision.Vision;

public class FuelShooter extends SubsystemBase {

  private SparkMax shooterMotor;
  public SparkClosedLoopController shooterPID;
  private SparkMax intakeMotor1;
  //private SparkClosedLoopController intakePID1;
  private SparkMax intakeMotor2;
 // private SparkClosedLoopController intakePID2;

  private double shooterRPM = 0.0;

  private double tx, ty, shooterAngleDeg;

  private FuelShooterState state;

  /** Distance to target (meters) for dashboard/debug */
  public double dx = 0.0;

  public enum FuelShooterState{
    CHARGEMOTOR,
    NONE,
    FEEDBALL,
    LOCKTOHUB;

    
  }

  public FuelShooter() {
    shooterMotor = new SparkMax(Constants.FuelShooterConstants.SHOOTER_MOTOR_ID,SparkLowLevel.MotorType.kBrushless);
    Constants.FuelShooterConstants.SHOOTER_MOTOR_PID.setSparkMaxPID(shooterMotor,ResetMode.kResetSafeParameters,PersistMode.kPersistParameters);
    shooterPID = shooterMotor.getClosedLoopController();

  //   intakeMotor1 = new SparkMax(Constants.FuelShooterConstants.INTAKE_MOTOR_1_ID,SparkLowLevel.MotorType.kBrushless);
  //   Constants.FuelShooterConstants.INTAKE_MOTOR_1_PID.setSparkMaxPID(intakeMotor1, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  //   intakePID1 = intakeMotor1.getClosedLoopController();

  //   //intakeMotor2 = new SparkMax(Constants.FuelShooterConstants.INTAKE_MOTOR_2_ID, SparkLowLevel.MotorType.kBrushless);
  //   Constants.FuelShooterConstants.INTAKE_MOTOR_2_PID.setSparkMaxPID(intakeMotor2, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
  //   intakePID2 = intakeMotor2.getClosedLoopController();
   }

  public void SetShooterState(FuelShooterState state){
    this.state = state;
    UpdateShooterState(state);
  }

  public void UpdateShooterState(FuelShooterState state){
    switch(state){
      case CHARGEMOTOR: 
      break;
      case NONE: shooterPID.setReference(0, ControlType.kCurrent);
      break;
      case FEEDBALL: 
      break;
      case LOCKTOHUB: Vision.usingLimelight = true;
      break;
    }
  }

  /** Set shooter velocity in RPM */
  public void setShooterRPM(double rpm) {
    shooterRPM = rpm;
    shooterPID.setReference(rpm, ControlType.kVelocity);
  }

  /** Stop shooter safely */
  public void stopShooter() {
    shooterRPM = 0.0;
    shooterPID.setReference(0.0, ControlType.kVelocity);
  }

  public double calculateRPMFromLimelight(double tx, double ty, double shooterAngleDeg) {
    this.tx = tx;
    this.ty = ty;
    this.shooterAngleDeg = shooterAngleDeg;
    // --- Camera geometry ---
    double cameraAngleRad = Math.toRadians(ty + Constants.CalculateShooterRpmConstants.MOUNTING_ANGLE);
    //change in height from camera to target
    double deltaH = Constants.CalculateShooterRpmConstants.TARGET_HEIGHT - Constants.CalculateShooterRpmConstants.CAMERA_HEIGHT;
    // Distance straight ahead
    double forwardDistance = 51 * 0.0254 /*deltaH / Math.tan(cameraAngleRad) */;
    // Compensate for Limelight yaw
    this.dx = forwardDistance / Math.cos(Math.toRadians(tx));
    // --- Ballistics ---
    double thetaRad = Math.toRadians(shooterAngleDeg);
    //denominator
    double denominator = (2.0 * dx * dx * (dx * Math.tan(thetaRad) - deltaH));
    //denominator checks
    if (denominator <= 0){ 
     // denominator *= -1;
      return 0;
    } //converts denominator to positive
   //  if (denominator > 0){return denominator;} //returns denominator as normal
    if (denominator <= 0.0) {return shooterRPM;} // If unreachable, return last RPM instead of crashing
    //solves for velocity initial
    double v0 = Math.sqrt(Constants.CalculateShooterRpmConstants.GRAVITY * Math.pow(dx, 2) / denominator);
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
    
  }

  public FuelShooterState getShooterState() {
    return state;
  }

  public SparkMax getMotor() {
    return shooterMotor;
  }
}
