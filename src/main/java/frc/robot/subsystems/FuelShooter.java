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
  public SparkClosedLoopController ShooterPID; //shooter pid is null design way around for limelight testing
                                               //add shooter motor controller and motor on tuesday to get around this problem
  private AbsoluteEncoder ShooterEncoder;
  
  private Shooter shooterState = Shooter.NONE;
  private double shooterRPM = 4000;
  private boolean feedingFuel = false;
  public double dx;

  public enum Shooter{
    // INTAKE,
    // OUTTAKE,
    SHOOT,
    NONE;
  }

  public FuelShooter() {
    
    // IntakeMotor1 = new SparkMax(Constants.FuelShooterConstants.INTAKE_MOTOR_1_ID, SparkLowLevel.MotorType.kBrushless);
    // Constants.FuelShooterConstants.motor1PID.setSparkMaxPID(IntakeMotor1, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    //IntakePIDOne = IntakeMotor1.getClosedLoopController();

    // IntakeMotor2 = new SparkMax(Constants.FuelShooterConstants.INTAKE_MOTOR_2_ID, SparkLowLevel.MotorType.kBrushless);
    // Constants.FuelShooterConstants.motor2PID.setSparkMaxPID(IntakeMotor2, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    // IntakePIDTwo = IntakeMotor2.getClosedLoopController();

    shooterMotor = new SparkMax(Constants.FuelShooterConstants.SHOOTER_MOTOR_ID, SparkLowLevel.MotorType.kBrushless);
    Constants.FuelShooterConstants.SHOOTER_MOTOR_PID.setSparkMaxPID(shooterMotor, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    ShooterPID = shooterMotor.getClosedLoopController();
    
  }

  public void Shooter(){

    switch (shooterState) {
      case NONE: feedingFuel = false;
                 ShooterPID.setReference(0, ControlType.kCurrent);
      break;
      // case INTAKE: feedingFuel = true;

      // break;
      // case OUTTAKE: IntakePIDOne.setReference(-Constants.FuelShooterConstants.INTAKESPEED, ControlType.kVelocity);
      //               IntakePIDTwo.setReference(-Constants.FuelShooterConstants.INTAKESPEED, ControlType.kVelocity);
      //               ShooterPID.setReference(0, ControlType.kVelocity);
      // break;
      case SHOOT: IntakePIDOne.setReference(0, ControlType.kVelocity);
                  IntakePIDTwo.setReference(0, ControlType.kVelocity);
                  ShooterPID.setReference(shooterRPM, ControlType.kVelocity);
      break;
      default: ShooterPID.setReference(0, ControlType.kCurrent);
               IntakePIDOne.setReference(0, ControlType.kCurrent);
               IntakePIDTwo.setReference(0, ControlType.kCurrent);
      
    }

  }

//   public void feedFuel() {
//     if (feedingFuel == true) return;
//       IntakePIDOne.setReference(Constants.FuelShooterConstants.IntakeSpeed, ControlType.kVelocity);
//       IntakePIDTwo.setReference(-Constants.FuelShooterConstants.IntakeSpeed, ControlType.kVelocity);
//       feedingFuel = true;
//     }

//   public void stopFeedingFuel() {
//     if (feedingFuel == false) return;
//       IntakePIDOne.setReference(0, ControlType.kCurrent);
//       IntakePIDTwo.setReference(0, ControlType.kCurrent);
//       feedingFuel = false;
//     }

  

  public void setState(Shooter state) {
    shooterState = state;
    Shooter();
  }

//   public void setShooterRPM(double rpm) {
//     shooterRPM = rpm;
//     Shooter();
//   } 

//   public boolean shooterAtTargetRPM() {
//       if(shooterMotor.getEncoder().getVelocity() == shooterRPM && shooterRPM != 0) {
//         return true;
//       }
//       return false;
//   }

  //does the calculations in HubAimCommand
  public double calculateRPMFromLimelight(
        double tx,
        double ty,
        double shooterAngleDeg
) {

    // --- Camera geometry (distance calculation) ---
    double cameraAngleRad = Math.toRadians(
            ty + Constants.CalculateShooterRpmConstants.MOUNTING_ANGLE
    );

    double deltaH =
            Constants.CalculateShooterRpmConstants.TARGET_HEIGHT
            - Constants.CalculateShooterRpmConstants.CAMERA_HEIGHT;

    // Forward distance from camera to target
    double forwardDistance = deltaH / Math.tan(cameraAngleRad);

    // Compensate for Limelight yaw (tx)
    double dx = forwardDistance / Math.cos(Math.toRadians(tx));

    // --- Shooter physics (ballistic calculation) ---
    double thetaRad = Math.toRadians(shooterAngleDeg);

    double denominator =
            2.0
            * Math.pow(Math.cos(thetaRad), 2.0)
            * (dx * Math.tan(thetaRad) - deltaH);

    if (denominator <= 0.0) {
        throw new IllegalArgumentException(
                "Target unreachable at this distance/angle"
        );
    }

    double v0 = Math.sqrt(
            Constants.CalculateShooterRpmConstants.GRAVITY
            * dx * dx
            / denominator
    );

    // --- Convert linear velocity to wheel RPM ---
    double rpm =
            (60.0 / (2.0 * Math.PI
            * Constants.CalculateShooterRpmConstants.WHEEL_RADIUS))
            * v0;

    // --- Empirical correction (tune this on the robot) ---
    rpm *= Constants.CalculateShooterRpmConstants.RPM_FUDGE_FACTOR;

    // --- Optional debugging ---
    /*
    System.out.println(
        String.format(
            "dx=%.2f m | shooterAngle=%.1f deg | rpm=%.0f",
            dx,
            shooterAngleDeg,
            rpm
        )
    );
    */

    return rpm;
}


  @Override
  public void periodic() {

  }
}
