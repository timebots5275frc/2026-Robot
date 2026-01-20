// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class FuelShooter extends SubsystemBase {
  /** Creates a new FuelErector. */

  SparkMax motor1;
  SparkMax motor2;
  SparkMax motor3;
  

  public enum Shooter{
    INTAKE,
    OUTTAKE,
    SHOOT;
  }

  public FuelShooter() {
    /* 
    motor1 = new SparkMax(0, ControlType.kVelocity);

    motor2 = new SparkMax(0, null);

    motor3 = new SparkMax(0, null);
    */
  }

  public double calculateRPMFromLimelight(double ty, double tx, double cameraHeight, 
  double targetHeight, double mountingAngle, double theta, double r, double g) {

    // Convert vertical angles to radians
    double totalAngle = Math.toRadians(ty + mountingAngle);

    // Calculate straight-line distance to target
    double distance = (targetHeight - cameraHeight) / Math.tan(totalAngle);

    // Horizontal distance along forward axis
    double dx = distance * Math.cos(Math.toRadians(tx));

    // Vertical difference
    double dy = targetHeight - cameraHeight;

    // Formula for linear velocity
    double denominator = 2 * Math.pow(Math.cos(theta), 2) * (dx * Math.tan(theta) - dy);
    if (denominator <= 0) {
        throw new IllegalArgumentException("Target unreachable at this angle.");
    }

    double v0 = Math.sqrt(g * dx * dx / denominator);

    // Convert to wheel RPM
    return (60 / (2 * Math.PI * r)) * v0;
}


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
