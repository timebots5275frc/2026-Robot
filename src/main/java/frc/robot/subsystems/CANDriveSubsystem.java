// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import static frc.robot.Constants.DriveConstants.*;

public class CANDriveSubsystem extends SubsystemBase {
  Constants c = new Constants();

  private final SparkFlex leftLeader;
  private final SparkFlex leftFollower;
  private final SparkFlex rightLeader;
  private final SparkFlex rightFollower;

  private final DifferentialDrive drive;

  private double leftMult = 1;
  private double rightMult = 1;

  public CANDriveSubsystem() {
    // create brushed motors for drive
    leftLeader = new SparkFlex(LEFT_LEADER_ID, MotorType.kBrushless);
    leftFollower = new SparkFlex(LEFT_FOLLOWER_ID, MotorType.kBrushless);
    rightLeader = new SparkFlex(RIGHT_LEADER_ID, MotorType.kBrushless);
    rightFollower = new SparkFlex(RIGHT_FOLLOWER_ID, MotorType.kBrushless);

    
    // set up differential drive class
    drive = new DifferentialDrive(leftLeader, rightLeader);

    // Set can timeout. Because this project only sets parameters once on
    // construction, the timeout can be long without blocking robot operation. Code
    // which sets or gets parameters during operation may need a shorter timeout.
    leftLeader.setCANTimeout(250);
    rightLeader.setCANTimeout(250);
    leftFollower.setCANTimeout(250);
    rightFollower.setCANTimeout(250);

    // Create the configuration to apply to motors. Voltage compensation
    // helps the robot perform more similarly on different
    // battery voltages (at the cost of a little bit of top speed on a fully charged
    // battery). The current limit helps prevent tripping
    // breakers.
    SparkFlexConfig config = new SparkFlexConfig();
    config.voltageCompensation(12);
    config.smartCurrentLimit(Constants.DriveConstants.DRIVE_MOTOR_STALL_LIMIT, 
                             Constants.DriveConstants.DRIVE_MOTOR_FREE_LIMIT,
                             Constants.DriveConstants.DRIVE_MOTOR_LIMIT_RPM);

    config.closedLoopRampRate(Constants.DriveConstants.DRIVE_MOTOR_RAMP_RATE);

    

    // Set configuration to follow each leader and then apply it to corresponding
    // follower. Resetting in case a new controller is swapped
    // in and persisting in case of a controller reset due to breaker trip
    config.follow(leftLeader);
    leftFollower.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    config.follow(rightLeader);
    rightFollower.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    // Remove following, then apply config to right leader
    config.disableFollowerMode();
    rightLeader.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    // Set config to inverted and then apply to left leader. Set Left side inverted
    // so that postive values drive both sides forward
    config.inverted(true);
    leftLeader.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  @Override
  public void periodic() {

  }

  public void driveArcade(double xSpeed, double zRotation) {

    var speeds = DifferentialDrive.arcadeDriveIK(xSpeed, zRotation, true);

    double leftSpeed = speeds.left;
    double rightSpeed = speeds.right;

    leftSpeed = leftSpeed * leftMult;
    rightSpeed = rightSpeed * rightMult;

    drive.tankDrive(leftSpeed, rightSpeed, false);

    System.out.println(xSpeed);

  }

  public void changeRightSpeed(double change) {
    rightMult += change;
  }

  public void changeLeftSpeed(double change) {
    leftMult += change;
  }

}
