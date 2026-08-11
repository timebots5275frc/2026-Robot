// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Input;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Constants;
import frc.robot.CustomTypes.Math.Vector2;

public class Input extends SubsystemBase {
  Joystick driveJoystick;
  XboxController controller;
  double controllerSpeed;

  // Whichever device most recently produced non-zero input is treated as the
  // active driving device, so the joystick and the Xbox controller can both
  // drive the robot without either one needing to be unplugged.
  public boolean usingJoystick = true;

  Vector2 rawJoystickInput = Vector2.zero;
  double rawJoystickTwist = 0;

  Vector2 joystickInput = Vector2.zero;
  double joystickTwist = 0;

  Vector2 rawControllerInput = Vector2.zero;
  double rawControllerTurn = 0;

  Vector2 controllerInput = Vector2.zero;
  double controllerTurn = 0;

  public BooleanSupplier receivingJoystickInput = new BooleanSupplier() {
    public boolean getAsBoolean() { return joystickInput.x != 0 || joystickInput.y != 0; }
  };

  public static double Throttle;

  public Input(Joystick joystick, XboxController controller) {
    this.driveJoystick = joystick;
    this.controller = controller;
    this.controllerSpeed = 0.6;
  }

  @Override
  public void periodic() {
    getRawJoystickInput();
    calculateJoystickInput();

    getRawControllerInput();
    calculateControllerInput();

    if (joystickInput.x != 0 || joystickInput.y != 0 || joystickTwist != 0) {
      usingJoystick = true;
    } else if (controllerInput.x != 0 || controllerInput.y != 0 || controllerTurn != 0) {
      usingJoystick = false;
    }
  }

  void getRawJoystickInput()
  {
    rawJoystickInput = new Vector2(-driveJoystick.getY(), -driveJoystick.getX());
    rawJoystickTwist = driveJoystick.getTwist();

    Throttle = (driveJoystick.getThrottle() / -2) + .5;
  }

  void calculateJoystickInput()
  {
    joystickInput = new Vector2(calculateInputWithDeadzone(rawJoystickInput.x, Constants.ControllerConstants.DEADZONE_DRIVE), calculateInputWithDeadzone(rawJoystickInput.y, Constants.ControllerConstants.DEADZONE_DRIVE));
    joystickTwist = calculateInputWithDeadzone(rawJoystickTwist, Constants.ControllerConstants.DEADZONE_STEER);
  }

  void getRawControllerInput() {
    rawControllerInput = new Vector2(-controller.getLeftY(), -controller.getLeftX());
    rawControllerTurn = -controller.getRightX();
  }

  void calculateControllerInput() {
    controllerInput = new Vector2(calculateInputWithDeadzone(rawControllerInput.x, Constants.ControllerConstants.DEADZONE_DRIVE), calculateInputWithDeadzone(rawControllerInput.y, Constants.ControllerConstants.DEADZONE_DRIVE));
    controllerTurn = -calculateInputWithDeadzone(rawControllerTurn, Constants.ControllerConstants.DEADZONE_STEER);
  }

  public double getThrottle() {
    return driveJoystick.getThrottle();
  }

  public void incrementControllerSpeed() {
    if (controllerSpeed + 0.2 <= 1) { controllerSpeed += 0.2; }
  }

  public void decrementControllerSpeed() {
    if (controllerSpeed - 0.2 >= 0) { controllerSpeed -= 0.2; }
  }

  public double getControllerSpeed() {
    return controllerSpeed;
  }

  public void flipRumble() {
    System.out.println("Flip rumble");
    rumbleController(GenericHID.RumbleType.kBothRumble, 1);
  }

  public void stopRumble() {
    rumbleController(GenericHID.RumbleType.kBothRumble, 0.0);
  }

  public void rumbleController(GenericHID.RumbleType rumbleType, double Throttle) {
    controller.setRumble(rumbleType, Throttle);
  }

  public Vector2 JoystickInput() { return joystickInput; }
  public double JoystickTwist() { return joystickTwist; }
  public Vector2 ControllerInput() {return controllerInput; }
  public double ControllerTurn() {return controllerTurn; }

  /** Move input from whichever device (joystick or Xbox controller) is currently active. */
  public Vector2 DriveInput() { return usingJoystick ? joystickInput : controllerInput; }
  /** Turn input from whichever device (joystick or Xbox controller) is currently active. */
  public double DriveTwist() { return usingJoystick ? joystickTwist : controllerTurn; }
  /** Speed percent (0-1) from whichever device (joystick or Xbox controller) is currently active. */
  public double DriveSpeedPercent() { return usingJoystick ? (-getThrottle() + 1) / 2 : controllerSpeed; }

  public double calculateInputWithDeadzone(double input, double deadZone) {
    if (Math.abs(input) < deadZone) {
        return 0;
    }

    if (input > 0) {
        return (input - deadZone) / (1 - deadZone);
    } else if (input < 0) {
        return (input + deadZone) / (1 - deadZone);
    }
    return 0;
}
}
