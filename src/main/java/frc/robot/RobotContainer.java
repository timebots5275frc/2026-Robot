// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.Constants;
import frc.robot.Constants.Constants.OperatorConstants;

import frc.robot.commands.TeleopJoystickDrive;
import frc.robot.commands.auto.AutoDrive;
import frc.robot.commands.auto.DistanceDrive;

import frc.robot.subsystems.CANDriveSubsystem;

import frc.robot.subsystems.Input.Input;


import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...

    Joystick joy;
    Input input;
    TeleopJoystickDrive teleJoyDrive;
    GenericHID bBoard;
   
    CANDriveSubsystem tankDrive;
  
   

    
    // Climb climb;

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController m_driverController =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer(SendableChooser<Command> autonChooser) {
    
    bBoard = new GenericHID(1);
    joy = new Joystick(0);
    input = new Input(joy);
    tankDrive = new CANDriveSubsystem();
   

    // autonChooser.setDefaultOption("Drive ONLY", new SequentialCommandGroup(
    //   // new AutoDrive(tankDrive,MathConstants.INCH_TO_METER*22,0),
    //   new AutoDrive(tankDrive,MathConstants.INCH_TO_METER*22, 0)
    // ));


    
    

    SmartDashboard.putData(autonChooser);

    // Configure the trigger bindings
    configureBindings();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {

     new JoystickButton(joy, Constants.ButtonConstants.FLIP_FRONT_BUTTON_ID).whileFalse(teleJoyDrive = new TeleopJoystickDrive(tankDrive, input, false, 1));
     new JoystickButton(joy, Constants.ButtonConstants.FLIP_FRONT_BUTTON_ID).whileTrue(teleJoyDrive = new TeleopJoystickDrive(tankDrive, input, false, -1));
    // teleJoyDrive = new TeleopJoystickDrive(tankDrive, input, false, -1);
     tankDrive.setDefaultCommand(teleJoyDrive);
    
   
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand(SendableChooser<Command> autonChooser) 
  {

    return autonChooser.getSelected(); 
  }
}
