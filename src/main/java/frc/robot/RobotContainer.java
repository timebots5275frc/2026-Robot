// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.MathConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.ClimbCommand;
import frc.robot.commands.HubAimCommand;
import frc.robot.commands.ResetClimb;
import frc.robot.commands.TeleopJoystickDrive;
import frc.robot.commands.auto.AutoDrive;
import frc.robot.subsystems.CANDriveSubsystem;
import frc.robot.subsystems.Climb;
import frc.robot.subsystems.FuelShooter;
import frc.robot.subsystems.Climb.ClimbStates;
import frc.robot.subsystems.Input.Input;

import frc.robot.subsystems.Vision.Vision;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
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
    Vision vision;
    CANDriveSubsystem tankDrive;
    FuelShooter fs;
    Climb climb;

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController m_driverController =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer(SendableChooser<Command> autonChooser) {
    bBoard = new GenericHID(1);
    joy = new Joystick(0);
    input = new Input(joy);
    tankDrive = new CANDriveSubsystem();
    fs = new FuelShooter();
    vision = new Vision();
    // fuelShooter = new FuelShooter();
    
    

    autonChooser.setDefaultOption("Drive ONLY", new SequentialCommandGroup(
      //new AutoDrive(MathConstants.INCH_TO_METER*22,1,tankDrive)
      new AutoDrive(tankDrive,MathConstants.INCH_TO_METER*22, 1)
    ));

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

    teleJoyDrive = new TeleopJoystickDrive(tankDrive, input, true);
    tankDrive.setDefaultCommand(teleJoyDrive);
    
    
    //new JoystickButton(joy, 7).onTrue(new InstantCommand(tankDrive::flipFieldRelative ,tankDrive));
    /*tmp */
    //pigeon
    //new JoystickButton(joy, 8).onTrue(new InstantCommand(swerveDrive::resetPigeon, swerveDrive));
    
    new JoystickButton(joy, 1).onTrue((new HubAimCommand(tankDrive, vision, fs))); //shoot with vision
    new JoystickButton(joy, 2).onTrue(new ClimbCommand(climb, ClimbStates.L1)); //climb L1
    new JoystickButton(joy, 2).onTrue(new ClimbCommand(climb, ClimbStates.DRIVE)); //climb to drive
    new JoystickButton(joy, 2).whileTrue(new ResetClimb(climb)); //reset climb

    // new JoystickButton(bBoard, Constants.ButtonConstants.INCREASE_LEFT_MOTOR).onTrue(new ChangeMotorSpeed(tankDrive, 0.1, false));

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
