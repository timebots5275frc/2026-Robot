// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.Constants;
import frc.robot.Constants.Constants.OperatorConstants;
import frc.robot.commands.FuelShooterCommand;
import frc.robot.commands.SetIntakeState;
import frc.robot.commands.StopShooter;
import frc.robot.commands.TeleopJoystickDrive;
import frc.robot.commands.auto.AutoDrive;
import frc.robot.commands.auto.DistanceDrive;
import frc.robot.commands.shoot.ChargeMotor;
import frc.robot.commands.shoot.FeedFuel;
import frc.robot.commands.shoot.LimelightDistanceShootCommand;
import frc.robot.commands.shoot.LockOnHub;
import frc.robot.subsystems.CANDriveSubsystem;
import frc.robot.subsystems.FuelShooter;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.FuelShooter.FuelShooterState;
import frc.robot.subsystems.IntakeSubsystem.IntakeState;
import frc.robot.subsystems.Input.Input;
import frc.robot.subsystems.Vision.Vision;

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
    Vision vision;
    CANDriveSubsystem tankDrive;
    FuelShooter fs;
    IntakeSubsystem intake;
   

    
    // Climb climb;

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController m_driverController =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer(SendableChooser<Command> autonChooser, Vision vision) {
    this.vision = vision;
    bBoard = new GenericHID(1);
    joy = new Joystick(0);
    input = new Input(joy);
    tankDrive = new CANDriveSubsystem(vision);
    fs = new FuelShooter();
    intake = new IntakeSubsystem();

    // autonChooser.setDefaultOption("Drive ONLY", new SequentialCommandGroup(
    //   // new AutoDrive(tankDrive,MathConstants.INCH_TO_METER*22,0),
    //   new AutoDrive(tankDrive,MathConstants.INCH_TO_METER*22, 0)
    // ));


    autonChooser.setDefaultOption("LIMELIGHT SHOOT with distance", new SequentialCommandGroup(
      new DistanceDrive(tankDrive, -1.5, 0.5),
      new LimelightDistanceShootCommand(vision, fs, tankDrive),
      //new LockOnHub(tankDrive, vision),
      // new ChargeMotor(fs, vision),
      new FeedFuel(intake)
    ));

    autonChooser.addOption("LIMELIGHT SHOOT with time", new SequentialCommandGroup(
      new AutoDrive(tankDrive, -0.5, 0).withTimeout(1.25),
      new WaitCommand(1.5),
      new LimelightDistanceShootCommand(vision, fs, tankDrive),
      new FeedFuel(intake).withTimeout(5),
      new AutoDrive(tankDrive, -2, 0).withTimeout(.1),
      new AutoDrive(tankDrive, 2, 0).withTimeout(.2),
      new FeedFuel(intake)
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

     new JoystickButton(joy, Constants.ButtonConstants.FLIP_FRONT_BUTTON_ID).whileFalse(teleJoyDrive = new TeleopJoystickDrive(tankDrive, input, false, 1));
     new JoystickButton(joy, Constants.ButtonConstants.FLIP_FRONT_BUTTON_ID).whileTrue(teleJoyDrive = new TeleopJoystickDrive(tankDrive, input, false, -1));
    // teleJoyDrive = new TeleopJoystickDrive(tankDrive, input, false, -1);
     tankDrive.setDefaultCommand(teleJoyDrive);
    
    //new JoystickButton(joy, 7).onTrue(new InstantCommand(tankDrive::flipFieldRelative ,tankDrive));
    /*tmp */
    //pigeon
    //new JoystickButton(joy, 8).onTrue(new InstantCommand(swerveDrive::resetPigeon, swerveDrive));
    
    //new JoystickButton(joy, 1).onTrue(new FuelShooterCommand(fs, vision, FuelShooterState.LOCKTOHUB));

    /*
     * 1.Locks onto hub
     * 2.Finds nescasarry RPM & charges motor
     * 3.feeds fuel into shooter
     */

     //shoot no lock on
     //new JoystickButton(joy, Constants.ButtonConstants.SHOOT_LIMELIGHT_BUTTON_ID).onTrue(new SequentialCommandGroup(new LimelightDistanceShootCommand(vision, fs, tankDrive), new FeedFuel(intake)));
    
     //shoot with lock on
     new JoystickButton(joy, Constants.ButtonConstants.SHOOT_LIMELIGHT_BUTTON_ID).onTrue(new SequentialCommandGroup(
      new LockOnHub(tankDrive, vision, fs),
      new FeedFuel(intake)
      ));
    
    
    //shoot without vision
    new JoystickButton(bBoard, Constants.ButtonConstants.SHOOT_NO_LIMELIGHT_BUTTON_ID).onTrue(new SequentialCommandGroup( new ChargeMotor(fs, Constants.FuelShooterConstants.DEFAULT_SHOOTER_RPM), new FeedFuel(intake)));

    new JoystickButton(bBoard, Constants.ButtonConstants.INTAKE_BUTTON_ID).onTrue(new SetIntakeState(intake, IntakeState.INTAKE));

    new JoystickButton(bBoard, Constants.ButtonConstants.OUTTAKE_BUTTON_ID).onTrue(new SetIntakeState(intake, IntakeState.OUTTAKE)); 

    new JoystickButton(bBoard, Constants.ButtonConstants.STOP_INTAKE_BUTTON_ID).onTrue(new SetIntakeState(intake, IntakeState.NONE)); 

    new JoystickButton(bBoard, Constants.ButtonConstants.STOP_SHOOTER_BUTTON_ID).onTrue(new StopShooter(fs)); 

    new JoystickButton(bBoard,Constants.ButtonConstants.SHAKE_ROBOT_BUTTON_ID).onTrue(new SequentialCommandGroup(new AutoDrive(tankDrive, -2, 0).withTimeout(.1), new AutoDrive(tankDrive, 2, 0).withTimeout(.2)/* , teleJoyDrive = new TeleopJoystickDrive(tankDrive, input, false, -1)*/));
  
    new JoystickButton(bBoard, Constants.ButtonConstants.SUCK_BUTTON).onTrue(new ParallelCommandGroup(new SetIntakeState(intake, IntakeState.SUCK), new FuelShooterCommand(fs, vision, FuelShooterState.SUCK)));

    // TODO label button
    new JoystickButton(bBoard, Constants.ButtonConstants.BLOW_BUTTON).onTrue(new ParallelCommandGroup(new SetIntakeState(intake, IntakeState.BLOW), new FuelShooterCommand(fs, vision, FuelShooterState.BLOW)));
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
