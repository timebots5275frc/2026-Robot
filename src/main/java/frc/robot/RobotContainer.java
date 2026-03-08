// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.MathConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.FuelShooterCommand;
import frc.robot.commands.Intake;
import frc.robot.commands.SetIntakeState;
import frc.robot.commands.StopShooter;
import frc.robot.commands.TeleopJoystickDrive;
import frc.robot.commands.auto.AutoDrive;
import frc.robot.commands.auto.DegreeTurn;
import frc.robot.commands.auto.DistanceDrive;
import frc.robot.commands.shoot.ChargeMotor;
import frc.robot.commands.shoot.FeedFuel;
import frc.robot.commands.shoot.LockOnHub;
import frc.robot.subsystems.CANDriveSubsystem;
import frc.robot.subsystems.FuelShooter;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.FuelShooter.FuelShooterState;
import frc.robot.subsystems.IntakeSubsystem.IntakeState;
import frc.robot.subsystems.Input.Input;
import frc.robot.subsystems.Vision.Vision;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
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
  public RobotContainer(SendableChooser<Command> autonChooser) {
    bBoard = new GenericHID(1);
    joy = new Joystick(0);
    input = new Input(joy);
    tankDrive = new CANDriveSubsystem();
    fs = new FuelShooter();
    intake = new IntakeSubsystem();
    vision = new Vision();

    autonChooser.setDefaultOption("LIMELIGHT SHOOT with distance", new SequentialCommandGroup(
      new DistanceDrive(tankDrive, -.75, -0.5),
      //new LockOnHub(tankDrive, vision),
      new ChargeMotor(fs, vision),
      new FeedFuel(intake)
    ));

    autonChooser.addOption("LIMELIGHT SHOOT with distance LEFT", new SequentialCommandGroup(
      new DistanceDrive(tankDrive, -.75, -0.5),
      new DegreeTurn(tankDrive, 45, true, 1.5),
      new LockOnHub(tankDrive, vision),
      new ChargeMotor(fs, vision),
      new FeedFuel(intake)
    ));

    autonChooser.addOption("LIMELIGHT SHOOT with distance RIGHT", new SequentialCommandGroup(
      new DistanceDrive(tankDrive, -1.25, -0.5),
      new LockOnHub(tankDrive, vision),
      new DegreeTurn(tankDrive, 45, false, 1.5),
      new ChargeMotor(fs, vision),
      new FeedFuel(intake)
    ));

    autonChooser.addOption("LIMELIGHT SHOOT with time", new SequentialCommandGroup(
      new AutoDrive(tankDrive, -0.5, 0).withTimeout(1),
      new LockOnHub(tankDrive, vision),
      new ChargeMotor(fs, vision),
      new FeedFuel(intake)
    ));

    autonChooser.addOption("Outpost", new SequentialCommandGroup(
      new DistanceDrive(tankDrive, -3.406 + Constants.DriveConstants.CHASSILENGTH, 1.75),
      new WaitCommand(2),
      new DistanceDrive(tankDrive, 1.7018 - Constants.DriveConstants.CHASSILENGTH, 1),
      new DegreeTurn(tankDrive, 135, true, 1.5),
      new ChargeMotor(fs, vision),
      new FeedFuel(intake)
    ));  

    autonChooser.addOption("Neutral Zone LEFT", new SequentialCommandGroup(
      new DistanceDrive(tankDrive, 4.72186, 2),
      new DegreeTurn(tankDrive, 90, true, 1.5),
      new ParallelRaceGroup(new DistanceDrive(tankDrive, 5.08, 0.5), new SetIntakeState(intake, IntakeState.INTAKE))
    ));

    autonChooser.addOption("Neutral Zone RIGHT", new SequentialCommandGroup(
      new DistanceDrive(tankDrive, 4.72186, 2),
      new DegreeTurn(tankDrive, 90, false, 1.5),
      new ParallelRaceGroup(new DistanceDrive(tankDrive, 5.08, 0.5), new SetIntakeState(intake, IntakeState.INTAKE))
    ));

    autonChooser.addOption("Center Block", new SequentialCommandGroup(
      new ParallelCommandGroup(new DistanceDrive(tankDrive, 185.9 * Constants.MathConstants.INCH_TO_METER, 2), new SetIntakeState(intake, IntakeState.INTAKE))
      
      
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

    //  new JoystickButton(joy, Constants.ButtonConstants.FLIP_FRONT_BUTTON_ID).whileFalse(teleJoyDrive = new TeleopJoystickDrive(tankDrive, input, false, -1));
     new JoystickButton(joy, Constants.ButtonConstants.FLIP_FRONT_BUTTON_ID).whileTrue(teleJoyDrive = new TeleopJoystickDrive(tankDrive, input, false, 1)).whileFalse(teleJoyDrive = new TeleopJoystickDrive(tankDrive, input, false, -1));
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
    new JoystickButton(joy, Constants.ButtonConstants.SHOOT_LIMELIGHT_BUTTON_ID).onTrue(new SequentialCommandGroup(/*new LockOnHub(tankDrive, vision),*/ new ChargeMotor(fs, vision), new FeedFuel(intake)));
    
    //shoot without vision
    new JoystickButton(bBoard, Constants.ButtonConstants.SHOOT_NO_LIMELIGHT_BUTTON_ID).onTrue(new SequentialCommandGroup( new ChargeMotor(fs, Constants.FuelShooterConstants.DEFAULT_SHOOTER_RPM), new FeedFuel(intake)));

    new JoystickButton(bBoard, Constants.ButtonConstants.INTAKE_BUTTON_ID).onTrue(new SetIntakeState(intake, IntakeState.INTAKE));

    new JoystickButton(bBoard, Constants.ButtonConstants.OUTTAKE_BUTTON_ID).onTrue(new SetIntakeState(intake, IntakeState.OUTTAKE)); 

    new JoystickButton(bBoard, Constants.ButtonConstants.STOP_INTAKE_BUTTON_ID).onTrue(new SetIntakeState(intake, IntakeState.NONE)); 
    //new JoystickButton(bBoard, Constants.ButtonConstants.FEED_INTAKE_BUTTON_ID).onTrue(new SetIntakeState(intake, IntakeState.FEED)); 

    new JoystickButton(bBoard, Constants.ButtonConstants.STOP_SHOOTER_BUTTON_ID).onTrue(new StopShooter(fs)); 

    new JoystickButton(bBoard,Constants.ButtonConstants.SHAKE_ROBOT_BUTTON_ID).onTrue(new SequentialCommandGroup(new AutoDrive(tankDrive, -2, 0).withTimeout(.1), new AutoDrive(tankDrive, 2, 0).withTimeout(.2)/* , teleJoyDrive = new TeleopJoystickDrive(tankDrive, input, false, -1)*/));

    new JoystickButton(bBoard, 10).onTrue(new ParallelCommandGroup(new FuelShooterCommand(fs, vision, FuelShooterState.REVERSE), new SetIntakeState(intake, IntakeState.SUCK)));
    // new JoystickButton(joy, 1).onTrue((new HubAimCommand(vision, fs))); //shoot with vision
    // new JoystickButton(joy, 2).onTrue(new ClimbCommand(climb, ClimbStates.L1)); //climb L1
    // new JoystickButton(joy, 2).onTrue(new ClimbCommand(climb, ClimbStates.DRIVE)); //climb to drive
    // new JoystickButton(joy, 2).whileTrue(new ResetClimb(climb)); //reset climb

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
