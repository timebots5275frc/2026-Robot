// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;


import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.CustomTypes.Math.Vector2;
import frc.robot.subsystems.CANDriveSubsystem;
import frc.robot.subsystems.Input.Input;

public class TeleopJoystickDrive extends Command {

    private CANDriveSubsystem drivetrain;
    private Input input;

    //private Joystick driveStick;
    public boolean fieldRelative;
    private boolean usingJoystick;
    private double C;

    /**
     * Creates a new DefaultDrive.
     *
     * @param subsystem The drive subsystem this command wil run on.
     * @param joystick  The control input for driving
     */
    public TeleopJoystickDrive(CANDriveSubsystem drive, Input input_, boolean _fieldRelative) {
        this.drivetrain = drive;
        this.input = input_;
        this.fieldRelative = _fieldRelative;
        usingJoystick = this.input.usingJoystick;
        addRequirements(drive);

        C = 6;
    } 

    @Override
    public void initialize() {
        //AutoTargetStateManager.onStart();
        //drivetrain.resetPigeon();
    }

    @Override
    public void execute() {
        Vector2 moveInput;
        double turnInput;
        double speedPercent = 0;

        if (usingJoystick) {
            moveInput = input.JoystickInput();
            turnInput = input.JoystickTwist();
            speedPercent = (-input.getThrottle() + 1) / 2; // between 0 and 1 = 0% and 100%
        }
        else {
            moveInput = input.ControllerInput();
            
            turnInput = input.ControllerTurn();
            speedPercent = input.getControllerSpeed();
        }

        moveInput = new Vector2(
            MathUtil.applyDeadband(moveInput.x, Constants.DriveConstants.deadband),
            MathUtil.applyDeadband(moveInput.y, Constants.DriveConstants.deadband)
        );

        turnInput = MathUtil.applyDeadband(turnInput, Constants.DriveConstants.deadband);

        
        Vector2 inputVelocity = moveInput.times((speedPercent * Constants.DriveConstants.MAX_DRIVE_SPEED));
        double inputRotationVelocity = (turnInput * speedPercent * Constants.DriveConstants.MAX_TWIST_RATE)*.35; //rot. vel.
                                                                                                               //remove last multiplied number for max results
        
        int rot_sign = (int)(inputRotationVelocity / Math.abs(inputRotationVelocity));
        
        if(Math.abs(inputRotationVelocity)>Constants.DriveConstants.MAX_TWIST_RATE){
            System.out.println("IRV exceeds max twist rate");
            inputRotationVelocity=Constants.DriveConstants.MAX_TWIST_RATE*rot_sign;
        }
        SmartDashboard.putNumber("Throttle teleJoy", speedPercent);
        SmartDashboard.putNumber("Turn_speed", inputRotationVelocity);

        System.out.println(moveInput.x);

        drivetrain.driveArcade(inputVelocity.x, inputRotationVelocity);
    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    public void setFieldRelative(boolean bool) {
        this.fieldRelative = bool;
    }
}