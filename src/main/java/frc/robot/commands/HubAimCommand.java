package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.CANDriveSubsystem;

import frc.robot.subsystems.Vision.Vision;

public class HubAimCommand extends Command {

    private CANDriveSubsystem drive;
    private Vision vision;

    public HubAimCommand(CANDriveSubsystem drive, Vision vision) {
        this.drive = drive;
        this.vision = vision;

        addRequirements(drive);
        addRequirements(vision);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        vision.setUsingLimelight(true);
    }

    // Called every time the scheduler runs while the command is scheduled.
    
    @Override
    public void execute() {

        if (!vision.hasValidData()) {
            drive.driveArcade(0, 0);
            return;
        }
        double allowedError = 1.0; //degrees //TODO 
        double kP = 0.1; // TODO
        double maxRot = 1; //TODO
        double tx = vision.HorizontalOffsetFromAprilTag(); 


        //STOP to not have wiggles
        if (Math.abs(tx) < allowedError) {
            drive.driveArcade(0,0);
            return;
        }

        double correctionDeg = tx * kP;
        double correctionRad = Math.toRadians(correctionDeg);

        correctionRad = MathUtil.clamp(correctionRad, -maxRot, maxRot);

        drive.driveArcade( 0, correctionRad);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        vision.setUsingLimelight(false);
        drive.driveArcade(0, 0);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }

}
