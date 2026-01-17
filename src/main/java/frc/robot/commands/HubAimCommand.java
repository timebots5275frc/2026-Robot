package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.SwerveDrive.SwerveDrive;
import frc.robot.subsystems.Vision.Vision;

public class HubAimCommand extends Command {

    private SwerveDrive swerve;
    private Vision vision;

    public HubAimCommand(SwerveDrive swerve, Vision vision) {
        this.swerve = swerve;
        this.vision = vision;

        addRequirements(swerve);
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
            swerve.drive(0, 0, 0, false);
            return;
        }
        double allowedError = 1.0; //degrees //TODO 
        double kP = 0.1; // TODO
        double maxRot = 1; //TODO
        double tx = vision.HorizontalOffsetFromAprilTag(); 


        //STOP to not have wiggles
        if (Math.abs(tx) < allowedError) {
            swerve.drive(0, 0, 0, false);
            return;
        }

        double correctionDeg = tx * kP;
        double correctionRad = Math.toRadians(correctionDeg);

        correctionRad = MathUtil.clamp(correctionRad, -maxRot, maxRot);

        swerve.drive(0, 0, correctionRad, false);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        vision.setUsingLimelight(false);
        swerve.drive(0, 0, 0, false);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }

}
