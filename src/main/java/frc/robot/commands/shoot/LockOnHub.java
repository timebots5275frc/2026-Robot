package frc.robot.commands.shoot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CANDriveSubsystem;
import frc.robot.subsystems.Vision.Vision;

public class LockOnHub extends Command {

    private CANDriveSubsystem drive;
    private Vision vision;

    private boolean lockedOn = false;

    public LockOnHub(CANDriveSubsystem drive, Vision vision) {
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
        if(
            vision.AprilTagID() == 18 || vision.AprilTagID() == 27 || vision.AprilTagID() == 26 || //blue
            vision.AprilTagID() == 25 || vision.AprilTagID() == 21 || vision.AprilTagID() == 24 || //blue
            vision.AprilTagID() == 5  || vision.AprilTagID() == 8  || vision.AprilTagID() == 9  || //red
            vision.AprilTagID() == 10 || vision.AprilTagID() == 11 || vision.AprilTagID() == 2  // //red
        ) {
            double allowedError = 100; //degrees //TODO 
            double kP = 0.1; // TODO is this neccasary
            double maxRot = 1; //TODO
            double tx = vision.HorizontalOffsetFromAprilTag(); 


            //STOP to not have wiggles
            if (Math.abs(tx) < allowedError) {
                drive.driveArcade(0, 0);
                lockedOn = true;
                return;
            }
            lockedOn = false;

             double correctionDeg = tx * kP;
             double correctionRad = Math.toRadians(correctionDeg);

             correctionRad = MathUtil.clamp(correctionRad, -maxRot, maxRot);

             drive.driveArcade(0, correctionRad);

            SmartDashboard.putBoolean("Locked in", lockedOn);
         }

        
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
        SmartDashboard.putBoolean("Locked in", lockedOn);
        return lockedOn;
    }

}