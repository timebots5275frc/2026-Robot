package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CANDriveSubsystem;
import frc.robot.subsystems.FuelShooter;
import frc.robot.subsystems.Vision.Vision;

public class HubAimCommand extends Command {

    private FuelShooter fs;
    private CANDriveSubsystem drive;
    private Vision vision;
    private SlewRateLimiter srl = new SlewRateLimiter(10);

    public HubAimCommand(CANDriveSubsystem drive, Vision vision, FuelShooter fs) {
        this.drive = drive;
        this.vision = vision;
        this.fs = fs;

        addRequirements(drive);
        addRequirements(vision);
        addRequirements(fs);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        vision.setUsingLimelight(true);
    }

    // Called every time the scheduler runs while the command is scheduled.
    
    @Override
    public void execute() {

        double rpm; 
        double g = 9.8; // gravity
        double r = 0.1016; // radius of wheel launching ball
        double x0 = vision.RobotPosInFieldSpace().x; // robot pose
        double theta = Math.toDegrees(0.977384); //angle in radians
        double y0 = vision.RobotPosInFieldSpace().y; // robot pose
        double tx = vision.HorizontalOffsetFromAprilTag(); //target pose
        double ty = vision.AprilTagRotInRobotSpace().y; //target pose
        double cameraHeight = 0; //look into limelight offset
        double targetHeight = 0;
        double mountingAngle = 0;
        double targetRPM;

        double allowedError = 1.0; //degrees //TODO 
        double kP = 0.1; // TODO
        double maxRot = 1; //TODO
        // double tx = vision.HorizontalOffsetFromAprilTag(); 

        if(vision.hasValidData() == true){
            System.out.println("See April tag " + vision.AprilTagID());
            targetRPM = fs.calculateRPMFromLimelight(ty, cameraHeight, targetHeight, mountingAngle, theta, r, g);
            rpm = srl.calculate(targetRPM);
            SmartDashboard.putNumber("Shooter Distance", fs.dx);
            SmartDashboard.putNumber("Shooter RPM (calc)", targetRPM);
        }
        if(vision.hasValidData() == false){
            System.out.println("No valid data");
            drive.driveArcade(0, 0);
            rpm = 4000;
        }

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
