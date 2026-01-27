package frc.robot.commands;

import com.revrobotics.spark.SparkBase.ControlType;

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
    // private SlewRateLimiter srl = new SlewRateLimiter(10);
    double tx = 0;
    double ty = 0;

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

        double rpm; //current rpm
        // double x0 = vision.RobotPosInFieldSpace().x; // robot X pose
        // double y0 = vision.RobotPosInFieldSpace().y; // robot Y pose
        double thetaRad = 0.977384; //launch angle in radians
        double thetaDeg = Math.toDegrees(thetaRad); //launch angle in degrees
        tx = vision.HorizontalOffsetFromAprilTag(); //target pose
        
        // if(vision.AprilTagRotInRobotSpace().y == null){ty = 0;} //doesnt work becuase cant  from double to null
        // else if(vision.AprilTagRotInRobotSpace().y >= 0){ty = vision.AprilTagRotInRobotSpace().y;} //target pose
        ty = vision.AprilTagRotInRobotSpace().y; //target pose
        double targetRPM; //target rpm

        double allowedError = 1.0; //degrees //TODO 
        double kP = 0.1; // TODO
        double maxRot = 1; //TODO
        // double tx = vision.HorizontalOffsetFromAprilTag(); 

        if(vision.hasValidData() == true){
            System.out.println("See April tag " + vision.AprilTagID());
            targetRPM = fs.calculateRPMFromLimelight(tx,ty,thetaRad);
            fs.ShooterPID.setReference(targetRPM, ControlType.kVelocity);
            //prints distance and target rpm
            SmartDashboard.putNumber("Shooter Distance", fs.dx);
            SmartDashboard.putNumber("Shooter RPM (calc)", targetRPM);
        }
        if(vision.hasValidData() == false){
            System.out.println("No valid data");
            drive.driveArcade(0, 0);
            fs.ShooterPID.setReference(4000, ControlType.kVelocity);
        }

        //dont need because tank drive
        //STOP to not have wiggles
        // if (Math.abs(tx) < allowedError) {
        //     drive.driveArcade(0,0);
        //     return;
        // }

        // double correctionDeg = tx * kP;
        // double correctionRad = Math.toRadians(correctionDeg);

        // correctionRad = MathUtil.clamp(correctionRad, -maxRot, maxRot);

        // drive.driveArcade( 0, correctionRad);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        vision.setUsingLimelight(false);
        //dont need because tank drive
        // drive.driveArcade(0, 0);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }

}
