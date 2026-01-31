package frc.robot.commands;

import com.revrobotics.spark.SparkBase.ControlType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
// import frc.robot.subsystems.CANDriveSubsystem;
import frc.robot.subsystems.FuelShooter;
import frc.robot.subsystems.Vision.Vision;

public class HubAimCommand extends Command {

    private FuelShooter fs;
    // private CANDriveSubsystem drive;
    private Vision vision;
    // private SlewRateLimiter srl = new SlewRateLimiter(10);
    double tx = 0;
    double ty = 0;

    public HubAimCommand(Vision vision, FuelShooter fs) {
        // this.drive = drive;
        this.vision = vision;
        this.fs = fs;

        // addRequirements(drive);
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
        
        // if(vision.AprilTagRotInRobotSpace() == null){ty = 0;} //doesnt work becuase cant  from double to null
        // else if(vision.AprilTagRotInRobotSpace().y >= 0){ty = vision.AprilTagRotInRobotSpace().y;} //target pose
        ty = vision.AprilTagRotInRobotSpace().y; //target pose
        double targetRPM; //target rpm
        // double tx = vision.HorizontalOffsetFromAprilTag(); 

        if(vision.hasValidData() == true){
            // System.out.println("See April tag " + vision.AprilTagID());
            //18,27,26,25,21,24 - blue hub
            //5,8,9,10,11,2 - red hub
            if(
                vision.AprilTagID() == 18 || vision.AprilTagID() == 27 || vision.AprilTagID() == 26 || //blue
                vision.AprilTagID() == 25 || vision.AprilTagID() == 21 || vision.AprilTagID() == 24 || //blue
                vision.AprilTagID() == 5  || vision.AprilTagID() == 8  || vision.AprilTagID() == 9  || //red
                vision.AprilTagID() == 10 || vision.AprilTagID() == 11 || vision.AprilTagID() == 2  // //red
              ){
                //  System.out.println("See April tag " + vision.AprilTagID());
                 targetRPM = fs.calculateRPMFromLimelight(tx,ty,thetaRad, 1);
                 fs.shooterPID.setReference(targetRPM, ControlType.kVelocity);
                 //prints distance and target rpm
                 SmartDashboard.putNumber("Shooter Distance", fs.dx);
                 SmartDashboard.putNumber("Shooter RPM (calc)", targetRPM);
                 System.out.println("target RPM " + targetRPM);
               }
        }
        if(vision.hasValidData() == false){
            // System.out.println(vision.AprilTagID() + "no valid data");
            // drive.driveArcade(0, 0);
            fs.shooterPID.setReference(100, ControlType.kVelocity);
        }

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
