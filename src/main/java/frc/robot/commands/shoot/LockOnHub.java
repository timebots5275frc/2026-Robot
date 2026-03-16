package frc.robot.commands.shoot;

import java.util.Optional;

import com.revrobotics.spark.SparkBase.ControlType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.VisionConstants;
import frc.robot.CustomTypes.Math.Vector3;
import frc.robot.subsystems.CANDriveSubsystem;
import frc.robot.subsystems.FuelShooter;
import frc.robot.subsystems.FuelShooter.FuelShooterState;
import frc.robot.subsystems.Vision.Vision;

public class LockOnHub extends Command {

    private CANDriveSubsystem drive;
    private Vision vision;
    private FuelShooter shooter;
    private double targetRPM = 0;
    private boolean lockedOn = false;
    private double angleTolerance = Math.toRadians(5);
    private double rpmTolerance = 100;

    public LockOnHub(CANDriveSubsystem drive, Vision vision, FuelShooter shooter) {
       this.drive = drive;
        this.vision = vision;
        this.shooter = shooter;

        addRequirements(drive);
        addRequirements(vision);
        addRequirements(shooter);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        vision.setUsingLimelight(true);
    }

    // Called every time the scheduler runs while the command is scheduled.
    
    @Override
    public void execute() {

        if (!vision.hasValidData()) { //SPIN
            drive.driveArcade(0, .4); 
            return;
        }

        Vector3 robotLocation = vision.RobotPosInFieldSpace();
        Vector3 robotRot = vision.RobotRotInFieldSpace();

        Pose3d robotPose = new Pose3d(robotLocation.x, robotLocation.y, robotLocation.z, new Rotation3d(robotRot.x, robotRot.y, robotRot.z));

        Pose3d shooterPose = robotPose.transformBy(VisionConstants.ROBOT_TO_TURRET);

        Optional<Alliance> alliance =  DriverStation.getAlliance();

        Translation3d targetPose = alliance.isPresent() && alliance.get().equals(Alliance.Blue) ? VisionConstants.BLUE_HUB_POSE : VisionConstants.RED_HUB_POSE;

        double dx = targetPose.getX() - shooterPose.getX();
        double dy = targetPose.getY() - shooterPose.getY();

        double angleToTag = Math.atan2(dy, dx);

        // this should be the angle of the robot to be compared to target angle

        // TODO use this angle to look at the tag.

        double robotHeading = shooterPose.getRotation().getZ();
        // double error = angleToTag - robotHeading;
        // error = Math.atan2(Math.sin(error), Math.cos(error));
        double error = MathUtil.angleModulus(angleToTag - robotHeading); //does same thing as previous 2 lines


        double kP = 2.0; 
        double maxRot = 1; 


        //STOP to not have wiggles
        if (Math.abs(error) < angleTolerance) {
            drive.driveArcade(0, 0);
            lockedOn = true;

            double distance = Math.hypot(dx, dy);
            targetRPM = -shooter.calculateRPMFromLimelight(distance);
            SmartDashboard.putNumber("target rpm", targetRPM);


            shooter.shooterMotorPID.setReference(targetRPM , ControlType.kVelocity);


        //  prints distance and target rpm
                
        //  SmartDashboard.putNumber("Shooter Distance", shooter.dx);
            SmartDashboard.putNumber("Shooter RPM (calc)", targetRPM);
            return;
        }
            lockedOn = false;

             double correctionRad = error * kP;

             correctionRad = MathUtil.clamp(correctionRad, -maxRot, maxRot);

             drive.driveArcade(0, correctionRad);

            SmartDashboard.putBoolean("Locked in", lockedOn);

    }
        

        

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        
        vision.setUsingLimelight(false);
        if(interrupted) {
            shooter.SetShooterState(FuelShooterState.NONE);
        }
       // drive.driveArcade(0, 0);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        if (targetRPM == 0) {
            return false;
        }
        SmartDashboard.putBoolean("Charging Motor", true);
    
        if (Math.abs(shooter.getMotor().getEncoder().getVelocity() - targetRPM) < rpmTolerance) {
            SmartDashboard.putBoolean("Charging Motor", false);
            return true;
        } 
        return false;
    }

}