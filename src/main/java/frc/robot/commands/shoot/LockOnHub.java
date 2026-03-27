// package frc.robot.commands.shoot;

// import java.util.Optional;

// import com.ctre.phoenix6.hardware.Pigeon2;
// import com.revrobotics.spark.SparkBase.ControlType;

// import edu.wpi.first.math.MathUtil;
// import edu.wpi.first.math.geometry.Pose3d;
// import edu.wpi.first.math.geometry.Rotation3d;
// import edu.wpi.first.math.geometry.Translation3d;
// import edu.wpi.first.wpilibj.DriverStation;
// import edu.wpi.first.wpilibj.DriverStation.Alliance;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj2.command.Command;
// import frc.robot.Constants.Constants;
// import frc.robot.Constants.VisionConstants;
// import frc.robot.CustomTypes.Math.Vector3;
// import frc.robot.subsystems.CANDriveSubsystem;
// import frc.robot.subsystems.FuelShooter;
// import frc.robot.subsystems.FuelShooter.FuelShooterState;
// import frc.robot.subsystems.Vision.Vision;

// public class LockOnHub extends Command {

//     private CANDriveSubsystem drive;
//     private Vision vision;
//     private FuelShooter shooter;

//     private double targetRPM = 0;
//     private boolean lockedOn = false;

//     private double angleTolerance = Math.toRadians(5);
//     private double rpmTolerance = 100;

//     private double dx = 0;
//     private double dy = 0;

//     private double targetHeading = 0;

//     //private Pigeon2 gyro = new Pigeon2(Constants.DriveConstants.PIGEON_2_ID);

//     public LockOnHub(CANDriveSubsystem drive, Vision vision, FuelShooter shooter) {
//         this.drive = drive;
//         this.vision = vision;
//         this.shooter = shooter;

//         addRequirements(drive, vision, shooter);
//     }

//     @Override
//     public void initialize() {
//         vision.setUsingLimelight(true);

//         vision.gyro.reset();

//         Optional<Alliance> alliance = DriverStation.getAlliance();
//         Translation3d targetPose =
//             alliance.isPresent() && alliance.get() == Alliance.Blue
//                 ? VisionConstants.BLUE_HUB_POSE
//                 : VisionConstants.RED_HUB_POSE;

//         Vector3 robotPos = vision.RobotPosInFieldSpace();

//         dx = targetPose.getX() - robotPos.x;
//         dy = targetPose.getY() - robotPos.y;

//         Vector3 robotLocation = vision.RobotPosInFieldSpace();
//         Vector3 robotRot = vision.RobotRotInFieldSpace();

//         Pose3d robotPose = new Pose3d(robotLocation.x, robotLocation.y, robotLocation.z, new Rotation3d(robotRot.x, robotRot.y, robotRot.z));
//         Pose3d shooterPose = robotPose.transformBy(VisionConstants.ROBOT_TO_TURRET);

//         double angleToTag = Math.atan2(dy, dx);

//         // targetHeading = Math.toRadians( Math.toDegrees(shooterPose.getRotation().getZ()) + angleToTag);
//         targetHeading = angleToTag;

//     }

//     @Override
//     public void execute() {

//     if (!vision.hasValidData()) { //SPIN
//             drive.driveArcade(0, 0.2); // good for auto but not tele - op 
//                                           // doesnt allow for driver control if april tag isnt seen so best case should just be, do nothing
//             return;
//         }
//         Vector3 robotLocation = vision.RobotPosInFieldSpace();
//         Vector3 robotRot = vision.RobotRotInFieldSpace();

//         Pose3d robotPose = new Pose3d(robotLocation.x, robotLocation.y, robotLocation.z, new Rotation3d(robotRot.x, robotRot.y, robotRot.z));
//         Pose3d shooterPose = robotPose.transformBy(VisionConstants.ROBOT_TO_TURRET);

//         Optional<Alliance> alliance =  DriverStation.getAlliance();
//         Translation3d targetPose = alliance.isPresent() && alliance.get().equals(Alliance.Blue) ? VisionConstants.BLUE_HUB_POSE : VisionConstants.RED_HUB_POSE;

//         dx = targetPose.getX() - shooterPose.getX();
//         dy = targetPose.getY() - shooterPose.getY();

//         double angleToTag = Math.atan2(dy, dx);


//         // double angleToTag = Math.atan(dy/dx);

//         // this should be the angle of the robot to be compared to target angle

//         // TODO use this angle to look at the tag.

//         double robotHeading = shooterPose.getRotation().getZ();
//         // // Rotation2d robotHeading = gyro.getRotation2d();
//         // // double error = robotHeading - angleToTag;
//         double error = targetHeading - robotHeading;
//         error = Math.atan2(Math.sin(error), Math.cos(error));

//         // double heading = shooterPose.getRotation().getZ();

//         // double cos = Math.cos(-heading);
//         // double sin = Math.sin(-heading);

//         // double robotX = dx * cos - dy * sin;
//         // double robotY = dx * sin + dy * cos;

//         double kP = .4; 
//         double maxRot = 10; 
//         // Math.abs(robotY) < 0.05

//         //ready to shoot
//         if (Math.abs(vision.RobotYawRadians()) >= angleToTag) {
//             drive.driveArcade(0, 0);
//             lockedOn = true;

//             // double distance = Math.hypot(dx, dy);
//             double distance = vision.AprilTagPosInRobotSpace().magnitude() + Constants.CalculateShooterRpmConstants.CAMERA_OFFSET;
//             targetRPM = -shooter.calculateRPMFromLimelight(distance);
//             // SmartDashboard.putNumber("target rpm", targetRPM);
//             // SmartDashboard.putNumber("distance", distance);


//             // shooter.shooterMotorPID.setReference(-targetRPM , ControlType.kVelocity);


//             //  prints distance and target rpm
                    
//             //  SmartDashboard.putNumber("Shooter Distance", shooter.dx);
//             // SmartDashboard.putNumber("Shooter RPM (calc)", targetRPM);
//            return;
//         }

//         lockedOn = false;

//              double correctionRad = error * kP;

//              correctionRad = MathUtil.clamp(correctionRad, -maxRot, maxRot);

//              drive.driveArcade(0, correctionRad);

//         // SmartDashboard.putBoolean("Locked in", lockedOn);
//         // SmartDashboard.putNumber("error", Math.toDegrees(error));
//         // SmartDashboard.putNumber("angle to tag", angleToTag);
//         // // SmartDashboard.putNumber("robot Heading", robotHeading);
//         // SmartDashboard.putNumber("target heading", targetHeading);
//         // SmartDashboard.putNumber("dx", dx);
//         // SmartDashboard.putNumber("dy", dy);
//         // SmartDashboard.putNumber("shooter pose x", shooterPose.getX());
//         // SmartDashboard.putNumber("shooter pose y", shooterPose.getY());
//         // SmartDashboard.putNumber("target pose x", targetPose.getX());
//         // SmartDashboard.putNumber("target pose y", targetPose.getY());

//     }

//     @Override
//     public void end(boolean interrupted) {
//         vision.setUsingLimelight(false);
//         vision.ClearAprilTagData();
//         if (interrupted) {
//             shooter.SetShooterState(FuelShooterState.NONE);
//         }
//     }

//     @Override
//     public boolean isFinished() {
//         if (targetRPM == 0) return false;

//         // SmartDashboard.putBoolean("Charging Motor", true);

//         if (Math.abs(shooter.getMotor().getEncoder().getVelocity() - targetRPM) < rpmTolerance) {
//             // SmartDashboard.putBoolean("Charging Motor", false);
//             return true;
//         }

//         return false;
//     }
// }