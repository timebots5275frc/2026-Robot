package frc.robot.commands.shoot;

import java.util.Optional;

import com.revrobotics.spark.SparkBase.ControlType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.Constants;
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

    private double dx = 0;
    private double dy = 0;

    public LockOnHub(CANDriveSubsystem drive, Vision vision, FuelShooter shooter) {
        this.drive = drive;
        this.vision = vision;
        this.shooter = shooter;

        addRequirements(drive, vision, shooter);
    }

    @Override
    public void initialize() {
        vision.setUsingLimelight(true);
    }

    @Override
    public void execute() {

        if (!vision.hasValidData()) return;

        // =========================
        // GET ROBOT STATE
        // =========================
        Vector3 robotPos = vision.RobotPosInFieldSpace();
        double robotHeading = vision.RobotYawRadians();

        // =========================
        // GET TARGET (FIELD SPACE)
        // =========================
        Optional<Alliance> alliance = DriverStation.getAlliance();

        Translation3d targetPose =
            alliance.isPresent() && alliance.get() == Alliance.Blue
                ? VisionConstants.BLUE_HUB_POSE
                : VisionConstants.RED_HUB_POSE;

        dx = targetPose.getX() - robotPos.x;
        dy = targetPose.getY() - robotPos.y;

        // =========================
        // ANGLE CALC
        // =========================
        double angleToTarget = Math.atan2(dy, dx);

        double error = angleToTarget - robotHeading;
        error = Math.atan2(Math.sin(error), Math.cos(error)); // normalize

        // =========================
        // TURN CONTROL
        // =========================
        double kP = 0.1;
        double maxRot = 0.5;

        if (Math.abs(error) < angleTolerance) {

            lockedOn = true;

            // =========================
            // DISTANCE + RPM
            // =========================
            double distance = Math.hypot(dx, dy);

            targetRPM = -shooter.calculateRPMFromLimelight(distance);

            shooter.shooterMotorPID.setReference(targetRPM, ControlType.kVelocity);

            SmartDashboard.putNumber("Distance", distance);
            SmartDashboard.putNumber("Target RPM", targetRPM);

            return;
        }

        lockedOn = false;

        double correction = MathUtil.clamp(error * kP, -maxRot, maxRot);
        drive.driveArcade(0, correction);

        // =========================
        // DEBUG
        // =========================
        SmartDashboard.putBoolean("Locked On", lockedOn);
        SmartDashboard.putNumber("Error (deg)", Math.toDegrees(error));
        SmartDashboard.putNumber("Angle To Target", Math.toDegrees(angleToTarget));
        SmartDashboard.putNumber("Robot Heading", Math.toDegrees(robotHeading));
        SmartDashboard.putNumber("dx", dx);
        SmartDashboard.putNumber("dy", dy);
    }

    @Override
    public void end(boolean interrupted) {
        vision.setUsingLimelight(false);

        if (interrupted) {
            shooter.SetShooterState(FuelShooterState.NONE);
        }
    }

    @Override
    public boolean isFinished() {
        if (targetRPM == 0) return false;

        SmartDashboard.putBoolean("Charging Motor", true);

        if (Math.abs(shooter.getMotor().getEncoder().getVelocity() - targetRPM) < rpmTolerance) {
            SmartDashboard.putBoolean("Charging Motor", false);
            return true;
        }

        return false;
    }
}