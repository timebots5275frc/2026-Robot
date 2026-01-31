// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shoot;

import com.revrobotics.spark.SparkBase.ControlType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.FuelShooter;
import frc.robot.subsystems.Vision.Vision;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ChargeMotor extends Command {
  /** Creates a new ChargeMotor. */

  Vision vision;
  FuelShooter shooter;
  double targetRPM;

  int allowedError = 5;

  public ChargeMotor(FuelShooter shooter, Vision vision) {
    this.vision = vision;
    this.shooter = shooter;

    addRequirements(vision);
    addRequirements(shooter);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

        vision.setUsingLimelight(true);

        
        // double x0 = vision.RobotPosInFieldSpace().x; // robot X pose
        // double y0 = vision.RobotPosInFieldSpace().y; // robot Y pose
        double thetaRad = 0.977384; //launch angle in radians
        double thetaDeg = Math.toDegrees(thetaRad); //launch angle in degrees
        double tx = vision.HorizontalOffsetFromAprilTag(); //target pose
        double ty = vision.AprilTagRotInRobotSpace().y;
        double dx = vision.AprilTagPosInRobotSpace().magnitude();
        

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

                 targetRPM = shooter.calculateRPMFromLimelight(tx,ty,thetaRad,dx);


                 shooter.shooterPID.setReference(targetRPM, ControlType.kVelocity);


                 //prints distance and target rpm
                 
                 SmartDashboard.putNumber("Shooter Distance", shooter.dx);
                 SmartDashboard.putNumber("Shooter RPM (calc)", targetRPM);
                 System.out.println("target RPM " + targetRPM);
               }
        }
        if(vision.hasValidData() == false){
            // System.out.println(vision.AprilTagID() + "no valid data");
            // drive.driveArcade(0, 0);
            shooter.shooterPID.setReference(100, ControlType.kVelocity);
        }
        SmartDashboard.putBoolean("Charging Motor", true);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    SmartDashboard.putNumber("Encoder Velocity", shooter.getMotor().getEncoder().getVelocity());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    vision.setUsingLimelight(false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() { 
    if (targetRPM == 0) {
      return false;
    }
     SmartDashboard.putBoolean("Charging Motor", true);
    if (shooter.getMotor().getEncoder().getVelocity() >= targetRPM - allowedError && shooter.getMotor().getAbsoluteEncoder().getVelocity() <= targetRPM + allowedError) {
      return true;
    } 
    return false;
    
  }
}
