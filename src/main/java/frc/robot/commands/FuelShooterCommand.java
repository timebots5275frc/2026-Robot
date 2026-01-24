// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.commands;

// import edu.wpi.first.wpilibj2.command.Command;
// import frc.robot.subsystems.FuelShooter;
// import frc.robot.subsystems.FuelShooter.Shooter;
// // import frc.robot.subsystems.FuelShooter.Shooter;
// import frc.robot.subsystems.Vision.Vision;

// /* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
// public class FuelShooterCommand extends Command {

  
//   private Vision vision;

//   private boolean end = false;

//   /** Creates a new FuelErectorCommand. */

//   private FuelShooter shooter;
  

//   public FuelShooterCommand(FuelShooter fs, Vision vision) {
    
    
//     // Use addRequirements() here to declare subsystem dependencies.

//     this.vision = vision;
//     this.shooter = fs;

//     addRequirements(shooter);
//     addRequirements(vision);
//   }

//   // Called when the command is initially scheduled.
//   @Override
//   public void initialize() {
//     double distance = vision.AprilTagPosInRobotSpace().magnitude();

//     double RPM = 4000;
//     double r = 4;
//     double velocity = (2*3.14*r*RPM)/60; //meters per second

//     shooter.setState(Shooter.SHOOT);
//     shooter.setShooterRPM(RPM);
//   }

//   // Called every time the scheduler runs while the command is scheduled.
//   @Override
//   public void execute() {
//       if (shooter.shooterAtTargetRPM()) {
//         shooter.feedFuel();
//       } else {
//         shooter.stopFeedingFuel();
//       }
//   }

//   // Called once the command ends or is interrupted.
//   @Override
//   public void end(boolean interrupted) {

//     // shooter.setShooterRPM(0);
//     // shooter.setState(Shooter.NONE);
//   }

//   // Returns true when the command should end.
//   @Override
//   public boolean isFinished() {
//     return end;
//   }
// }
