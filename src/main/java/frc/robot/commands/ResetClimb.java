// package frc.robot.commands;

// import edu.wpi.first.wpilibj2.command.Command;
// import frc.robot.subsystems.Climb;
// import frc.robot.subsystems.Climb.ClimbStates;

// public class ResetClimb extends Command {

//     Climb climb;
    

//     public ResetClimb(Climb climb){
//         this.climb = climb;
        
//         addRequirements(climb);
//     }

//     @Override
//     public void initialize() {
//         climb.setState(ClimbStates.RESET);
//     }

//     // Called every time the scheduler runs while the command is scheduled.
//     @Override
//     public void execute() {
        
//     }

//     // Called once the command ends or is interrupted.
//     @Override
//     public void end(boolean interrupted) {
//         //set encoder to zero
//     }

//     // Returns true when the command should end.
//     @Override
//     public boolean isFinished() {
//         return false;
//     }
// }


