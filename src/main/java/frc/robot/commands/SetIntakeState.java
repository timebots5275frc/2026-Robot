package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.IntakeSubsystem.IntakeState;

public class SetIntakeState extends InstantCommand {
  private IntakeSubsystem intake;
  private IntakeState state;
    public SetIntakeState(IntakeSubsystem intake, IntakeState state) {
        this.intake = intake;
        this.state = state;
        addRequirements(intake);
    }
    
    @Override
    public void initialize() {
        intake.setIntakeState(state);
    }
}
