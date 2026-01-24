package frc.robot.subsystems;


import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;

import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Climb extends SubsystemBase {

    private SparkMax motorLeft;
    private SparkMax motorRight;

    private AbsoluteEncoder encoder;

    private SparkClosedLoopController rightPID;
    private SparkClosedLoopController leftPID;

    private SparkMaxConfig smc;

    private ClimbStates state;

    public Climb() {
        smc = new SparkMaxConfig();

        motorLeft = new SparkMax(Constants.ClimbConstants.MOTOR1_ID, MotorType.kBrushless);
        motorRight = new SparkMax(Constants.ClimbConstants.MOTOR2_ID, MotorType.kBrushless);

        encoder = motorLeft.getAbsoluteEncoder();

        smc.follow(motorLeft);
        smc.inverted(true);
        motorRight.configure(smc, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        leftPID = motorLeft.getClosedLoopController();

    }

    public enum ClimbStates {
        DRIVE,
        L1,
        Reset
    }

    public void setState(ClimbStates state) {
        this.state = state;
        updateClimb(state);
    }


    private void updateClimb(ClimbStates state) {
        switch (state) {
            case DRIVE: leftPID.setReference(Constants.ClimbConstants.DRIVE, ControlType.kPosition);
                break;
            case L1: leftPID.setReference(Constants.ClimbConstants.L1, ControlType.kPosition);
                break;
            case Reset: leftPID.setReference(Constants.ClimbConstants.RESET, ControlType.kCurrent);
                break;
            
        }
    }

    

    @Override
    public void periodic() {
        encoder.getPosition();
        SmartDashboard.putNumber("Climber pose", encoder.getPosition());
    }

}
