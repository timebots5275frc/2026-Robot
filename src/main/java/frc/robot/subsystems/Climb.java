package frc.robot.subsystems;


import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;

import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Climb extends SubsystemBase {

    private SparkMax climbMotor;
    private AbsoluteEncoder climbEncoder;
    private SparkClosedLoopController climbPID;
    private SparkMaxConfig smc;

    private double climbPose;

    private ClimbStates state;

    public enum ClimbStates {
        DRIVE,
        L1,
        RESET;
    }

    public Climb() {
        smc = new SparkMaxConfig();

        climbMotor = new SparkMax(Constants.ClimbConstants.MOTOR1_ID, MotorType.kBrushless);
        climbEncoder = climbMotor.getAbsoluteEncoder();

        climbMotor.configure(smc, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        climbPID = climbMotor.getClosedLoopController();
    }

    public void setState(ClimbStates state) {
        this.state = state;
        updateClimb(state);
    }


    private void updateClimb(ClimbStates state) {
        switch (state) {
            case DRIVE: if(climbPose == Constants.ClimbConstants.DRIVE){climbPID.setReference(0, ControlType.kVoltage);}
                        else{climbPID.setReference(Constants.ClimbConstants.DRIVE, ControlType.kPosition);}
            break;
            case L1: if(climbPose == Constants.ClimbConstants.L1){climbPID.setReference(0, ControlType.kVoltage);}
                     else{climbPID.setReference(Constants.ClimbConstants.L1, ControlType.kPosition);}
            break;
            case RESET: climbPID.setReference(Constants.ClimbConstants.RESET, ControlType.kCurrent);
            break;
            
        }
    }

    

    @Override
    public void periodic() {
        climbPose = climbEncoder.getPosition();
        SmartDashboard.putNumber("Climber pose", climbEncoder.getPosition());
    }

}
