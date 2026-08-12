// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.Constants;
import frc.robot.CustomTypes.Math.Vector2;
import frc.robot.subsystems.CANDriveSubsystem;
import frc.robot.subsystems.Input.Input;

/**
 * Teleop arcade drive.
 *
 * Pipeline order (this order matters):
 *   raw stick -> deadband+rescale -> response curve -> slew limit -> scale to velocity -> clamp -> drive
 *
 * Note that the slew limiters run on NORMALIZED (-1..1) input, not on m/s. That makes
 * JOY_X_RATE_LIMIT / JOY_TURN_RATE_LIMIT mean "fraction of full stick travel per second",
 * which is far easier to tune. A value of 3.0 means full travel in 1/3 second.
 * You will need to re-tune those two constants after this change.
 */
public class TeleopJoystickDrive extends Command {

    /**
     * Response curve blend. 0.0 = fully linear, 1.0 = fully cubic.
     * Pure cubic (1.0) feels dead near center and abrupt near the ends.
     * 0.6-0.8 gives fine low-speed control while still reaching full output smoothly.
     */
    private static final double DRIVE_CURVE = 0.3;
    private static final double TURN_CURVE = 0.3;

    /**
     * How much throttle scales turning. 1.0 = turn rate fully scales with throttle
     * (original behavior - you can barely rotate at low throttle).
     * 0.0 = turn authority is independent of throttle.
     * ~0.5 is a good starting point.
     */
    private static final double TURN_THROTTLE_SCALE = 0.2;

    private final CANDriveSubsystem drivetrain;
    private final Input input;

    private final SlewRateLimiter srlX =
            new SlewRateLimiter(Constants.JoystickConstants.JOY_X_RATE_LIMIT);
    private final SlewRateLimiter srlTurn =
            new SlewRateLimiter(Constants.JoystickConstants.JOY_TURN_RATE_LIMIT);

    private boolean fieldRelative;
    private final int front;

    /**
     * @param drive         the drive subsystem this command will run on
     * @param input_        the control input for driving
     * @param _fieldRelative whether to drive field-relative
     * @param front         +1 or -1, which end of the robot is "forward"
     */
    public TeleopJoystickDrive(CANDriveSubsystem drive, Input input_, boolean _fieldRelative, int front) {
        this.drivetrain = drive;
        this.input = input_;
        this.fieldRelative = _fieldRelative;
        this.front = front;
        addRequirements(drive);
    }

    @Override
    public void initialize() {
        // Deadband is applied manually below, so don't let DifferentialDrive apply it twice.
        drivetrain.getDifferentialDrive().setDeadband(0);

        // Clear stale limiter state left over from the last time this command ran.
        // Without this, the first frame after enable can command a large jump.
        srlX.reset(0);
        srlTurn.reset(0);
    }

    @Override
    public void execute() {
        // Read inputs into locals. Do not mutate the Vector2 that Input handed us --
        // it may be cached or shared with other consumers.
        Vector2 rawMove = input.DriveInput();
        double rawTurn = input.DriveTwist();
        double speedPercent = input.DriveSpeedPercent();

        // 1. Deadband + rescale so output is continuous: 0 at the deadband edge, 1 at full stick.
        double move = deadband(rawMove.x, Constants.DriveConstants.DEAD_BAND_DRIVE);
        double turn = deadband(rawTurn, Constants.DriveConstants.DEAD_BAND_STEER);

        // 2. Response curve (still normalized -1..1).
        move = curve(move, DRIVE_CURVE);
        turn = curve(turn, TURN_CURVE);

        // 3. Slew limit in normalized units. Called EVERY loop, including when the
        //    stick is centered, so the limiter ramps down to zero instead of snapping
        //    and so its internal state never goes stale.
        move = srlX.calculate(move * front);
        turn = srlTurn.calculate(turn);

        // 4. Scale to real velocities.
        double throttleForTurn = MathUtil.interpolate(1.0, speedPercent, TURN_THROTTLE_SCALE);

        double velocity = move
                * speedPercent
                * Constants.DriveConstants.MAX_DRIVE_SPEED
                * Constants.JoystickConstants.JOY_INPUT_VELOCITY_MULT;

        double rotationVelocity = turn
                * throttleForTurn
                * Constants.DriveConstants.MAX_TWIST_RATE
                * Constants.JoystickConstants.JOY_INPUT_ROTATION_VELOCITY_MULT;

        // 5. Clamp both axes symmetrically.
        velocity = MathUtil.clamp(
                velocity,
                -Constants.DriveConstants.MAX_DRIVE_SPEED,
                Constants.DriveConstants.MAX_DRIVE_SPEED);

        rotationVelocity = MathUtil.clamp(
                rotationVelocity,
                -Constants.DriveConstants.MAX_TWIST_RATE,
                Constants.DriveConstants.MAX_TWIST_RATE);

        drivetrain.driveArcade(velocity, rotationVelocity);
    }

    /**
     * Deadband with rescale. Output is 0 at |value| == deadband and +/-1 at |value| == 1,
     * with no discontinuity at the edge.
     *
     * The original code was: value - (signum(value) * db) / (1 - db)
     * Java precedence made the division apply only to the second term, which produced a
     * sign inversion just outside the deadband. The parentheses below are the fix.
     */
    private static double deadband(double value, double db) {
        if (Math.abs(value) <= db) {
            return 0.0;
        }
        return (value - Math.copySign(db, value)) / (1.0 - db);
    }

    /**
     * Blend between linear and cubic response. Sign is always preserved.
     *
     * @param value    normalized input, -1..1
     * @param cubicness 0 = linear, 1 = cubic
     */
    private static double curve(double value, double cubicness) {
        return cubicness * (value * value * value) + (1.0 - cubicness) * value;
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.driveArcade(0, 0);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    public void setFieldRelative(boolean bool) {
        this.fieldRelative = bool;
    }
}