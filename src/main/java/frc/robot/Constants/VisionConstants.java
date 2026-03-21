// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Constants;

import java.util.List;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import frc.robot.CustomTypes.Math.Vector2;

/** Add your docs here. */
public class VisionConstants {



	// public static final boolean ENABLE_LIMELIGHT_LIGHT_ON_ENABLE = true;
	public static final int VALUES_TO_AVERAGE = 3;
	// public static final double TARGET_POSITION_ALLOWED_ERROR = .1; // meters
	// public static final double LIMELIGHT_X_OFFSET = 0.31773; // meters

	// public static final double LIMELIGHT_DATA_WAIT_TIME = .5; // seconds
	// public static final double MAX_AMP_TARGET_DISTANCE = 3;
	// public static final Vector2 AMP_VISION_DRIVE_TARGET = new Vector2(.07, .47);

	public static final double TURRET_LOCAL_POS_X = -9 * Constants.MathConstants.INCH_TO_METER; // meters, right hand rule!! 
    public static final double TURRET_LOCAL_POS_Y = 0; // meters, right hand rule!! 
    public static final double TURRET_LOCAL_POS_Z = 20 * Constants.MathConstants.INCH_TO_METER; // meters, right hand rule!! 

    public static final Transform3d ROBOT_TO_TURRET = new Transform3d(
        new Translation3d(
                TURRET_LOCAL_POS_X,
                TURRET_LOCAL_POS_Y,
                TURRET_LOCAL_POS_Z
        ),
        new Rotation3d(0,0,0) 
    );


	public static final double HUB_FIELD_SPACE_Z_POSITION = 1.8288;
	public static final double RED_HUB_FIELD_SPACE_X_POSITION = 11.919;
	public static final double RED_HUB_FIELD_SPACE_Y_POSITION = 4.02844;
	public static final double BLUE_HUB_FIELD_SPACE_X_POSITION = 4.626; 
	public static final double BLUE_HUB_FIELD_SPACE_Y_POSITION = 4.02844; 
	public static final Translation3d RED_HUB_POSE = new Translation3d(RED_HUB_FIELD_SPACE_X_POSITION, RED_HUB_FIELD_SPACE_Y_POSITION,HUB_FIELD_SPACE_Z_POSITION);
	public static final Translation3d BLUE_HUB_POSE = new Translation3d(BLUE_HUB_FIELD_SPACE_X_POSITION, BLUE_HUB_FIELD_SPACE_Y_POSITION,HUB_FIELD_SPACE_Z_POSITION);


	public class AprilTagFieldConstants {

	public static final double FIELD_LENGTH = 16.54;
	public static final double FIELD_WIDTH = 8.07;

	public static final List<AprilTag> TAGS = List.of(
	new AprilTag(1, new Pose3d(
		new Translation3d(11.878, 7.425, 0.889),
		new Rotation3d(0, 0, Math.toRadians(180)))),

	new AprilTag(2, new Pose3d(
		new Translation3d(11.915, 4.638, 1.124),
		new Rotation3d(0, 0, Math.toRadians(90)))),

	new AprilTag(3, new Pose3d(
		new Translation3d(11.312, 4.390, 1.124),
		new Rotation3d(0, 0, Math.toRadians(180)))),

	new AprilTag(4, new Pose3d(
		new Translation3d(11.312, 4.035, 1.124),
		new Rotation3d(0, 0, Math.toRadians(180)))),

	new AprilTag(5, new Pose3d(
		new Translation3d(11.915, 3.431, 1.124),
		new Rotation3d(0, 0, Math.toRadians(270)))),

	new AprilTag(6, new Pose3d(
		new Translation3d(11.878, 0.644, 0.889),
		new Rotation3d(0, 0, Math.toRadians(180)))),

	new AprilTag(7, new Pose3d(
		new Translation3d(11.953, 0.644, 0.889),
		new Rotation3d(0, 0, Math.toRadians(0)))),

	new AprilTag(8, new Pose3d(
		new Translation3d(12.271, 3.431, 1.124),
		new Rotation3d(0, 0, Math.toRadians(270)))),

	new AprilTag(9, new Pose3d(
		new Translation3d(4.022, 3.679, 1.124),
		new Rotation3d(0, 0, Math.toRadians(0)))),

	new AprilTag(10, new Pose3d(
		new Translation3d(12.519, 4.035, 1.124),
		new Rotation3d(0, 0, Math.toRadians(0)))),

	new AprilTag(11, new Pose3d(
		new Translation3d(12.271, 4.638, 1.124),
		new Rotation3d(0, 0, Math.toRadians(90)))),

	new AprilTag(12, new Pose3d(
		new Translation3d(11.953, 7.425, 0.889),
		new Rotation3d(0, 0, Math.toRadians(0)))),

	new AprilTag(13, new Pose3d(
		new Translation3d(16.533, 7.403, 0.552),
		new Rotation3d(0, 0, Math.toRadians(180)))),

	new AprilTag(14, new Pose3d(
		new Translation3d(16.533, 6.972, 0.552),
		new Rotation3d(0, 0, Math.toRadians(180)))),

	new AprilTag(15, new Pose3d(
		new Translation3d(16.533, 4.324, 0.552),
		new Rotation3d(0, 0, Math.toRadians(180)))),

	new AprilTag(16, new Pose3d(
		new Translation3d(16.533, 3.892, 0.552),
		new Rotation3d(0, 0, Math.toRadians(180)))),

	new AprilTag(17, new Pose3d(
		new Translation3d(4.663, 0.644, 0.889),
		new Rotation3d(0, 0, Math.toRadians(0)))),

	new AprilTag(18, new Pose3d(
		new Translation3d(4.626, 3.431, 1.124),
		new Rotation3d(0, 0, Math.toRadians(270)))),

	new AprilTag(19, new Pose3d(
		new Translation3d(5.229, 3.679, 1.124),
		new Rotation3d(0, 0, Math.toRadians(0)))),

	new AprilTag(20, new Pose3d(
		new Translation3d(5.229, 4.035, 1.124),
		new Rotation3d(0, 0, Math.toRadians(90)))),

	new AprilTag(21, new Pose3d(
		new Translation3d(4.626, 4.638, 1.124),
		new Rotation3d(0, 0, Math.toRadians(90)))),

	new AprilTag(22, new Pose3d(
		new Translation3d(4.663, 7.425, 0.889),
		new Rotation3d(0, 0, Math.toRadians(0)))),

	new AprilTag(23, new Pose3d(
		new Translation3d(4.588, 7.425, 0.889),
		new Rotation3d(0, 0, Math.toRadians(180)))),

	new AprilTag(24, new Pose3d(
		new Translation3d(4.271, 4.638, 1.124),
		new Rotation3d(0, 0, Math.toRadians(90)))),

	new AprilTag(25, new Pose3d(
		new Translation3d(4.022, 4.390, 1.124),
		new Rotation3d(0, 0, Math.toRadians(180)))),

	new AprilTag(26, new Pose3d(
		new Translation3d(4.022, 4.035, 1.124),
		new Rotation3d(0, 0, Math.toRadians(180)))),

	new AprilTag(27, new Pose3d(
		new Translation3d(4.271, 3.431, 1.124),
		new Rotation3d(0, 0, Math.toRadians(270)))),

	new AprilTag(28, new Pose3d(
		new Translation3d(4.588, 0.644, 0.889),
		new Rotation3d(0, 0, Math.toRadians(180)))),

	new AprilTag(29, new Pose3d(
		new Translation3d(0.008, 0.666, 0.552),
		new Rotation3d(0, 0, Math.toRadians(0)))),

	new AprilTag(30, new Pose3d(
		new Translation3d(0.008, 1.098, 0.552),
		new Rotation3d(0, 0, Math.toRadians(0)))),

	new AprilTag(31, new Pose3d(
		new Translation3d(0.008, 3.746, 0.552),
		new Rotation3d(0, 0, Math.toRadians(0)))),

	new AprilTag(32, new Pose3d(
		new Translation3d(0.008, 4.178, 0.552),
		new Rotation3d(0, 0, Math.toRadians(0))))
	);
     }
}
