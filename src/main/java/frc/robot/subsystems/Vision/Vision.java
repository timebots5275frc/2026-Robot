package frc.robot.subsystems.Vision;

import java.util.ArrayList;
import java.util.function.BooleanSupplier;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.VisionConstants;
import frc.robot.CustomTypes.Math.Vector3;

public class Vision extends SubsystemBase {

  private int aprilTagID = -1;
  private double horizontalOffsetFromAprilTag;

  // TAG (robot space)
  private Vector3 avgAprilTagPosInRobotSpace = Vector3.zero;
  private ArrayList<Vector3> aprilTagPosInRobotSpaceValues = new ArrayList<>();

  private Vector3 avgAprilTagRotInRobotSpace = Vector3.zero;
  private ArrayList<Vector3> aprilTagRotInRobotSpaceValues = new ArrayList<>();

  // ROBOT (field space)
  private Vector3 avgRobotPosInFieldSpace = Vector3.zero;
  private ArrayList<Vector3> robotPosInFieldSpaceValues = new ArrayList<>();
  
  private Vector3 avgRobotRotInFieldSpace = Vector3.zero;
  private ArrayList<Vector3> robotRotInFieldSpaceValues = new ArrayList<>();

  // YAW (radians)
  private double robotYawRadians = 0.0;

  public BooleanSupplier HasValidData = () -> hasValidData();

  public static boolean usingLimelight = false;
  int thingsUsingLimelight = 0;

  @Override
  public void periodic() {
    var table = NetworkTableInstance.getDefault().getTable("limelight");

    aprilTagID = (int) table.getEntry("tid").getDouble(-1.0);
    horizontalOffsetFromAprilTag = table.getEntry("tx").getDouble(0);
    double tv = table.getEntry("tv").getDouble(0);

    if (tv == 1) {
      CalculateRobotPositionInFieldSpaceone();
      CalculateTargetTransformInRobotSpaceone();
    } else {
      // ClearAprilTagData();
    }

    LogData();
  }

  void CalculateTargetTransformInRobotSpaceone()
  {
    double[] vals = NetworkTableInstance.getDefault().getTable("limelight").getEntry("targetpose_robotspace").getDoubleArray(new double[6]);
    if(vals[0]!=0){
      // addVector3ToArrayList(new Vector3(vals[0], vals[1], vals[2]), aprilTagPosInRobotSpaceValues);
      addVector3ToArrayList(new Vector3(vals[3], vals[4], vals[5]), aprilTagRotInRobotSpaceValues);
      // avgAprilTagPosInRobotSpace = getAverageOfArrayList(aprilTagPosInRobotSpaceValues);
      avgAprilTagRotInRobotSpace = getAverageOfArrayList(aprilTagRotInRobotSpaceValues);
    } else {
      System.out.println("Bad Data");
    }
  }

  public void CalculateRobotPositionInFieldSpaceone()
  {
    double[] vals = NetworkTableInstance.getDefault().getTable("limelight").getEntry("botpose").getDoubleArray(new double[6]);
    //botpose - x,y,z, roll, pitch, yaw
    if (vals[0]!=0) {
      //  addVector3ToArrayList(new Vector3(vals[0], vals[1], vals[2]), robotPosInFieldSpaceValues);
      //addVector3ToArrayList(new Vector3(vals[2], vals[0] * -1, vals[1] * -1), robotPosInFieldSpaceValues);

      addVector3ToArrayList(new Vector3(vals[3], vals[4], vals[5]), robotRotInFieldSpaceValues);
      // avgRobotPosInFieldSpace = getAverageOfArrayList(robotPosInFieldSpaceValues);
      avgRobotRotInFieldSpace = getAverageOfArrayList(robotRotInFieldSpaceValues);
    } else {
      System.out.println("Bad Data");
    }
    
  }

  void addVector3ToArrayList(Vector3 newVal, ArrayList<Vector3> arrayList)
  {
    arrayList.add(0, newVal);
    if (arrayList.size() > VisionConstants.VALUES_TO_AVERAGE) {arrayList.remove(arrayList.size() - 1); }
  }


  // =========================
  // TAG POSITION (robot space)
  // =========================
  void CalculateTargetTransformInRobotSpace() {
    var table = NetworkTableInstance.getDefault().getTable("limelight");

    double[] vals = table.getEntry("targetpose_robotspace").getDoubleArray(new double[6]);

    if (vals[0]!=0) {

      Vector3 pos = new Vector3(vals[0], vals[1], vals[2]);

      addFilteredSample(pos, 100);
      avgAprilTagPosInRobotSpace = getAverageOfArrayList(aprilTagPosInRobotSpaceValues);
    }
  }

  // =========================
  // ROBOT POSITION (field space)
  // =========================
  public void CalculateRobotPositionInFieldSpace() {

    var table = NetworkTableInstance.getDefault().getTable("limelight");

    double[] vals = table.getEntry("botpose").getDoubleArray(new double[6]);

    if (vals[0]!=0) {
      AprilTag tag = VisionConstants.AprilTagFieldConstants.TAGS.get(aprilTagID);

      Vector3 pos = new Vector3(vals[0] + tag.pose.getX(), vals[1] + tag.pose.getY(), vals[2]);

      addFilteredRobotSample(pos, 100);
      avgRobotPosInFieldSpace = getAverageOfArrayList(robotPosInFieldSpaceValues);

      robotYawRadians = Math.toRadians(vals[5]);
    }
  }

  // =========================
  // FILTERS
  // =========================
  void addFilteredSample(Vector3 newPos, double maxDelta) {
    if (!aprilTagPosInRobotSpaceValues.isEmpty()) {
      Vector3 avg = getAverageOfArrayList(aprilTagPosInRobotSpaceValues);
      if (Vector3.distance(newPos, avg) > maxDelta) return;
    }

    aprilTagPosInRobotSpaceValues.add(0, newPos);

    if (aprilTagPosInRobotSpaceValues.size() > 5) {
      aprilTagPosInRobotSpaceValues.remove(aprilTagPosInRobotSpaceValues.size() - 1);
    }
  }

  void addFilteredRobotSample(Vector3 newPos, double maxDelta) {
    if (!robotPosInFieldSpaceValues.isEmpty()) {
      Vector3 avg = getAverageOfArrayList(robotPosInFieldSpaceValues);
      if (Vector3.distance(newPos, avg) > maxDelta) return;
    }

    robotPosInFieldSpaceValues.add(0, newPos);

    if (robotPosInFieldSpaceValues.size() > 10) {
      robotPosInFieldSpaceValues.remove(robotPosInFieldSpaceValues.size() - 1);
    }
  }

  // =========================
  // UTIL
  // =========================
  Vector3 getAverageOfArrayList(ArrayList<Vector3> arrayList) {
    if (arrayList.isEmpty()) return Vector3.zero;

    Vector3 sum = Vector3.zero;
    for (Vector3 v : arrayList) {
      sum = sum.add(v);
    }

    return sum.divideBy(arrayList.size());
  }

  public boolean hasValidData() {
    return aprilTagID != -1;
  }

  void LogData() {
    SmartDashboard.putString("Detected AprilTagID", aprilTagID == -1 ? "None" : String.valueOf(aprilTagID));
    SmartDashboard.putString("AprilTag position (robot)", aprilTagID == -1 ? "N/A" : avgAprilTagPosInRobotSpace.toString(3));
    SmartDashboard.putNumber("Robot Yaw (rad)", robotYawRadians);
    SmartDashboard.putNumber("Things using limelight", thingsUsingLimelight);
  }

  public void ClearAprilTagData() {
    aprilTagPosInRobotSpaceValues.clear();
    robotPosInFieldSpaceValues.clear(); // ✅ important
  }

  public void OnTeleopDisable() {
    thingsUsingLimelight = 0;
    ToggleLimelightLight(false);
  }

  public void setUsingLimelight(boolean using) {
    thingsUsingLimelight += using ? 1 : -1;

    if (thingsUsingLimelight <= 0) {
      thingsUsingLimelight = 0;
      ToggleLimelightLight(false);
    } else {
      ToggleLimelightLight(true);
    }
  }

  public void ToggleLimelightLight(boolean on) {
    usingLimelight = on;
    NetworkTableInstance.getDefault().getTable("limelight")
        .getEntry("ledMode").setNumber(on ? 3 : 1);
  }

  // =========================
  // GETTERS
  // =========================
  public int AprilTagID() { return aprilTagID; }
  public Vector3 AprilTagPosInRobotSpace() { return avgAprilTagPosInRobotSpace; }
  public Vector3 RobotPosInFieldSpace() { return avgRobotPosInFieldSpace; }
  public double RobotYawRadians() { return robotYawRadians; }
  public double HorizontalOffsetFromAprilTag() { return horizontalOffsetFromAprilTag; }
  public Vector3 AprilTagRotInRobotSpace() { return avgAprilTagRotInRobotSpace; } //
  public Vector3 RobotRotInFieldSpace() { return avgRobotRotInFieldSpace; } //
}