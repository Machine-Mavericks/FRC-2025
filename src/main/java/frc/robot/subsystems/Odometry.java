package frc.robot.subsystems;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;

/** Subsystem */
public class Odometry extends SubsystemBase {
  // constant to convert degrees to radians
  public final static float DEGtoRAD = (float) (3.1415926 / 180);
  static double previousLeft, previousRight, previousFront;
  // Local objects and variables here
  // These are for things that only belong to, and used by, the subsystem
  /** Place code here to initialize subsystem */

  private SwerveDrivePoseEstimator m_Estimator;

  double fieldX = 0.0;
  double fieldY = 0.0;
  double fieldAngle = 0.0;
  double leftPos;
  double rightPos;
  double frontPos;
  double leftChangePos;
  double rightChangePos;
  double frontChangePos;

  public Odometry() {

    // create position estimator - set to (0,0,0)(x,y,ang)
    // initialize swerve drive odometry
    m_Estimator = new SwerveDrivePoseEstimator(
        RobotContainer.drivesystem.getKinematics(),
        new Rotation2d(0, 0),
        RobotContainer.drivesystem.GetSwerveDistances(),
        new Pose2d(0.0, 0.0, new Rotation2d(0.0)),
        VecBuilder.fill(0.02, 0.02, 0.02),
        VecBuilder.fill(0.02, 0.02, 0.03));

    // reset initial position
    setPosition(0.0, 0.0, 0.0, 0.0);

    // create odometry shuffleboard page
    initializeShuffleboard();

  }

  /**
   * Method called periodically by the scheduler
   * Place any code here you wish to have run periodically
   */
  @Override
  public void periodic() {
    updateOdometry();
    updateShuffleboard();
    leftPos = DeadWheel.getLeftEncoderDistance();
    rightPos = DeadWheel.getRightEncoderDistance();
    frontPos = DeadWheel.getFrontEncoderDistance();

    leftChangePos = leftPos - previousLeft;
    rightChangePos = rightPos - previousRight;
    frontChangePos = frontPos - previousFront;

    previousLeft = leftPos;
    previousRight = rightPos;
    previousFront = frontPos;

    // creating the value of sin theta (aka the angle of the hipotinuse)
    double theta = Math.asin((rightChangePos - leftChangePos) / DeadWheel.LATERAL_DISTANCE);

    // equation that tells us how much the robot has moved forward
    double ForwardChange = (leftChangePos + rightChangePos) / 2.0;

    // equation that tells us how much the robot has moved laterally
    double LateralChange = (frontChangePos - DeadWheel.FORWARD_OFFSET * Math.sin(theta));// Lateral means left to right

    double IMUHeading = Math.toRadians(RobotContainer.gyro.getYawAngle());

    double fieldForwardChange = ForwardChange * Math.cos(IMUHeading) - LateralChange * Math.sin(IMUHeading);

    double fieldLateralChange = ForwardChange * Math.sin(IMUHeading) + LateralChange * Math.cos(IMUHeading);

    fieldX += fieldForwardChange;// += means is equal to and add fieldForwardChange to itself

    fieldY += fieldLateralChange;// += means is equal to and add fieldLateralChange to itself

    fieldAngle = IMUHeading;
  }

  // -------- update odometry methods

  // initialize robot odometry to zero
  public void InitializeToZero() {
    setPosition(0.0, 0.0, 0.0, 0.0);
  }

  // called by the drive train synchronously with swerve module data updates to
  // reduce latency
  public void updateOdometry() {
    // get gyro angle (in degrees) and make rotation vector
    Rotation2d gyroangle = new Rotation2d(RobotContainer.gyro.getYawAngle() * DEGtoRAD);

    // get position of all sewrve modules from subsystem
    SwerveModulePosition[] positions = RobotContainer.drivesystem.GetSwerveDistances();

    // ensure we have the proper length array positions
    if (positions.length >= 4) {
      // update robots odometry
      m_Estimator.update(gyroangle, positions);
    }
    updateShuffleboard();
  }

  public void setPosition(Pose2d position, double gyroangle) {
    setPosition(position.getX(), position.getY(), position.getRotation().getDegrees(), gyroangle);
  }

  public void setPosition(double x, double y, double robotangle, double gyroangle) {
    // make robot position vector
    Pose2d position = new Pose2d(x, y, new Rotation2d(robotangle * DEGtoRAD));

    // set robot odometry
    m_Estimator.resetPosition(new Rotation2d(gyroangle * DEGtoRAD),
        RobotContainer.drivesystem.GetSwerveDistances(),
        position);

  }

  /**
   * adds vision measurements
   * 
   * @param vision
   * @param timeStamp
   * @param distance
   */
  /*
   * public void addVision(Pose2d vision, double distance) {
   * if (m_useLimelightAngle) {
   * double stDevs = 0.03 * distance;
   * 
   * double stdDevs = 0.06*distance;
   * m_Estimator.setVisionMeasurementStdDevs(VecBuilder.fill(stdDevs, stdDevs, 2.0
   * * stdDevs));
   * m_Estimator.addVisionMeasurement(vision, Timer.getFPGATimestamp());
   * 
   * // show april tag as dot on field 2d widget
   * // RobotContainer.operatorinterface.m_field.getObject("tag").setPose(vision);
   * } else {
   * double CurrentGyro = RobotContainer.gyro.getYawAngle()*DEGtoRAD;
   * double VisionAngle = vision.getRotation().getRadians();
   * 
   * Pose2d NewEstimate = new Pose2d(vision.getX(),vision.getY(),new
   * Rotation2d(0.995*CurrentGyro + 0.005*VisionAngle));
   * 
   * double stdDevs = 0.06*distance;
   * m_Estimator.setVisionMeasurementStdDevs(VecBuilder.fill(stdDevs, stdDevs,
   * stdDevs));
   * m_Estimator.addVisionMeasurement(NewEstimate, Timer.getFPGATimestamp());
   * 
   * // show apriltag estimate as 'dot' on field2d widget
   * // RobotContainer.operatorinterface.m_field.getObject("tag").setPose(vision);
   * }
   * }
   */

  // -------------------- Robot Current Odometry Access Methods
  // --------------------

  /** return robot's current position vector Pose2d */
  public Pose2d getPose2d() {
    return m_Estimator.getEstimatedPosition();
  }

  Pose2d m_MemPoints[] = { new Pose2d(0, 0, new Rotation2d(0.0)),
      new Pose2d(0, 0, new Rotation2d(0.0)),
      new Pose2d(0, 0, new Rotation2d(0.0)) };

  /**
   * saves Pose2D coordinate for later recall
   * num = 0 to 2 (three memories available)
   */
  public void RecordPose2d(Pose2d point, int num) {
    if (num < m_MemPoints.length)
      m_MemPoints[num] = point;
  }

  /**
   * recalls Pose2D coordinate previously saved
   * num = 0 to 2 (three memories available)
   */
  public Pose2d RecallPoint(int num) {
    // return saved point. If not in range, simply return 0,0,0 point
    if (num < m_MemPoints.length)
      return m_MemPoints[num];
    else
      return new Pose2d(0, 0, new Rotation2d(0.0));
  }

  // -------------------- Subsystem Shuffleboard Methods --------------------
private GenericEntry m_fieldXPos;
private GenericEntry m_fieldYPos;
private GenericEntry m_fieldAngle;
  /** Initialize subsystem shuffleboard page and controls */
  private void initializeShuffleboard() {
     // Create page in shuffleboard
        ShuffleboardTab Tab = Shuffleboard.getTab("Odometry");
        ShuffleboardLayout l1 = Tab.getLayout("Odometry", BuiltInLayouts.kList);
        l1.withPosition(0, 0);
        l1.withSize(2, 4);
        m_fieldXPos = l1.add("Field X ",0.0 ).getEntry();
        m_fieldYPos = l1.add("Field Y ",0.0 ).getEntry();
        m_fieldAngle = l1.add("Angle ",0.0 ).getEntry();
  }

  /** Update subsystem shuffle board page with current odometry values */
  private void updateShuffleboard() {
    m_fieldXPos.setDouble(fieldX);
    m_fieldYPos.setDouble(fieldY);
    m_fieldAngle.setDouble(fieldAngle);
  }

}
