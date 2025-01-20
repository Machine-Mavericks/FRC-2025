package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.RobotMap;
import frc.robot.utils.Utils;

/** Subsystem */
public class SwerveDrive extends SubsystemBase {


    // Setup Drive Kinematics Object Constants
    private final double TRACK_WIDTH = 0.31;   // Width between the left and right wheels - in m.
    private final double TRACK_LENGTH = 0.29;  // Length between the front and back wheel - in m.
    
    // steering gear ratio for SPS MK4 L1 Swerve
    static final double STEER_RATIO = 12.8;

    // drive gear ratio for SPS MK4 L1 Swerve
    static final double DRIVE_RATIO = 8.14;

    // drive wheel diameter in m
    final double WHEEL_DIA = 0.1;              

    // conversion factors - used to convert motor MPS to RPS and vice versa
    final double RPS_TO_MPS = Math.PI * WHEEL_DIA / DRIVE_RATIO;
    final double MPS_TO_RPS = 1.0 / RPS_TO_MPS;

    // constants for FalconFx motors
    final double MAXRPM = 6380.0;
    final double MAXRPS = MAXRPM / 60.0;

    // maximum expected drive speed
    public final double MAX_SPEED = MAXRPS * RPS_TO_MPS;

     // The drivetrain's kinematics model
    private SwerveDriveKinematics driveKinematics;
    private final Translation2d m_CenterOfRotation = new Translation2d(0.0,0.0);;
    private final Rotation2d m_ParkAngleLF = new Rotation2d(45.0);
    private final Rotation2d m_ParkAngleRF = new Rotation2d(-45.0);


    // create CANCoder sensor objects
    private CANcoder m_LFCanCoder;
    private CANcoder m_RFCanCoder;
    private CANcoder m_LRCanCoder;
    private CANcoder m_RRCanCoder;

    // create steer motor objects
    // create position controllers for steer motors
    private TalonFX m_LFSteerMotor;
    private TalonFX m_RFSteerMotor;
    private TalonFX m_LRSteerMotor;
    private TalonFX m_RRSteerMotor;
    private PositionDutyCycle m_LFSteerControl;
    private PositionDutyCycle m_RFSteerControl;
    private PositionDutyCycle m_LRSteerControl;
    private PositionDutyCycle m_RRSteerControl;

    // create drive motor objects
    // create speed controllers for drive motors
    private TalonFX m_LFDriveMotor;
    private TalonFX m_RFDriveMotor;
    private TalonFX m_LRDriveMotor;
    private TalonFX m_RRDriveMotor;
    private VelocityVoltage m_LFDriveControl;
    private VelocityVoltage m_RFDriveControl;
    private VelocityVoltage m_LRDriveControl;
    private VelocityVoltage m_RRDriveControl;


    
    // Swerve module states - contains speed(m/s) and angle for each swerve module
    private SwerveModuleState[] m_states;

    /** Place code here to initialize subsystem */
    public SwerveDrive() {


        // setup the drive Kinematics
        driveKinematics = new SwerveDriveKinematics(
                // Front Left
                new Translation2d(TRACK_LENGTH * 0.5, TRACK_WIDTH * 0.5),
                // Front Right
                new Translation2d(TRACK_LENGTH * 0.5, TRACK_WIDTH * -0.5),
                // Back Left
                new Translation2d(TRACK_LENGTH * -0.5, TRACK_WIDTH * 0.5),
                // Back Right
                new Translation2d(TRACK_LENGTH * -0.5, TRACK_WIDTH * -0.5));


        // ---------- CANCoders Setup ----------

        // create CANCoder objects - cancoders used for absolute steering feedback
        m_LFCanCoder = new CANcoder(RobotMap.CANID.LF_CANCODER);
        m_RFCanCoder = new CANcoder(RobotMap.CANID.RF_CANCODER);
        m_LRCanCoder = new CANcoder(RobotMap.CANID.LR_CANCODER);
        m_RRCanCoder = new CANcoder(RobotMap.CANID.RR_CANCODER);

        // configure cancoders
        // configure for counter-clockwise positive to match FRC coorinate system
        // set to provide values between -0.5 and 0.5
        // Note: must configure separately as each cancoder will have a unique offset to be set
        CANcoderConfiguration LFEncoderConfig = new CANcoderConfiguration();
        LFEncoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
        LFEncoderConfig.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 0.5;
        CANcoderConfiguration RFEncoderConfig = new CANcoderConfiguration();
        RFEncoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
        RFEncoderConfig.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 0.5;
        CANcoderConfiguration LREncoderConfig = new CANcoderConfiguration();
        LREncoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
        LREncoderConfig.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 0.5;
        CANcoderConfiguration RREncoderConfig = new CANcoderConfiguration();
        RREncoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
        RREncoderConfig.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 0.5;
        
        // encoder offsets - ADJUST values to align wheels
        // units - in rotations
        // note:  if +ve value when wheels are aligned straight, then decrease offset by that value
        // encoders should then read zero when aligned straight.
        LFEncoderConfig.MagnetSensor.MagnetOffset = 0.0;
        RFEncoderConfig.MagnetSensor.MagnetOffset = 0.0;
        LREncoderConfig.MagnetSensor.MagnetOffset = 0.0;
        RREncoderConfig.MagnetSensor.MagnetOffset = 0.0;

        // apply configuration to cancoders
        m_LFCanCoder.getConfigurator().apply(LFEncoderConfig);
        m_RFCanCoder.getConfigurator().apply(RFEncoderConfig);
        m_LRCanCoder.getConfigurator().apply(LREncoderConfig);
        m_RRCanCoder.getConfigurator().apply(RREncoderConfig);
        

        // ---------- Steer Motor Setup ----------

        // create steer motors
        m_LFSteerMotor = new TalonFX(RobotMap.CANID.LF_STEER_MOTOR);
        m_RFSteerMotor = new TalonFX(RobotMap.CANID.RF_STEER_MOTOR);
        m_LRSteerMotor = new TalonFX(RobotMap.CANID.LR_STEER_MOTOR);
        m_RRSteerMotor = new TalonFX(RobotMap.CANID.RR_STEER_MOTOR);

        // turn on safety oversight of steer motors
        m_LFSteerMotor.setSafetyEnabled(true);
        m_RFSteerMotor.setSafetyEnabled(true);
        m_LRSteerMotor.setSafetyEnabled(true);
        m_RRSteerMotor.setSafetyEnabled(true);

        // configure steer motors
        // set for counter-clockwise +ve rotation - to match FRC coordinate system
        // set neutral mode to coast.  Can change to brake later for competition use
        // set deadband so steer motor does not chatter when at destination position
        // used CTRE webpage to estimate gains from Phoenix 5 gains used in 2023
        // set sensor to mechanism gear ratio
        TalonFXConfiguration steerConfig = new TalonFXConfiguration();
        steerConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        steerConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        steerConfig.MotorOutput.DutyCycleNeutralDeadband = 0.0001;
        steerConfig.Slot0.kP = 0.0;
        steerConfig.Slot0.kI = 0.0;
        steerConfig.Slot0.kD = 0.0;
        steerConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
        steerConfig.Feedback.SensorToMechanismRatio = STEER_RATIO;
        //steerConfig.CurrentLimits.SupplyCurrentLimitEnable =  // note default is true
        //steerConfig.CurrentLimits.SupplyCurrentLimit =        // note default is 80A
        //steerConfig.CurrentLimits.SupplyCurrentLowerTime =    // note default is 1.0s
        //steerConfig.CurrentLimits.SupplyCurrentLowerLimit =   // note default is 40A

        // apply configuration to motors
        m_LFSteerMotor.getConfigurator().apply(steerConfig);
        m_RFSteerMotor.getConfigurator().apply(steerConfig);
        m_LRSteerMotor.getConfigurator().apply(steerConfig);
        m_RRSteerMotor.getConfigurator().apply(steerConfig);

        // set up steer motor controllers
        // use SLot0 for PID values
        // set initial targert position to 0.0
        m_LFSteerControl.Slot = 0;
        m_RFSteerControl.Slot = 0;
        m_LRSteerControl.Slot = 0;
        m_RRSteerControl.Slot = 0;
        m_LFSteerControl.Position = 0.0;
        m_RFSteerControl.Position = 0.0;
        m_LRSteerControl.Position = 0.0;
        m_RRSteerControl.Position = 0.0;

        
        // ---------- Drive Motor Setup ----------


        // create drive motors
        m_LFDriveMotor = new TalonFX(RobotMap.CANID.LF_DRIVE_MOTOR);
        m_RFDriveMotor = new TalonFX(RobotMap.CANID.RF_DRIVE_MOTOR);
        m_LRDriveMotor = new TalonFX(RobotMap.CANID.LR_DRIVE_MOTOR);
        m_RRDriveMotor = new TalonFX(RobotMap.CANID.RR_DRIVE_MOTOR);

        // turn on safety of all drive motors
        m_LFDriveMotor.setSafetyEnabled(true);
        m_RFDriveMotor.setSafetyEnabled(true);
        m_LRDriveMotor.setSafetyEnabled(true);
        m_RRDriveMotor.setSafetyEnabled(true);

        // configure drive motors
        // set for clockwise +ve rotation
        // set neutral mode to coast.  Can change to brake later for competition use
        // set deadband so drive motor does not chatter when at zero speed
        // used CTRE webpage to estimate gains from Phoenix 5 gains used in 2023
        // set sensor to mechanism gear ratio
        TalonFXConfiguration driveConfig = new TalonFXConfiguration();
        driveConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        driveConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        driveConfig.MotorOutput.DutyCycleNeutralDeadband = 0.0001;
        driveConfig.Slot0.kP = 0.0;
        driveConfig.Slot0.kI = 0.0;
        driveConfig.Slot0.kD = 0.0;
        driveConfig.Slot0.kS = 0.0;
        driveConfig.Slot0.kV = 0.0;
        driveConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
        driveConfig.Feedback.SensorToMechanismRatio = DRIVE_RATIO;
        //steerConfig.CurrentLimits.SupplyCurrentLimitEnable =  // note default is true
        //steerConfig.CurrentLimits.SupplyCurrentLimit =        // note default is 80A
        //steerConfig.CurrentLimits.SupplyCurrentLowerTime =    // note default is 1.0s
        //steerConfig.CurrentLimits.SupplyCurrentLowerLimit =   // note default is 40A

        // apply configuration to motors
        m_LFDriveMotor.getConfigurator().apply(driveConfig);
        m_RFDriveMotor.getConfigurator().apply(driveConfig);
        m_LRDriveMotor.getConfigurator().apply(driveConfig);
        m_RRDriveMotor.getConfigurator().apply(driveConfig);

        // set up drive motor controllers
        // use SLot0 for PID values
        // set initial targert speed to 0.0
        m_LFDriveControl.Slot = 0;
        m_RFDriveControl.Slot = 0;
        m_LRDriveControl.Slot = 0;
        m_RRDriveControl.Slot = 0;
        m_LFDriveControl.Velocity = 0.0;
        m_RFDriveControl.Velocity = 0.0;
        m_LRDriveControl.Velocity = 0.0;
        m_RRDriveControl.Velocity = 0.0;


        // ---------- Other ----------


        // initialize encoders of each steer motor according to CANCoder positions
        ResetSteerEncoders();

        // create subsystem shuffle board page
        //initializeShuffleboard();

        // initially motors are off
        RobotDrive(0.0, 0.0, 0.0, false);

    }

    // seed the encoder value of steering motors based on reported CANcoder positions
    public void ResetSteerEncoders() {
    
        // wheel alignment calibration robot base
        m_LFSteerMotor.setPosition(m_LFCanCoder.getAbsolutePosition().getValue());
        m_RFSteerMotor.setPosition(m_RFCanCoder.getAbsolutePosition().getValue());
        m_LRSteerMotor.setPosition(m_LRCanCoder.getAbsolutePosition().getValue());
        m_RRSteerMotor.setPosition(m_RRCanCoder.getAbsolutePosition().getValue());
    }



    /** Method called periodically by the scheduler */
    @Override
    public void periodic() {

    }

    /** Drive robot in field-oriented coordinates
      ChassisSpeeds x,y in m/s, omega in rad/s
      Park - places robot in park mode - other inputs are ignored */
    public void FieldDrive(double dx, double dy, double omega, boolean Park)
        { FieldDrive (new ChassisSpeeds(dx, dy, omega), Park); }

    public void FieldDrive(ChassisSpeeds speed, boolean Park) {
        // convert chassis speeds from field to robot oriented, getting angle from gyro
        ChassisSpeeds newSpeed = ChassisSpeeds.fromFieldRelativeSpeeds(speed, Rotation2d.fromDegrees(RobotContainer.gyro.getYaw()));
    
        // speeds now in robot coordinates - call robot drive
        RobotDrive (newSpeed, Park);
    }

    /** Drive robot in robot-oriented coordinates
      ChassisSpeeds x,y in m/s, omega in rad/s
      Park - places robot in park mode - other inputs are ignored */
    public void RobotDrive(double dx, double dy, double omega, boolean Park)
        { RobotDrive (new ChassisSpeeds(dx, dy, omega), Park); }
    
    public void RobotDrive(ChassisSpeeds speed, boolean Park) {
    
        // try this to see if the swerve modules will leave the wheel angles alone when the speed is zero instead of setting the angle to zero
        m_states = driveKinematics.toSwerveModuleStates(speed, m_CenterOfRotation); 

        // Park will override any of the drive inputs for chassis speeds
        if (Park) {
            // Note that the LF and RR wheels are both set to the same angle. Same for the RF and LR wheels
            m_states[0].angle = m_ParkAngleLF;       m_states[0].speedMetersPerSecond = 0;
            m_states[1].angle = m_ParkAngleRF;       m_states[1].speedMetersPerSecond = 0;
            m_states[2].angle = m_ParkAngleRF;       m_states[2].speedMetersPerSecond = 0;
            m_states[3].angle = m_ParkAngleLF;       m_states[3].speedMetersPerSecond = 0;
        }

        // if desired speed of a swerve(s) exceed maximum possible, then reduce all speeds while maintaining ratio
        SwerveDriveKinematics.desaturateWheelSpeeds(m_states, MAX_SPEED);

        // assume drive motors driving in forward direction until determined otherwise
        double LFDriveDir = 1.0;
        double RFDriveDir = 1.0;
        double LRDriveDir = 1.0;
        double RRDriveDir = 1.0;
        
        
        // ---------- Angle Determination for LF Swerve
        
        // adder used to determine angle depending on direction of swerve drive
        double adder1=0.0;

        // get LF motor's current drive direction. If currently in reverse:
        // a) consider its angle to be 180deg more than its sensor currently shows; and
        // b) set direction flag to reverse
        if (m_LFDriveMotor.getClosedLoopReference().getValueAsDouble() < 0.0)
        { adder1 = 180.0; LFDriveDir = -1.0; }

        // get current angle of swerve (in deg)
        double LFCurrentAngleDeg = m_LFSteerMotor.getPosition().getValueAsDouble()*360.0;

        // determine smallest angle to turn swerve to get to desired angle
        double LFAngleDiff = Utils.AngleDifference(LFCurrentAngleDeg%360.0, adder1+m_states[0].angle.getDegrees());

        // to minimize turning, it may be easier to reverse drive, and turn by smaller angle
        if (LFAngleDiff<-90.0)
        { LFDriveDir *= -1.0; LFAngleDiff+=180.0; }
        else if (LFAngleDiff>90)
        { LFDriveDir *= -1.0; LFAngleDiff-=180.0; }

        // set angle of swerve drive
        //m_LFSteerMotor.set(ControlMode.Position, (LFCurrentAngleDeg + LFAngleDiff)*DEG_TO_ENCODERPULSE);
        
        // ---------- Set Drive Motor Speeds

        // go ahead and set motor closed loop target speeds
        //PositionVoltage m_LFSteerControl = new PositionVoltage(0.0);
        //m_LFSteerMotor.setControl(m_LFSteerControl.withPosition(0.0);

        //m_LFDriveMotor.setControl
        //m_LFDriveMotor.set(ControlMode.Velocity, m_states[0].speedMetersPerSecond*LFDriveDir*METERS_TO_ENCODERPULSE*0.1);


    }



    /** Returns kinematics of drive system */
    public SwerveDriveKinematics getKinematics() {
        return driveKinematics;
    }


}
