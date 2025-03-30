package frc.robot.subsystems;

import java.lang.annotation.Target;

import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.LimitSwitchConfig;
import com.revrobotics.spark.config.LimitSwitchConfig.Type;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.RobotMap;

/** Subsystem */
public class AlgaeRemover extends SubsystemBase {
    static SparkMax algaeMotor = new SparkMax(RobotMap.CANID.ALGAE_MOTOR, MotorType.kBrushless);
    static SparkMax algaeTiltMotor = new SparkMax(RobotMap.CANID.ALGAE_TILT_MOTOR, MotorType.kBrushless);
    SparkMaxConfig algaeTiltConfig;
    static double gearDiameterCM = 0.5 * 2.54;
    static double gearCircumference = gearDiameterCM * Math.PI;
    static double gearRatio = 9.0, paddingOffset = 0;
    //public static double L2 = 19.0, L3 = 36.5, L4 = 70.0, L1 = 10.7, L0 = 0.0; // l4 in cm high 71.5, l3 47.5 from floor
    public static double TILT_DOWN = 2.0, TILT_UP = 0.0; // TILT_DOWN to be set
    static double ticksMoved;
    static double feedForward = -0.2;
    double TargetPositionCM = 0.0;

    // Local objects and variables here
    // These are for things that only belong to, and used by, the subsystem

    /** Place code here to initialize subsystem */
    public AlgaeRemover() {
        // initialize limit switch and motors
        algaeTiltConfig = new SparkMaxConfig();
        // Current limit while testing this...possibly remove after.
        algaeTiltConfig.smartCurrentLimit(30);
        algaeTiltConfig.limitSwitch.reverseLimitSwitchEnabled(true);
        algaeTiltConfig.limitSwitch.reverseLimitSwitchType(Type.kNormallyOpen);
        //algaeTiltConfig.limitSwitch.forwardLimitSwitchEnabled(true);
        //algaeTiltConfig.limitSwitch.forwardLimitSwitchType(Type.kNormallyOpen);
        algaeTiltConfig.idleMode(IdleMode.kBrake);
        algaeTiltConfig.inverted(false); // Change this if motor is running backwards
        algaeTiltConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder);
        algaeTiltConfig.closedLoop.p(0.90);
        algaeTiltConfig.closedLoop.i(0.0);
        algaeTiltConfig.closedLoop.d(0.0);
        algaeTiltConfig.closedLoop.outputRange(-0.5, 0.13);
        algaeTiltConfig.closedLoop.positionWrappingEnabled(false);
        algaeTiltConfig.encoder.positionConversionFactor(1);
        
        algaeTiltMotor.configure(algaeTiltConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        ticksMoved = 0;
        initializeShuffleboard();
    }

    private static GenericEntry m_algaeMotorEntry;
    private static GenericEntry m_algaeTiltMotorEntry;
    private static GenericEntry m_switchR;
    private static GenericEntry m_current;
    private static GenericEntry m_output;

    private void initializeShuffleboard() {
        // Create page in shuffleboard
        ShuffleboardTab Tab = Shuffleboard.getTab("Algae Remover");
        ShuffleboardLayout l1 = Tab.getLayout("Algae", BuiltInLayouts.kList);
        l1.withPosition(0, 0);
        l1.withSize(2, 4);
        m_algaeMotorEntry = l1.add("motor", 0.0).getEntry();
        m_algaeTiltMotorEntry = l1.add("tilt", 0.0).getEntry();
        m_switchR = l1.add("switchB", false).getEntry();
        m_current = l1.add("current", 0.0).getEntry();
        m_output = l1.add("output", 0.0).getEntry();

    }

    private void updateShuffleboard() {
        m_algaeTiltMotorEntry.setDouble((algaeTiltMotor.getEncoder().getPosition()));
        m_switchR.setBoolean(algaeTiltMotor.getReverseLimitSwitch().isPressed());
        m_current.setDouble(algaeTiltMotor.getOutputCurrent());
        m_output.setDouble(algaeTiltMotor.getAppliedOutput());

    }
    public void ZeroEncoder(){
        algaeTiltMotor.getEncoder().setPosition(0);
        algaeTiltMotor.getClosedLoopController().setReference(0,ControlType.kPosition);
    }

    /**
     * Method called periodically by the scheduler
     * Place any code here you wish to have run periodically
     */
    @Override
    public void periodic() {
        updateShuffleboard();
       if(algaeTiltMotor.getReverseLimitSwitch().isPressed()){
           algaeTiltMotor.getEncoder().setPosition(0); // this might need to be ZeroEncoder function
           //ZeroEncoder();

       }
    }

    public void RemoveAlgae() {
        TargetPositionCM = TILT_DOWN;
        algaeTiltMotor.getClosedLoopController().setReference((TILT_DOWN), ControlType.kPosition,
            ClosedLoopSlot.kSlot0,feedForward);
        algaeMotor.set(0.5);
    }

    public void ResetTilt() {
        TargetPositionCM = TILT_UP;
        algaeTiltMotor.getClosedLoopController().setReference((TILT_UP), ControlType.kPosition,
            ClosedLoopSlot.kSlot0,feedForward);
        algaeMotor.set(0.0);
    }

    public void AlgaeBloom() {
        algaeTiltMotor.getEncoder().setPosition(TILT_DOWN);
    }

    // place special subsystem methods here
    // this is where rest of program can access functions to return
    // values or control the subsystem
    private double cmToRotations(double cm) {
        return cm * (gearRatio / gearCircumference);
        // 8.43 - 1 gear ratio
        // 3.588" diameter

    }
    private double rotationstoCm (double rotations) {
        return rotations * (gearCircumference / gearRatio);
        // 8.43 - 1 gear ratio
        // 3.588" diameter

    }

    // is the algaeTiltMotor at (or near) its target position
    public boolean isAlgaeTiltAtDestination() {

        // return true if elevator within 2cm of target position
        if (Math.abs(rotationstoCm(algaeTiltMotor.getEncoder().getPosition()) - 
                    TargetPositionCM) < 0.2)
            return true;
        else
            return false;

    }
    
}