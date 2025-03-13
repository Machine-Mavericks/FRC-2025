package frc.robot.subsystems;

import java.lang.annotation.Target;

import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.LimitSwitchConfig.Type;
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
public class Elevator extends SubsystemBase {
    static SparkMax elevatorMotorL = new SparkMax(RobotMap.CANID.L_ELEVATOR, MotorType.kBrushless);
    static SparkMax elevatorMotorR = new SparkMax(RobotMap.CANID.R_ELEVATOR, MotorType.kBrushless);
    SparkMaxConfig elevatorConfig;
    static double gearDiameterCM = 3.588 * 2.54;
    static double gearCircumference = gearDiameterCM * Math.PI;
    static double gearRatio = 8.43, paddingOffset = 0;
    public static double L2 = 23.0, L3 = 44.0, L4 = 73.0, L1 = 10.7, L0 = 0.0; // in cm
    static double ticksMoved;
    static double feedForward = 0.45;
    double TargetPositionCM = 0.0;

    // Local objects and variables here
    // These are for things that only belong to, and used by, the subsystem

    /** Place code here to initialize subsystem */
    public Elevator() {
        // initialize limit switches, left motor follows right
        elevatorConfig = new SparkMaxConfig();
        elevatorConfig.limitSwitch.reverseLimitSwitchEnabled(true);//change to true
        elevatorConfig.limitSwitch.forwardLimitSwitchEnabled(true);//change to true
        elevatorConfig.limitSwitch.forwardLimitSwitchType(Type.kNormallyClosed);
        elevatorConfig.limitSwitch.reverseLimitSwitchType(Type.kNormallyClosed);
        elevatorConfig.idleMode(IdleMode.kBrake);// change to break later
        elevatorConfig.inverted(true);
        elevatorConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder);
        elevatorConfig.closedLoop.p(0.25);
        elevatorConfig.closedLoop.i(0.0);
        elevatorConfig.closedLoop.d(0.0);
        elevatorConfig.closedLoop.outputRange(-0.1, 0.4);
        elevatorConfig.closedLoop.positionWrappingEnabled(false);
        elevatorConfig.encoder.positionConversionFactor(1);
        elevatorMotorL.configure(elevatorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        SparkMaxConfig followConfig = new SparkMaxConfig();
        followConfig.follow(elevatorMotorL, false);
        elevatorMotorR.configure(followConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        ticksMoved = 0;
        initializeShuffleboard();
    }

    private static GenericEntry m_left;
    private static GenericEntry m_right;
    private static GenericEntry m_switchF;
    private static GenericEntry m_switchR;
    private static GenericEntry m_current;

    private void initializeShuffleboard() {
        // Create page in shuffleboard
        ShuffleboardTab Tab = Shuffleboard.getTab("elevator");
        ShuffleboardLayout l1 = Tab.getLayout("Elevator", BuiltInLayouts.kList);
        l1.withPosition(0, 0);
        l1.withSize(2, 4);
        m_left = l1.add("Left ", 0.0).getEntry();
        m_right = l1.add("Right", 0.0).getEntry();
        m_switchF = l1.add("switchF",false).getEntry();
        m_switchR = l1.add("switchR", false).getEntry();
        m_current = l1.add("current", 0.0).getEntry();

    }

    private void updateShuffleboard() {
        m_left.setDouble(rotationstoCm(elevatorMotorL.getEncoder().getPosition()));
        m_right.setDouble(rotationstoCm(elevatorMotorR.getEncoder().getPosition()));
        m_switchR.setBoolean(elevatorMotorL.getReverseLimitSwitch().isPressed());
        m_switchF.setBoolean(elevatorMotorL.getForwardLimitSwitch().isPressed());
        m_current.setDouble(elevatorMotorL.getOutputCurrent());

    }
    public void ZeroEncoder(){
        elevatorMotorL.getEncoder().setPosition(0);
        elevatorMotorR.getEncoder().setPosition(0);
        elevatorMotorL.getClosedLoopController().setReference(0,ControlType.kPosition);
    }

    /**
     * Method called periodically by the scheduler
     * Place any code here you wish to have run periodically
     */
    @Override
    public void periodic() {
        updateShuffleboard();
        if(elevatorMotorL.getForwardLimitSwitch().isPressed()){
            ZeroEncoder();
        }
    }

    public void Level1() {
        TargetPositionCM = L1;
        elevatorMotorL.getClosedLoopController().setReference(cmToRotations(L1), ControlType.kPosition,
                ClosedLoopSlot.kSlot0, feedForward);
    }

    public void Level2() {
        TargetPositionCM = L2;
        elevatorMotorL.getClosedLoopController().setReference(cmToRotations(L2), ControlType.kPosition,
                ClosedLoopSlot.kSlot0, feedForward);
    }

    public void Level3() {
        TargetPositionCM = L3;
        elevatorMotorL.getClosedLoopController().setReference(cmToRotations(L3), ControlType.kPosition,
                ClosedLoopSlot.kSlot0, feedForward);
    }

    public void Level4() {
        TargetPositionCM = L4;
        elevatorMotorL.getClosedLoopController().setReference(cmToRotations(L4), ControlType.kPosition,
                ClosedLoopSlot.kSlot0, feedForward);
    }

    public void Level0() {
        TargetPositionCM = 0.0;
        ticksMoved = elevatorMotorL.getEncoder().getPosition();
        elevatorMotorL.getClosedLoopController().setReference(0 - (ticksMoved), ControlType.kPosition,
                ClosedLoopSlot.kSlot0, feedForward);
    }

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

    // is elevator at (or near) its target position
    public boolean isElevatorAtDestination() {

        // return true if elevator within 2cm of target position
        if (Math.abs(rotationstoCm(elevatorMotorL.getEncoder().getPosition()) - 
                    TargetPositionCM) < 2.0)
            return true;
        else
            return false;

    }

}