package frc.robot.subsystems;

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
import frc.robot.RobotMap;

/** Subsystem */
public class Elevator extends SubsystemBase {
    static SparkMax elevatorMotorL = new SparkMax(RobotMap.CANID.L_ELEVATOR, MotorType.kBrushless);
    static SparkMax elevatorMotorR = new SparkMax(RobotMap.CANID.R_ELEVATOR, MotorType.kBrushless);
    SparkMaxConfig elevatorConfig;
    static double gearDiameterCM = 3.588 * 2.54;
    static double gearCircumference = gearDiameterCM * Math.PI;
    static double ticksPerRev = 1;
    public static double L2 = 81.0, L3 = 121.0, L4 = 183.0, L1 = 95.0, intake = 0.0; 
    static double ticksMoved;
    // Local objects and variables here
    // These are for things that only belong to, and used by, the subsystem

    /** Place code here to initialize subsystem */
    public Elevator() {
        // initialize limit switches, left motor follows right
        elevatorConfig = new SparkMaxConfig();
        elevatorConfig.limitSwitch.reverseLimitSwitchEnabled(false);
        elevatorConfig.limitSwitch.forwardLimitSwitchEnabled(false);
        elevatorConfig.limitSwitch.forwardLimitSwitchType(Type.kNormallyClosed);
        elevatorConfig.limitSwitch.reverseLimitSwitchType(Type.kNormallyClosed);
        elevatorConfig.idleMode(IdleMode.kCoast);
        elevatorConfig.inverted(true);
        elevatorConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder);
        elevatorConfig.closedLoop.p(0.5);
        elevatorConfig.closedLoop.i(0.0);
        elevatorConfig.closedLoop.d(0.0);
        elevatorConfig.closedLoop.outputRange(-0.5, 0.5);
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
        
    
     private void initializeShuffleboard() {
        // Create page in shuffleboard
        ShuffleboardTab Tab = Shuffleboard.getTab("elevator");
        ShuffleboardLayout l1 = Tab.getLayout("Elevator", BuiltInLayouts.kList);
        l1.withPosition(0, 0);
        l1.withSize(2, 4);
        m_left = l1.add("Left ", 0.0).getEntry();
        m_right = l1.add("Right", 0.0).getEntry();
        
    }

    private void updateShuffleboard(){
        m_left.setDouble(elevatorMotorL.getEncoder().getPosition());
        m_right.setDouble(elevatorMotorR.getEncoder().getPosition());

    }
    /**
     * Method called periodically by the scheduler
     * Place any code here you wish to have run periodically
     */
    @Override
    public void periodic() {
        updateShuffleboard();
    }

    public void Level1() {
        elevatorMotorL.getClosedLoopController().setReference(cmToTicks(L1), ControlType.kPosition, ClosedLoopSlot.kSlot0);
    }
    public void Level2() {
        elevatorMotorL.getClosedLoopController().setReference(cmToTicks(L2), ControlType.kPosition, ClosedLoopSlot.kSlot0);
    }
    public void Level3() {
        elevatorMotorL.getClosedLoopController().setReference(cmToTicks(L3), ControlType.kPosition, ClosedLoopSlot.kSlot0);
    }
    public void Level4() {
        elevatorMotorL.getClosedLoopController().setReference(cmToTicks(L4), ControlType.kPosition, ClosedLoopSlot.kSlot0);
    }
    
    public void returnToIntake(){
        ticksMoved = elevatorMotorL.getEncoder().getPosition();
        elevatorMotorL.getClosedLoopController().setReference(0 - (ticksMoved), ControlType.kPosition, ClosedLoopSlot.kSlot0);
    }
    

    private double cmToTicks(double cm){
        return cm/(gearCircumference/ticksPerRev);
        //8.43 - 1 gear ratio
        //3.588" diameter
        // 42 ticks/revolution(from spec sheet)
    }

    

}