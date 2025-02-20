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

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

/** Subsystem */
public class Elevator extends SubsystemBase {
    static SparkMax elevatorMotorL = new SparkMax(RobotMap.CANID.L_ELEVATOR, MotorType.kBrushless);
    static SparkMax elevatorMotorR = new SparkMax(RobotMap.CANID.R_ELEVATOR, MotorType.kBrushless);
    SparkMaxConfig elevatorConfig;
    static double gearDiameterCM = 3.588 * 2.54;
    static double gearCircumference = gearDiameterCM * Math.PI;
    static double ticksPerRev = 42;
    public static double L2 = 81.0, L3 = 121.0, L4 = 183.0, L1 = 95.0, intake = 0.0; 
    private static double ELEVATOR_SPEED = 1; 
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
        elevatorConfig.idleMode(IdleMode.kBrake);
        elevatorConfig.inverted(false);
        elevatorConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder);
        elevatorConfig.closedLoop.p(0.5);
        elevatorConfig.closedLoop.i(0.0);
        elevatorConfig.closedLoop.d(0.0);
        elevatorConfig.closedLoop.outputRange(-0.1, 0.1);
        elevatorConfig.closedLoop.positionWrappingEnabled(false);
        elevatorConfig.encoder.positionConversionFactor(1);
        elevatorMotorL.configure(elevatorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        SparkMaxConfig followConfig = new SparkMaxConfig();
        followConfig.follow(elevatorMotorL, false);
        elevatorMotorR.configure(followConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        ticksMoved = 0;
    
        
    }

    /**
     * Method called periodically by the scheduler
     * Place any code here you wish to have run periodically
     */
    @Override
    public void periodic() {
        
    }

    public void Level1() {
        elevatorMotorL.getClosedLoopController().setReference(L1, ControlType.kPosition, ClosedLoopSlot.kSlot0);
    }
    public void Level2() {
        elevatorMotorL.getClosedLoopController().setReference(L2, ControlType.kPosition, ClosedLoopSlot.kSlot0);
    }
    public void Level3() {
        elevatorMotorL.getClosedLoopController().setReference(L3, ControlType.kPosition, ClosedLoopSlot.kSlot0);
    }
    public void Level4() {
        elevatorMotorL.getClosedLoopController().setReference(L4, ControlType.kPosition, ClosedLoopSlot.kSlot0);
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