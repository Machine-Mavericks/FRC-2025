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
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

/** Subsystem */
public class Elevator extends SubsystemBase {
    static SparkMax elevatorMotorL = new SparkMax(RobotMap.CANID.L_ELEVATOR, MotorType.kBrushless);
    static SparkMax elevatorMotorR = new SparkMax(RobotMap.CANID.R_ELEVATOR, MotorType.kBrushless);
    SparkMaxConfig elevatorConfig = new SparkMaxConfig();
    static double gearDiameterCM = 3.588 * 2.54;
    static double gearCircumference = gearDiameterCM * Math.PI;
    static double ticksPerRev = 42;
    static double lastPosTicks;
    public static double low = 81.0, med = 121.0, high = 183.0, trough = 95.0; 
    private static double ELEVATOR_SPEED = 1; 
    // Local objects and variables here
    // These are for things that only belong to, and used by, the subsystem

    /** Place code here to initialize subsystem */
    public Elevator() {
        // initialize limit switches, left motor follows right
        elevatorConfig.limitSwitch.reverseLimitSwitchEnabled(true);
        elevatorConfig.limitSwitch.forwardLimitSwitchEnabled(true);
        elevatorConfig.limitSwitch.forwardLimitSwitchType(Type.kNormallyClosed);
        elevatorConfig.limitSwitch.reverseLimitSwitchType(Type.kNormallyClosed);
        elevatorConfig.idleMode(IdleMode.kBrake);
        elevatorConfig.inverted(false);
        elevatorConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder);
        elevatorConfig.closedLoop.p(104.0);
        elevatorConfig.closedLoop.i(0.0);
        elevatorConfig.closedLoop.d(0.0);
        elevatorConfig.closedLoop.outputRange(-0.6, 0.6);
        elevatorConfig.closedLoop.positionWrappingEnabled(false);
        elevatorMotorL.configure(elevatorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        SparkMaxConfig followConfig = new SparkMaxConfig();
        followConfig.follow(elevatorMotorL, false);
        elevatorMotorR.configure(followConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        lastPosTicks = 0.0;
        
    }

    /**
     * Method called periodically by the scheduler
     * Place any code here you wish to have run periodically
     */
    @Override
    public void periodic() {

    }

    public void moveToPosition(Double position) {
        cmToTicks(position);      
    }
    
    private double cmToTicks(double cm){
        return cm/(gearCircumference/ticksPerRev);
        //8.43 - 1 gear ratio
        //3.588" diameter
        // 42 ticks/revolution(from spec sheet)
    }

    public void elevatorRun(double state) {
        elevatorMotorR.getClosedLoopController().setReference(state, ControlType.kPosition, ClosedLoopSlot.kSlot0);
        elevatorMotorL.getClosedLoopController().setReference(state, ControlType.kPosition, ClosedLoopSlot.kSlot0);
      }
    // place special subsystem methods here
    // this is where rest of program can access functions to return
    // values or control the subsystem

}