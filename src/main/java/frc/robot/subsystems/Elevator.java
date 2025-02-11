package frc.robot.subsystems;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.LimitSwitchConfig.Type;
import com.revrobotics.spark.SparkBase.ResetMode;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

/** Subsystem */
public class Elevator extends SubsystemBase {
    static SparkMax elevatorMotorL = new SparkMax(RobotMap.CANID.L_ELEVATOR, MotorType.kBrushless);
    static SparkMax elevatorMotorR = new SparkMax(RobotMap.CANID.R_ELEVATOR, MotorType.kBrushless);
    SparkMaxConfig elevatorConfig = new SparkMaxConfig();
    // Local objects and variables here
    // These are for things that only belong to, and used by, the subsystem

    /** Place code here to initialize subsystem */
    public Elevator() {
        elevatorConfig.limitSwitch.reverseLimitSwitchEnabled(true);
        elevatorConfig.limitSwitch.forwardLimitSwitchEnabled(true);
        elevatorConfig.limitSwitch.forwardLimitSwitchType(Type.kNormallyClosed);
        elevatorConfig.limitSwitch.reverseLimitSwitchType(Type.kNormallyClosed);
        elevatorMotorL.configure(elevatorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        elevatorMotorR.configure(elevatorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    /**
     * Method called periodically by the scheduler
     * Place any code here you wish to have run periodically
     */
    @Override
    public void periodic() {

    }

    public void ElevatorUp() {
        elevatorMotorL.set(0.5);
        elevatorMotorR.set(0.5);
    }

    public void ElevatorDown() {

        elevatorMotorL.set(-0.5);
        elevatorMotorR.set(-0.5);

    }

    public void ElevatorStop() {
        elevatorMotorL.set(0);
        elevatorMotorR.set(0);
    }
    // place special subsystem methods here
    // this is where rest of program can access functions to return
    // values or control the subsystem

}
