package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.LimitSwitchConfig.Type;
import com.revrobotics.spark.config.LimitSwitchConfig;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

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

    // Local objects and variables here
    // These are for things that only belong to, and used by, the subsystem
    private SparkMax m_AlgaeMotor;
    private static SparkMax m_AlgaeTiltMotor;
    /** Place code here to initialize subsystem */
    public AlgaeRemover() {
        m_AlgaeMotor = new SparkMax(RobotMap.CANID.ALGAE_MOTOR, MotorType.kBrushless);
        m_AlgaeTiltMotor = new SparkMax(RobotMap.CANID.ALGAE_TILT_MOTOR, MotorType.kBrushless);
        
        SparkBaseConfig config = new SparkMaxConfig()
                .smartCurrentLimit(5)
                .apply(new LimitSwitchConfig()
                    .reverseLimitSwitchEnabled(true)
                    .reverseLimitSwitchType(Type.kNormallyOpen)
                    .forwardLimitSwitchEnabled(false));

        m_AlgaeTiltMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        initializeShuffleboard();
    }

    private GenericEntry m_AlgaeMotorEntry;
    private GenericEntry m_AlgaeTiltMotorEntry;
    private static GenericEntry m_current;

    private void initializeShuffleboard() {
        // Create page in shuffleboard
        ShuffleboardTab Tab = Shuffleboard.getTab("Algae Remover");
        ShuffleboardLayout l1 = Tab.getLayout("Algae", BuiltInLayouts.kList);
        l1.withPosition(0, 0);
        l1.withSize(2, 4);
        m_AlgaeMotorEntry = l1.add("motor", 0.0).getEntry();
        m_AlgaeTiltMotorEntry = l1.add("tilt", 0.0).getEntry();
        m_current = l1.add("current", 0.0).getEntry();

    }

    private void updateShuffleboard() {
        m_AlgaeMotorEntry.setDouble((m_AlgaeMotor.getEncoder().getPosition()));
        m_AlgaeTiltMotorEntry.setDouble((m_AlgaeTiltMotor.getEncoder().getPosition()));
        m_current.setDouble(m_AlgaeTiltMotor.getOutputCurrent());

    }

    /**
     * Method called periodically by the scheduler
     * Place any code here you wish to have run periodically
     */
    @Override
    public void periodic() {
        if(m_AlgaeTiltMotor.getReverseLimitSwitch().isPressed()){
           m_AlgaeTiltMotor.getEncoder().setPosition(0);
        }
        m_AlgaeTiltMotor.getEncoder().getPosition();
        updateShuffleboard();


    }

    public void RemoveAlgae(double speed) {
        m_AlgaeMotor.set(speed);
    }

    public void Tilt(double direction) {
        System.out.println(direction);
        m_AlgaeTiltMotor.set(direction);
    }

    
    // place special subsystem methods here
    // this is where rest of program can access functions to return
    // values or control the subsystem

}