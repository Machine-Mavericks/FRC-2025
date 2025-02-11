package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;

import java.io.ObjectInputFilter.Config;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.LimitSwitchConfig.Type;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;
import com.revrobotics.spark.SparkBase;


/** Subsystem */
public class Climb extends SubsystemBase {

    // Local objects and variables here
    // These are for things that only belong to, and used by, the subsystem

    /** Place code here to initialize subsystem */

    private SparkMax m_ClimbMotor1Max;
    private SparkMax m_ClimbMotor2Max;
    SparkMaxConfig climbConfig = new SparkMaxConfig();
    
    // gear ratio is 80-1 and only needs to travel 90 degrees
    private static double  ClimbSpeed = 1;

    public Climb() {
        // still needs limit switches
        m_ClimbMotor1Max = new SparkMax (RobotMap.CANID.Cl1_Climb_Motor,MotorType.kBrushless);
        m_ClimbMotor2Max = new SparkMax (RobotMap.CANID.CL2_Climb_Motor,MotorType.kBrushless);
        climbConfig.limitSwitch.reverseLimitSwitchEnabled(true);
        climbConfig.limitSwitch.forwardLimitSwitchEnabled(true);
        climbConfig.limitSwitch.forwardLimitSwitchType(Type.kNormallyClosed);
        climbConfig.limitSwitch.reverseLimitSwitchType(Type.kNormallyClosed);
        m_ClimbMotor1Max.configure(climbConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        m_ClimbMotor2Max.configure(climbConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    }

    /** Method called periodically by the scheduler
     * Place any code here you wish to have run periodically */
    @Override
    public void periodic() {

    }

    public void climbRun(double state){
        m_ClimbMotor1Max.set(ClimbSpeed * state);
        m_ClimbMotor2Max.set(ClimbSpeed * state);
    }

    // place special subsystem methods here
    // this is where rest of program can access functions to return
    // values or control the subsystem


}
