package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;

import java.io.ObjectInputFilter.Config;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.LimitSwitchConfig.Type;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase;


/** Subsystem */
public class Climb extends SubsystemBase {

    // Local objects and variables here
    // These are for things that only belong to, and used by, the subsystem

    /** Place code here to initialize subsystem */

    private SparkMax m_ClimbMotor;
    SparkMaxConfig climbConfig = new SparkMaxConfig();
    double currentPos = 0;
    double extended = 1;
    double retracted = 0;

    public Climb() {
        m_ClimbMotor = new SparkMax (RobotMap.CANID.Cl_Climb_Motor,MotorType.kBrushless);
        climbConfig.idleMode(IdleMode.kBrake);
        climbConfig.inverted(false);
        climbConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder);
        climbConfig.closedLoop.p(0.0);
        climbConfig.closedLoop.i(0.0);
        climbConfig.closedLoop.d(0.0);
        climbConfig.closedLoop.outputRange(-1.0,1.0);
        climbConfig.closedLoop.positionWrappingEnabled(false);
        m_ClimbMotor.configure(climbConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        climbConfig.encoder.positionConversionFactor(1);

    }

    /** Method called periodically by the scheduler
     * Place any code here you wish to have run periodically */
    @Override
    public void periodic() {
        currentPos = m_ClimbMotor.getEncoder().getPosition();
    }

    public void Extend(){
       m_ClimbMotor.getClosedLoopController().setReference(currentPos - extended, ControlType.kPosition);
    }

    public void Retract(){
       m_ClimbMotor.getClosedLoopController().setReference(currentPos - retracted, ControlType.kPosition);
    }

    // place special subsystem methods here
    // this is where rest of program can access functions to return
    // values or control the subsystem


}
