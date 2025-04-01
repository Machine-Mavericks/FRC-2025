package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;


/** Subsystem */
public class CoralGrabber extends SubsystemBase {

    // Local objects and variables here
    // These are for things that only belong to, and used by, the subsystem

    /** Place code here to initialize subsystem */

    private SparkMax m_IntakeMotor1;
    private SparkMax m_IntakeMotor2;
    private DigitalInput photoSensor;
    private SparkMaxConfig intakeConfig;


    public CoralGrabber() {
        
        m_IntakeMotor1 = new SparkMax (RobotMap.CANID.IN1_INTAKE_Motor,MotorType.kBrushless);
        m_IntakeMotor2 = new SparkMax (RobotMap.CANID.IN2_INTAKE_Motor,MotorType.kBrushless);
        photoSensor =  new DigitalInput(RobotMap.DIO.photoSensor);
        intakeConfig = new SparkMaxConfig();
        intakeConfig.idleMode(IdleMode.kBrake);
        m_IntakeMotor1.configure(intakeConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        m_IntakeMotor2.configure(intakeConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    /** Method called periodically by the scheduler
     * Place any code here you wish to have run periodically */
    @Override
    public void periodic() {

    }

    public void intakeRun(double speed){
        m_IntakeMotor1.set(0-speed);
        m_IntakeMotor2.set(speed);
    }
   
    public void intakeRunTop(double speed) {
        m_IntakeMotor1.set(0-speed);
    }

    public void intakeRunBottom(double speed) {
        m_IntakeMotor2.set(speed);
    }


    public boolean getSensorState(){
        return !photoSensor.get();
    }
    
        
    // public void OutakeRun(double speed){
    //     m_IntakeMotor1.set(speed);
    //     m_IntakeMotor2.set(0-speed);
    // }
    

    // place special subsystem methods here
    // this is where rest of program can access functions to return
    // values or control the subsystem


}
