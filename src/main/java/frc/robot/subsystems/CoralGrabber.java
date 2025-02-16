package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj.DigitalInput;
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
  

    // needs gear ratio factored in  
    private static double  Intake_Speed = 1;



    public CoralGrabber() {
        
        m_IntakeMotor1 = new SparkMax (RobotMap.CANID.IN1_INTAKE_Motor,MotorType.kBrushless);
        m_IntakeMotor2 = new SparkMax (RobotMap.CANID.IN2_INTAKE_Motor,MotorType.kBrushless);

    }

    /** Method called periodically by the scheduler
     * Place any code here you wish to have run periodically */
    @Override
    public void periodic() {

    }

    public void intakeRun(double state){
        m_IntakeMotor1.set(Intake_Speed * state);
        m_IntakeMotor2.set(-1.0* Intake_Speed * state);
    }

    // place special subsystem methods here
    // this is where rest of program can access functions to return
    // values or control the subsystem


}
