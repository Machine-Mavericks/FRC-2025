package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;


import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;


/** Subsystem */
public class AlgaeRemover extends SubsystemBase {

    // Local objects and variables here
    // These are for things that only belong to, and used by, the subsystem
    private SparkMax m_AlgaeMotor;
    private SparkMax m_AlgaeTiltMotor;
    /** Place code here to initialize subsystem */
    public AlgaeRemover() {
       m_AlgaeMotor = new SparkMax(RobotMap.CANID.ALGAE_MOTOR, MotorType.kBrushless);
       m_AlgaeTiltMotor = new SparkMax(RobotMap.CANID.ALGAE_TILT_MOTOR, MotorType.kBrushless);
    }

    /** Method called periodically by the scheduler
     * Place any code here you wish to have run periodically */
    @Override
    public void periodic() {

    }

    public void RemoveAlgae(double speed){
        m_AlgaeMotor.set(speed);
    }

    public void Tilt(double direction){
        m_AlgaeTiltMotor.set(direction);
    }
    // place special subsystem methods here
    // this is where rest of program can access functions to return
    // values or control the subsystem


}
