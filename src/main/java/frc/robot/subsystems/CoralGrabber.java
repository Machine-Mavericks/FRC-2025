package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;

import java.util.Map;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
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

    double speedAdjust1 = 1;
    double speedAdjust2 = 1;


    public CoralGrabber() {
        
        m_IntakeMotor1 = new SparkMax (RobotMap.CANID.IN1_INTAKE_Motor,MotorType.kBrushless);
        m_IntakeMotor2 = new SparkMax (RobotMap.CANID.IN2_INTAKE_Motor,MotorType.kBrushless);
        photoSensor =  new DigitalInput(RobotMap.DIO.photoSensor);
        intakeConfig = new SparkMaxConfig();
        intakeConfig.idleMode(IdleMode.kBrake);
        m_IntakeMotor1.configure(intakeConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        m_IntakeMotor2.configure(intakeConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        InitializeShuffleboard();
    }
    private static GenericEntry m_sensor;
    private static GenericEntry m_motor1;
    private static GenericEntry m_motor2;
    private static GenericEntry m_motor1speed;
    private static GenericEntry m_motor2speed;

    private void InitializeShuffleboard(){
        ShuffleboardTab Tab = Shuffleboard.getTab("Intake");
        ShuffleboardLayout l1 = Tab.getLayout("Intake", BuiltInLayouts.kList);
        l1.withPosition(0, 0);
        l1.withSize(2, 4);
        m_sensor = l1.add("Sensor ", false).getEntry();
        m_motor1 = l1.add("Motor 1 ", 0.0).getEntry();
        m_motor2 = l1.add("Motor 2 ", 0.0).getEntry();
        m_motor1speed = Tab.add("Motor 1 Speed", 0)
                        .withWidget(BuiltInWidgets.kNumberSlider)
                        .withPosition(2, 0).
                        withSize(2, 1).
                        withProperties(Map.of("min", -5, "max", 5))
                        .getEntry();
        m_motor2speed = Tab.add("Motor 2 Speed", 0)
                        .withWidget(BuiltInWidgets.kNumberSlider)
                        .withPosition(2, 1).
                        withSize(2, 1).
                        withProperties(Map.of("min", -5, "max", 5))
                        .getEntry();
        

    }
    private void UpdateShuffleboard(){
        m_sensor.setBoolean(getSensorState());
        m_motor1.setDouble(m_IntakeMotor1.get());
        m_motor2.setDouble(m_IntakeMotor2.get());
       
    }

    /** Method called periodically by the scheduler
     * Place any code here you wish to have run periodically */
    @Override
    public void periodic() {
        UpdateShuffleboard();
    }

    public void intakeRun(double speed){
        m_IntakeMotor1.set(0-speed*speedAdjust1);
        m_IntakeMotor2.set(speed*speedAdjust2);
        speedAdjust1 = m_motor1speed.getDouble(1);
        speedAdjust2 = m_motor2speed.getDouble(1);
        
       
    }

    public void sillyIntakeRun(double speed){
        m_IntakeMotor1.set(speed*speedAdjust1);
        m_IntakeMotor2.set(speed*speedAdjust2);
        speedAdjust1 = m_motor1speed.getDouble(1);
        speedAdjust2 = m_motor2speed.getDouble(1);
        
       
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
