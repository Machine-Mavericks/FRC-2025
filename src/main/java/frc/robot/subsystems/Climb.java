package frc.robot.subsystems;

import java.io.ObjectInputFilter.Config;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.ForwardLimitTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.ReverseLimitTypeValue;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase;


/** Subsystem */
public class Climb extends SubsystemBase {

    // Local objects and variables here
    // These are for things that only belong to, and used by, the subsystem

    /** Place code here to initialize subsystem */

    private TalonFX m_ClimbMotor;


    double currentPos = 0;
    double extended = 1;
    double retracted = 0;

    public Climb() {
        m_ClimbMotor = new TalonFX(RobotMap.CANID.Cl_Climb_Motor);
        TalonFXConfiguration config= new TalonFXConfiguration();
        config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        // limit switch 
        config.HardwareLimitSwitch.ForwardLimitType = ForwardLimitTypeValue.NormallyOpen;
        config.HardwareLimitSwitch.ReverseLimitType = ReverseLimitTypeValue.NormallyOpen;
        config.HardwareLimitSwitch.ForwardLimitEnable = true;
        config.HardwareLimitSwitch.ReverseLimitEnable = false; 


        m_ClimbMotor.getConfigurator().apply(config);

        initializeShuffleboard();

    }

    /** Method called periodically by the scheduler
     * Place any code here you wish to have run periodically */

    @Override
    public void periodic() {
        updateShuffleboard();
    }


    public void SetSpeed(double speed){
        m_ClimbMotor.set(speed);
    }

    public boolean getSwitchState(){
        if (m_ClimbMotor.getReverseLimit().getValue().value == 1){
            return true; 
        }
        else{
            return false; 
        }
    }


    

    private static GenericEntry m_switchF;
    private static GenericEntry m_switchR;
    


    private void initializeShuffleboard() {
        // Create page in shuffleboard
        ShuffleboardTab Tab = Shuffleboard.getTab("climb");
        ShuffleboardLayout l1 = Tab.getLayout("Climb", BuiltInLayouts.kList);
        l1.withPosition(0, 0);
        l1.withSize(2, 4);

    
        m_switchF = l1.add("switchF",0).getEntry();
        m_switchR = l1.add("switchR", 0).getEntry();
        
    }

    private void updateShuffleboard() {
        m_switchF.setInteger(m_ClimbMotor.getForwardLimit().getValue().value);
        m_switchR.setInteger(m_ClimbMotor.getReverseLimit().getValue().value);
       

    }



    // place special subsystem methods here
    // this is where rest of program can access functions to return
    // values or control the subsystem


}
