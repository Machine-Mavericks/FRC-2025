package frc.robot.subsystems;

import com.ctre.phoenix6.signals.BrushedMotorWiringValue;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.RobotMap;

/** Subsystem */
public class Elevator extends SubsystemBase {
    static SparkMax elevatorMotorL = new SparkMax(RobotMap.CANID.L_ELEVATOR, MotorType.kBrushless);
    static SparkMax elevatorMotorR = new SparkMax(RobotMap.CANID.R_ELEVATOR, MotorType.kBrushless);
    static DigitalInput maxLimitSwitch = new DigitalInput(0);
    static DigitalInput minLimitSwitch = new DigitalInput(1);
    static boolean heightMax = false;
    static boolean heightMin = true;
    // Local objects and variables here
    // These are for things that only belong to, and used by, the subsystem

    /** Place code here to initialize subsystem */
    public Elevator() {
        heightMax = false;
        heightMin = true;
    }

    /**
     * Method called periodically by the scheduler
     * Place any code here you wish to have run periodically
     */
    @Override
    public void periodic() {
      if (maxLimitSwitch.get()){ // elevator reaches max height
        heightMax = true;
      }
      if (minLimitSwitch.get()){ // elevator reaches max height
        heightMin = true;
      }
    }

    public void ElevatorUp() {
        while (!heightMax){
        elevatorMotorL.set(0.5);
        elevatorMotorR.set(0.5);
        }
       ElevatorStop();
    }

    public void ElevatorDown() {
        while (!heightMin){
        elevatorMotorL.set(-0.5);
        elevatorMotorR.set(-0.5);
        }
       ElevatorStop();
    }

    public void ElevatorStop() {
        elevatorMotorL.set(0);
        elevatorMotorR.set(0);
    }
    // place special subsystem methods here
    // this is where rest of program can access functions to return
    // values or control the subsystem

}
