package frc.robot.subsystems;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;


/** Subsystem */
public class DeadWheel extends SubsystemBase {
    
    // deadwheel encoders
    private Encoder encoderLeft = new Encoder(RobotMap.DIO.LEFTENCODER_A,RobotMap.DIO.LEFTENCODER_B);
    private Encoder encoderFront = new Encoder(RobotMap.DIO.FRONTENCODER_A, RobotMap.DIO.FRONTENCODER_B);
    private Encoder encoderRear = new Encoder(RobotMap.DIO.REARENCODER_A, RobotMap.DIO.REARENCODER_B);
    

    public static double FRONT_TO_BACK_DISTANCE = 0.568;
    public static double LATERAL_OFFSET = 0.285;
    

    /** Place code here to initialize subsystem */
    public DeadWheel() {
        encoderLeft.setDistancePerPulse(1.05373*2.54*4.0*Math.PI/1024.0);
        encoderLeft.setReverseDirection(true);
        encoderFront.setDistancePerPulse(1.06322*2.54*4.0*Math.PI/1024.0);
        encoderFront.setReverseDirection(true);
        encoderRear.setDistancePerPulse(1.06322*2.54*4.0*Math.PI/1024.0);
        encoderRear.setReverseDirection(false);
        initializeShuffleboard();
    }

    /** Method called periodically by the scheduler
     * Place any code here you wish to have run periodically */
    @Override
    public void periodic() {
        updateShuffleboard();

    }


    private static GenericEntry m_left;
    private static GenericEntry m_rear;
    private static GenericEntry m_front;
  
    private void initializeShuffleboard() {
        // Create page in shuffleboard
        ShuffleboardTab Tab = Shuffleboard.getTab("DeadWheel");
        ShuffleboardLayout l1 = Tab.getLayout("DeadWheel", BuiltInLayouts.kList);
        l1.withPosition(0, 0);
        l1.withSize(2, 4);
        m_left = l1.add("Left ", 0.0).getEntry();
        m_rear = l1.add("Rear ", 0.0).getEntry();
        m_front = l1.add("Front ", 0.0).getEntry();
    }

    private void updateShuffleboard(){
        m_left.setDouble(encoderLeft.getDistance());
        m_front.setDouble(encoderFront.getDistance());
        m_rear.setDouble(encoderRear.getDistance());
    }

    public void ResetEncoder(){
        encoderLeft.reset();
        encoderFront.reset();
        encoderRear.reset();
    }

    public double getLeftEncoderDistance(){
        return encoderLeft.getDistance();
    }
    public double getFrontEncoderDistance(){
        return encoderFront.getDistance();
    }
    public double getRearEncoderDistance(){
        return encoderRear.getDistance();
    }

   
   // in centimeters
    // place special subsystem methods here
    // this is where rest of program can access functions to return
    // values or control the subsystem


}
