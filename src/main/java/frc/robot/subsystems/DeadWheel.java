package frc.robot.subsystems;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;


/** Subsystem */
public class DeadWheel extends SubsystemBase {
    
    // deadwheel encoders
    static Encoder encoderLeft = new Encoder(RobotMap.DIO.LEFTENCODER_A,RobotMap.DIO.LEFTENCODER_B);
    static Encoder encoderRight = new Encoder(RobotMap.DIO.RIGHTENCODER_A, RobotMap.DIO.RIGHTENCODER_B);
    static Encoder encoderFront = new Encoder(RobotMap.DIO.FRONTENCODER_A, RobotMap.DIO.FRONTENCODER_B);
    
    static double distanceCM;
    static double LATERAL_DISTANCE = 1, FORWARD_OFFSET = 1;
    

    /** Place code here to initialize subsystem */
    public DeadWheel() {
       encoderLeft.setDistancePerPulse(4.0/256.0);
       encoderRight.setDistancePerPulse(4.0/256.0);
       encoderFront.setDistancePerPulse(4.0/256.0);
        initializeShuffleboard();
    }

    /** Method called periodically by the scheduler
     * Place any code here you wish to have run periodically */
    @Override
    public void periodic() {
        updateShuffleboard();

    }

    private static GenericEntry m_left;
    private static GenericEntry m_right;
    private static GenericEntry m_front;
    private void initializeShuffleboard() {
        // Create page in shuffleboard
        ShuffleboardTab Tab = Shuffleboard.getTab("DeadWheel");
        ShuffleboardLayout l1 = Tab.getLayout("DeadWheel", BuiltInLayouts.kList);
        l1.withPosition(0, 0);
        l1.withSize(2, 4);
        m_left = l1.add("Left ", 0.0).getEntry();
        m_right = l1.add("Right ", 0.0).getEntry();
        m_front = l1.add("Front ", 0.0).getEntry();
    }

    private void updateShuffleboard(){
        m_left.setDouble(encoderLeft.getDistance());
        m_front.setDouble(encoderRight.getDistance());
        m_right.setDouble(encoderRight.getDistance());
    }

    public static void ResetEncoder(){
        encoderLeft.reset();
        encoderRight.reset();
        encoderFront.reset();
    }

    public static double getLeftEncoderDistance(){
        convertToCm(encoderLeft);
        return distanceCM;
    }
    public static double getRightEncoderDistance(){
        convertToCm(encoderRight);
        return distanceCM;
    }
    public static double getFrontEncoderDistance(){
        convertToCm(encoderFront);
        return distanceCM;
    }
    public static double convertToCm(Encoder encoder){
        distanceCM = encoder.getDistance()*2;
        return distanceCM;
    }
   // in centimeters
    // place special subsystem methods here
    // this is where rest of program can access functions to return
    // values or control the subsystem


}
