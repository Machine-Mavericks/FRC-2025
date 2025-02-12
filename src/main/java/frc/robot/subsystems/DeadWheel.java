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
    private Encoder encoderLeft = new Encoder(RobotMap.DIO.LEFTENCODER_A,RobotMap.DIO.LEFTENCODER_B);
    private Encoder encoderFront = new Encoder(RobotMap.DIO.FRONTENCODER_A, RobotMap.DIO.FRONTENCODER_B);
    private Encoder encoderRear = new Encoder(RobotMap.DIO.REARENCODER_A, RobotMap.DIO.REARENCODER_B);
    
    private double distanceCM;
    public static double FRONT_TO_BACK_DISTANCE = 1.0;
    public static double LATERAL_OFFSET = 1.0;
    

    /** Place code here to initialize subsystem */
    public DeadWheel() {
        encoderLeft.setDistancePerPulse(4.0/256.0);
        encoderFront.setDistancePerPulse(4.0/256.0);
        encoderRear.setDistancePerPulse(4.0/256.0);
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
        m_right = l1.add("Rear ", 0.0).getEntry();
        m_front = l1.add("Front ", 0.0).getEntry();
    }

    private void updateShuffleboard(){
        m_left.setDouble(encoderLeft.getDistance());
        m_front.setDouble(encoderRear.getDistance());
        m_right.setDouble(encoderFront.getDistance());
    }

    public void ResetEncoder(){
        encoderLeft.reset();
        encoderFront.reset();
        encoderRear.reset();
    }

    public double getLeftEncoderDistance(){
        convertToCm(encoderLeft);
        return distanceCM;
    }
    public double getFrontEncoderDistance(){
        convertToCm(encoderFront);
        return distanceCM;
    }
    public double getRearEncoderDistance(){
        convertToCm(encoderRear);
        return distanceCM;
    }

    public double convertToCm(Encoder encoder){
        distanceCM = encoder.getDistance()*2;
        return distanceCM;
    }
   // in centimeters
    // place special subsystem methods here
    // this is where rest of program can access functions to return
    // values or control the subsystem


}
