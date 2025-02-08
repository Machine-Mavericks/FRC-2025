package frc.robot.subsystems;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;


/** Subsystem */
public class DeadWheel extends SubsystemBase {
    static Encoder encoder = new Encoder(0,1);
    static double distanceCM;
    // Local objects and variables here
    // These are for things that only belong to, and used by, the subsystem

    /** Place code here to initialize subsystem */
    public DeadWheel() {
       encoder.setDistancePerPulse(4.0/256.0);
        initializeShuffleboard();
    }

    /** Method called periodically by the scheduler
     * Place any code here you wish to have run periodically */
    @Override
    public void periodic() {
        updateShuffleboard();
       distanceCM = encoder.getDistance()*2;// in centimeters
    }
    private static GenericEntry m_distance;
    private static GenericEntry m_rate;
    private static void initializeShuffleboard() {
       
        // Create page in shuffleboard
        ShuffleboardTab Tab = Shuffleboard.getTab("DeadWheel");
        ShuffleboardLayout l1 = Tab.getLayout("DeadWheel", BuiltInLayouts.kList);
        l1.withPosition(0, 0);
        l1.withSize(2, 4);
        m_distance = l1.add("Total Distance (cm)", 0.0).getEntry();
        m_rate = l1.add("Rate ", 0.0).getEntry();
    }

    private static void updateShuffleboard(){
        m_distance.setDouble(distanceCM);
        m_rate.setDouble(encoder.getRate());
    }

    public static void ResetEncoder(){
        encoder.reset();
    }
    // place special subsystem methods here
    // this is where rest of program can access functions to return
    // values or control the subsystem


}
