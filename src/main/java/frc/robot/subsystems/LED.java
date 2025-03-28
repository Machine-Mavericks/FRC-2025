package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.RobotMap;

/** Subsystem */
public class LED extends SubsystemBase {

    // Local objects and variables here
    // These are for things that only belong to, and used by, the subsystem

    /** Place code here to initialize subsystem */
    // Addressable LED and buffer
    AddressableLED m_led;
    AddressableLEDBuffer m_ledBuffer;

    private final int NumLEDs = 47;

    // used for simple light strobe
    private int StrobeIndex = 0;

    // timer to slow down strobe
    private int timer;

    // PowerDistribution robot_pdp = new PowerDistribution();

    public LED() {
        // create addressble LED controller
        m_led = new AddressableLED(RobotMap.PWMPorts.LED_BLINKING);
        m_led.setLength(NumLEDs);

        // create LED buffer - length is # of LEDs in string
        m_ledBuffer = new AddressableLEDBuffer(NumLEDs);

        // Set the data
        // m_led.setLength (m_ledBuffer.getLength());
        m_led.setData(m_ledBuffer);
        m_led.start();
    }

    /**
     * Method called periodically by the scheduler
     * Place any code here you wish to have run periodically
     */
    @Override
    public void periodic() {
        timer++;
        if (timer >= 1)
            timer = 0;

        if (timer == 0) {
            StrobeIndex ++;
            if (StrobeIndex >= NumLEDs){
                StrobeIndex = 0;
            }
        
            for (int i = 0; i < NumLEDs; ++i){
                m_ledBuffer.setRGB(i, 0, 0, 0);
            }            
        }
        
        
        if(RobotContainer.camr.isTargetPresent()){
            m_ledBuffer.setRGB(StrobeIndex, 255, 255, 255);
            if (StrobeIndex>0)
                m_ledBuffer.setRGB(StrobeIndex-1, 255, 255, 255);
            if (StrobeIndex>1)
                m_ledBuffer.setRGB(StrobeIndex-2, 255, 255, 255);
            if (StrobeIndex>2)
                m_ledBuffer.setRGB(StrobeIndex-3, 255, 255, 255);
        }else if(RobotContainer.camleft.isTargetPresent()) {
            m_ledBuffer.setRGB(StrobeIndex, 115, 255, 0);
            if (StrobeIndex>0)
                m_ledBuffer.setRGB(StrobeIndex-1, 115, 255, 0);
            if (StrobeIndex>1)
                m_ledBuffer.setRGB(StrobeIndex-2, 115, 255, 0);
            if (StrobeIndex>2)
                m_ledBuffer.setRGB(StrobeIndex-3, 115, 255, 0);

        } else {
        if (DriverStation.getAlliance().get() == Alliance.Blue) {
            m_ledBuffer.setRGB(StrobeIndex, 0, 0, 255);
            if (StrobeIndex>0)
                m_ledBuffer.setRGB(StrobeIndex-1, 0, 0, 255);
            if (StrobeIndex>1)
                m_ledBuffer.setRGB(StrobeIndex-2, 0, 0, 255);
            if (StrobeIndex>2)
                m_ledBuffer.setRGB(StrobeIndex-3, 0, 0, 255);
        }else {
            m_ledBuffer.setRGB(StrobeIndex, 255, 0, 0);
            if (StrobeIndex>0)
                m_ledBuffer.setRGB(StrobeIndex-1, 255, 0, 0);
            if (StrobeIndex>1)
                m_ledBuffer.setRGB(StrobeIndex-2, 255, 0, 0);
            if (StrobeIndex>2)
                m_ledBuffer.setRGB(StrobeIndex-3, 255, 0, 0);
        }
    }




        m_led.setData(m_ledBuffer);
        }




          
        
    
            }
    
        
    

