package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;


public class NavX extends SubsystemBase {
  
    // make our gyro object
    private AHRS gyro;

    // gyro offset
    private double YawAngleOffset;


    /** Creates a new Gyro. */
    public NavX() {
        // create gyro and initialize it
        YawAngleOffset = 0.0;
        gyro = new AHRS(NavXComType.kMXP_SPI);
        gyro.reset();
    
        // initialize shuffleboard for reporting gyro data
        initializeShuffleboard();
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        updateShuffleboard();
    } 

    /** Gets the yaw of the robot
    * @return current yaw value (-180 to 180) */
    public double getYawAngle() {
        return (-gyro.getYaw())+YawAngleOffset;   // note: navx +ve is opposite of FRC defined axes
    }

    // resets gyro and offset value
    public void resetYawAngle() {
        setYawAngle(0.0);
    }

    // sets gyro to provided angle (in deg)
    public void setYawAngle(double angle) {
        YawAngleOffset -= getYawAngle() - angle;
    }


    /** Gets the pitch of the robot
    * @return current pitch value (-180 to 180) */
    public double getPitchAngle() {
        return gyro.getPitch();
    }

    /** Get Roll
    * @return -180 to 180 degrees */
    public double getRollAngle() {
        return gyro.getRoll();
    }

    /** X Acceleration
    * @return ratio of gravity */
    public double getXAcceleration() {
        return gyro.getRawAccelX();
    }

    /** Y Acceleration
    * @return ratio of gravity */
    public double getYAcceleration() {
        return gyro.getRawAccelY();
    }


    /** Gyro Shuffleboard */

  
    // -------------------- Subsystem Shuffleboard Methods --------------------

    // subsystem shuffleboard controls
    private GenericEntry m_gyroPitch;
    private GenericEntry m_gyroYaw;
    private GenericEntry m_gyroRoll;
    private GenericEntry m_xAcceleration;
    private GenericEntry m_yAcceleration;


    /** Initialize subsystem shuffleboard page and controls */
    private void initializeShuffleboard() {
        // Create odometry page in shuffleboard
        ShuffleboardTab Tab = Shuffleboard.getTab("NavX");

        // create controls to display robot position, angle, and gyro angle
        ShuffleboardLayout l1 = Tab.getLayout("NavX", BuiltInLayouts.kList);
        l1.withPosition(0, 0);
        l1.withSize(2, 4);
        m_gyroPitch = l1.add("Pitch (deg)", 0.0).getEntry();
        m_gyroYaw = l1.add("Yaw (deg)", 0.0).getEntry();
        m_gyroRoll = l1.add("Roll (deg)", 0.0).getEntry();
        m_xAcceleration = l1.add("X Acceleration", 0.0).getEntry();
        m_yAcceleration = l1.add("Y Acceleration", 0.0).getEntry();
    }

    /** Update subsystem shuffle board page with current Gyro values */
    private void updateShuffleboard() {
        // write current robot Gyro
        m_gyroPitch.setDouble(getPitchAngle());
        m_gyroYaw.setDouble(getYawAngle());
        m_gyroRoll.setDouble(getRollAngle());
        m_xAcceleration.setDouble(getXAcceleration());
        m_yAcceleration.setDouble(getYAcceleration());
    }

}