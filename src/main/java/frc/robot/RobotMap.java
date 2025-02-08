package frc.robot;


/**
 * Mapping and creation of hardware on the robot
 */
public class RobotMap {
    
    /**
     * Inner class to hold CAN ID constants.
     */
    public static class CANID {
        
        // CAN IDs for Swerve Cancoders
        public static final int LF_CANCODER = 10;
        public static final int RF_CANCODER = 11;
        public static final int LR_CANCODER = 12;
        public static final int RR_CANCODER  = 13;
        // CAN IDs for Steer Motors
        public static final int LF_STEER_MOTOR = 2;
        public static final int RF_STEER_MOTOR = 4;
        public static final int LR_STEER_MOTOR = 6;
        public static final int RR_STEER_MOTOR = 8;
        // CAN IDs for Drive Motors
        public static final int LF_DRIVE_MOTOR = 3;
        public static final int RF_DRIVE_MOTOR = 5;
        public static final int LR_DRIVE_MOTOR = 7;
        public static final int RR_DRIVE_MOTOR = 9;
        // CAN ID for CTR Pigeon Gyro
        public static final int PIGEON = 14;
        public static final int L_ELEVATOR = 13;
        public static final int R_ELEVATOR = 15;

        // CAN ID for ARM
        //public static final int ARM_MOTOR = 13;
        //public static final int ARM_CANCODER = 14;
        // CAN ID for Grabber
        //public static final int GRABBER_MOTOR = 15;
    }

    /**
     * Inner class to hold RoboRIO I/O connection constants
     */
  public static class RIO {
        // Grabber - Input Digital Switch (true if item is grabbed)
        //public static final int DIO_0_GRABBERSW = 0;

        // Ultrasonic sensor - used for distance to shelf pickup
        //public static final int DIO_3_ULTRASONIC_A = 3;
        //public static final int DIO_4_ULTRASONIC_B = 4;

        // Analog range sensor - used for floor cone pickup
        //public static final int AIN_0_FLOORSENSOR = 0;
        
    }
    public static class DIO {
        public static final int maxLimitSwitch = 0;
        public static final int minLimitSwitch = 1;
    }

    public static class PWMPorts {
        /** PWM Port for led strip */
        //public static final int LED_BLINKIN = 0;

        // PWM port for camera tilting subsystem
        //public static final int CAMERA_SERVO_ID = 1;
    } 

    // game controller port IDs
    public static class GamePadPorts {
        public static final int DriverID = 0;
        public static final int OperatorID = 1;
    }

}