package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Limelight;
import frc.robot.utils.Utils;


// command template
public class ApproachReef extends Command {

    boolean chooseLeftSide;
    
    // our chosen camera to use
    Limelight selectedCamera;

    // controller to steer towards reef - y used for 'sideways control'
    PIDController yControl;  
    PIDController omegaControl;

    // primary tag ID
    int DestinationTagID;

    // target robot angle
    double targetAngle;

    // is target detected
    boolean TargetDetected;

    final double LeftOffset = 0.0; 

    final double RightOffset = 0.0;

    // constructor
    public ApproachReef(boolean side) {

        // add subsystem requirements (if any) - for example:
        addRequirements(RobotContainer.drivesystem);

        // set up sideways and rotational controllers
        yControl = new PIDController(0.11, 0.005, 0.0);
        omegaControl = new PIDController(0.09, 0.001, 0.0000);

        // by default - choose left side
        chooseLeftSide = side;
   
    }

    // This method is called once when command is started
    @Override
    public void initialize() {

        // initialize controllers
        yControl.reset();
        omegaControl.reset();

        if (chooseLeftSide == true)
            selectedCamera = RobotContainer.camleft;
        else 
            selectedCamera = RobotContainer.camr;

        // is tag detected?
        TargetDetected = selectedCamera.isTargetPresent();

        // get apriltag id that we are trying to approach
        DestinationTagID=(int)selectedCamera.getPrimAprilTagID();

        
        
        // determine desired robot angle
        targetAngle = 0.0;
        switch (DestinationTagID)
        {
            case 6:
                targetAngle = 120.0;
                break;
            case 7:
                targetAngle = 180.0;
                break;
            case 8:
                targetAngle = -120.0;
                break;
            case 9:
                targetAngle = -60.0;
                break;
            case 10:
                targetAngle = 0.0;
                break;
            case 11:
                targetAngle = 60.0;
                break;
            case 17:
                targetAngle = 60.0;
                break;
            case 18:
                targetAngle = 0.0;
                break;
            case 19:
                targetAngle = -60.0;
                break;
            case 20:
                targetAngle = -120.0;
                break;
            case 21:
                targetAngle = 180.0;
                break;
            case 22:
                targetAngle = 120.0;
                break;
        };
        
        
        


    }

    // This method is called periodically while command is active
    @Override
    public void execute() {

        // speed to move forward at
        double x_speed;
        double y_speed;
        double omega_speed;

        // double horizontal error to target
        double horizError = selectedCamera.getHorizontalTargetOffsetAngle();

        //TargetDetected = RobotContainer.camera.isTargetPresent();

        // determine forward speed (m/s)
        x_speed = 1.5;

        // sideways control
        if (chooseLeftSide)
            y_speed = yControl.calculate(horizError+LeftOffset);
        else
            y_speed = yControl.calculate(horizError+RightOffset);
        
        if (y_speed > 3.0)
            y_speed = 3.0;
        if (y_speed < -3.0)
            y_speed = -3.0;


        // rotational control
        double currentAngle = RobotContainer.odometry.getPose2d().getRotation().getDegrees();        
        omega_speed = omegaControl.calculate(Utils.AngleDifference(targetAngle,currentAngle));

        if (omega_speed > 0.42)
                omega_speed = 0.42;
        if (omega_speed < -0.42)
                omega_speed = -0.42;
        if (!selectedCamera.isTargetPresent()){
            x_speed = 0.0;
            y_speed = 0.0;

        }

        // drive robot
        RobotContainer.drivesystem.RobotDrive(x_speed, y_speed, omega_speed, false);

       




    }

    // This method to return true only when command is to finish. Otherwise return false
    @Override
    public boolean isFinished() {

        return !TargetDetected || (chooseLeftSide && selectedCamera.getTargetArea() > 48.0) ||
         (!chooseLeftSide && selectedCamera.getTargetArea() > 7.0);

    }

    // This method is called once when command is finished.
    @Override
    public void end(boolean interrupted) {
        RobotContainer.drivesystem.RobotDrive(0, 0, 0, false);

    }

}