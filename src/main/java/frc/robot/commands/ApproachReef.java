package frc.robot.commands;

import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.utils.Utils;


// command template
public class ApproachReef extends Command {

    boolean chooseLeftSide;

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
        yControl = new PIDController(0.03, 0.005, 0.0);
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

        // is tag detected?
        TargetDetected = RobotContainer.camera.isTargetPresent();

        // get apriltag id that we are trying to approach
        DestinationTagID=(int)RobotContainer.camera.getPrimAprilTagID();

        if (chooseLeftSide == true)
            RobotContainer.camera.setPipeline(0);
        else 
            RobotContainer.camera.setPipeline(1);
        
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
        double horizError = RobotContainer.camera.getHorizontalTargetOffsetAngle();

        //TargetDetected = RobotContainer.camera.isTargetPresent();

        // determine forward speed (m/s)
        x_speed = 0.3;

        // sideways control
        if (chooseLeftSide)
            y_speed = yControl.calculate(horizError+LeftOffset);
        else
            y_speed = yControl.calculate(horizError+RightOffset);
        
        if (y_speed > 1.0)
            y_speed = 1.0;
        if (y_speed < -1.0)
            y_speed = -1.0;


        // rotational control
        double currentAngle = RobotContainer.odometry.getPose2d().getRotation().getDegrees();        
        omega_speed = omegaControl.calculate(Utils.AngleDifference(targetAngle,currentAngle));

        if (omega_speed > 0.42)
                omega_speed = 0.42;
        if (omega_speed < -0.42)
                omega_speed = -0.42;
        if (!RobotContainer.camera.isTargetPresent()){
            x_speed = 0.0;
            y_speed = 0.0;

        }

        // drive robot
        RobotContainer.drivesystem.RobotDrive(x_speed, y_speed, omega_speed, false);




    }

    // This method to return true only when command is to finish. Otherwise return false
    @Override
    public boolean isFinished() {

        return !TargetDetected || (chooseLeftSide && RobotContainer.camera.getTargetArea() > 4.2) ||
         (!chooseLeftSide && RobotContainer.camera.getTargetArea() > 10.1);

    }

    // This method is called once when command is finished.
    @Override
    public void end(boolean interrupted) {
        RobotContainer.drivesystem.RobotDrive(0, 0, 0, false);

    }

}