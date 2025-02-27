package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;


// command template
public class RobotDriveFixedTime extends Command {

    Timer time;

    double duration;
    double xspeed;
    double yspeed;

    // constructor
    public RobotDriveFixedTime(double xspeed, double yspeed, double duration_s) {

        // add subsystem requirements (if any) - for example:
        addRequirements(RobotContainer.drivesystem);

        // create timer
        time = new Timer();

        duration = duration_s;
        this.xspeed = xspeed;
        this.yspeed = yspeed;
    }

    // This method is called once when command is started
    @Override
    public void initialize() {

        time.reset();
        time.start();

    }

    // This method is called periodically while command is active
    @Override
    public void execute() {
        RobotContainer.drivesystem.RobotDrive(xspeed, yspeed, 0.0, false);
    }

    // This method to return true only when command is to finish. Otherwise return false
    @Override
    public boolean isFinished() {

        return time.hasElapsed(duration);

    }

    // This method is called once when command is finished.
    @Override
    public void end(boolean interrupted) {
        // disable drive
        RobotContainer.drivesystem.RobotDrive(0.0, 0.0, 0.0, false);
    }

}