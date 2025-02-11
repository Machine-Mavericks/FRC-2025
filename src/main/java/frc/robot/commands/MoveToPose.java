package frc.robot.commands;

import java.util.ArrayList;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;


// command template
public class MoveToPose extends Command {
    
    // the follow-path command that this command will use
    FollowPath cmd;

    // local copy of input parameters
    double maxSpeed, maxAccel;
    Pose2d dest;
    
    // constructor
    public MoveToPose(double maxSpeed,
                      double maxAccel, 
                      Pose2d destination) {
        
        // record input parameters
        this.maxSpeed= maxSpeed;
        this.maxAccel = maxAccel;
        this.dest = destination;

        // this command requires robot drive subsystem
        addRequirements(RobotContainer.drivesystem);
    }

    // This method is called once when command is started
    @Override
    public void initialize() {

        // current robot pose
        Pose2d currentPose = RobotContainer.odometry.getPose2d();

        // straight-line travel heading
        Rotation2d heading = new Rotation2d(dest.getX()-currentPose.getX(),
                dest.getY()- currentPose.getY());

        // construct follow-path command
        cmd = new FollowPath(maxSpeed,
                maxAccel,
                0.0,
                0.0,
                heading,
                new ArrayList<Translation2d>() {{ }},
                new Pose2d(dest.getX(), dest.getY(), heading),
                dest.getRotation()
        );

        // initialize the follow-path command
        cmd.initialize();
    }

    // This method is called periodically while command is active
    @Override
    public void execute() { 
        cmd.execute();
    }

    // This method to return true only when command is to finish. Otherwise return false
    @Override
    public boolean isFinished() {
        return cmd.isFinished();
    }

    // This method is called once when command is finished.
    @Override
    public void end(boolean interrupted) {
        cmd.end(interrupted);
    }

}