package frc.robot.commands;

import java.util.ArrayList;
import java.util.List;

import frc.robot.utils.AutoFunctions;
import frc.robot.utils.Utils;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.Trajectory.State;
import edu.wpi.first.math.trajectory.constraint.SwerveDriveKinematicsConstraint;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;


public class FollowPath extends Command {

    // Trajectory Configuration
    TrajectoryConfig config;

    // Trajectory
    Trajectory trajectory;

    // trajectory points
    Rotation2d pathStartAngle_unmirrored;
    Pose2d pathEndPose_unmirrored;
    List<Translation2d> pathWaypoints_unmirrored;
    Rotation2d robotEndAngle_unmirrored;
    Rotation2d pathStartAngle;
    Pose2d pathEndPose;
    List<Translation2d> pathWaypoints;
    Rotation2d robotEndAngle;

    // ending speed
    double endSpeed;

    // x,y,theta PID controllers
    PIDController xController, yController, thetaController;

    // time in path
    Timer pathTime;

    // robot rotation control
    double endRobotAngle;       // target robot angle in rad
    double startRobotAngle;     // initial robot angle in rad
    double rotationRate;        // desired rotation rate in rad/s

    // state variables for manually-implemented robot x & y position PID controllers
    double x_ierror;
    double y_ierror;

    double MaxPathSpeed;

    // true if to translate destinaton using redvsblue
    boolean redVsBlueEnable;
    
    /** A command to follow a path based on input parameters
     *
     *  @param maxSpeed - max robot travel speed (m/s)
     *  @param maxAccel - max robot travel accel (m/s2)
     *  @param startSpeed - starting robot speed of path (m/s) - normally 0m/s unless continuing from previous path
     *  @param endSpeed - ending robot speed of path (m/s) - normally 0m/s unless continuing into subsequent path
     *  @param pathStartAngle - starting angle of generated path (Rotation2d)
     *  @param pathWaypoints - list of waypoints (x,y) (List<Translation2d>)
     *  @param pathEndPose - ending position of path (x,y and angle combined) (Pose2d)
     *  @param robotEndAngle - angle robot faces at end of path (Rotation2d)
     * */
    
     public FollowPath(double maxSpeed,
                      double maxAccel,
                      double startSpeed,
                      double endSpeed,
                      Rotation2d pathStartAngle,
                      List<Translation2d> pathWaypoints,
                      Pose2d pathEndPose,
                      Rotation2d robotEndAngle) 
    { this(maxSpeed, maxAccel, startSpeed, endSpeed, pathStartAngle, pathWaypoints, pathEndPose, robotEndAngle, true); }
    
    
     public FollowPath(double maxSpeed,
                      double maxAccel,
                      double startSpeed,
                      double endSpeed,
                      Rotation2d pathStartAngle,
                      List<Translation2d> pathWaypoints,
                      Pose2d pathEndPose,
                      Rotation2d robotEndAngle,
                      boolean redVsBlueEnable
                      ) {

        // add subsystem requirements (if any) - for example:
        addRequirements(RobotContainer.drivesystem);

        // create trajectory configuration
        config = new TrajectoryConfig(maxSpeed, maxAccel);

        // set constraints of trajectory
        config.setReversed(false);
        config.setStartVelocity(startSpeed);
        config.setEndVelocity(endSpeed);
        config.setKinematics(RobotContainer.drivesystem.getKinematics());
        config.addConstraint(new SwerveDriveKinematicsConstraint(RobotContainer.drivesystem.getKinematics(),
                RobotContainer.drivesystem.MAX_SPEED));

                
        // save positions for use during command initialization
        this.pathStartAngle_unmirrored = pathStartAngle;
        this.pathEndPose_unmirrored = pathEndPose;
        this.pathWaypoints_unmirrored = pathWaypoints;
        this.robotEndAngle_unmirrored = robotEndAngle;

        // save ending speed of robot
        this.endSpeed = endSpeed;

        // save redvsblue translation enable
        this.redVsBlueEnable = redVsBlueEnable;

        // configure PID controllers - set PID gains
        xController = new PIDController(0.5, 5.00, 0.0);
        yController = new PIDController(0.5, 5.00, 0.0);
        thetaController = new PIDController(3.0, 0.05, 0.0);

        // configure PID controllers integration limiters - prevents excessive windup of integrated error
        xController.setIntegratorRange(-20.0, 20.0);
        yController.setIntegratorRange(-20.0, 20.0);
        thetaController.setIntegratorRange(-5.0, 5.0);

        // set up timer
        pathTime = new Timer();
    }

    // This method is called once when command is started
    @Override
    public void initialize() {

        // get current robot position from odometry
        Pose2d currentPos = RobotContainer.odometry.getPose2d();

        // if required, mirror the path points per redvsblue
        if (redVsBlueEnable)
        {
            pathStartAngle = AutoFunctions.redVsBlue(pathStartAngle_unmirrored);
            pathEndPose = AutoFunctions.redVsBlue(pathEndPose_unmirrored);
            robotEndAngle = AutoFunctions.redVsBlue(robotEndAngle_unmirrored);
            
            pathWaypoints = new ArrayList<>();
            for (int i=0;i<pathWaypoints_unmirrored.size();++i)
            {
                pathWaypoints.add(AutoFunctions.redVsBlue(pathWaypoints_unmirrored.get(i)));
            }
        }
        else
        {
            pathStartAngle = pathStartAngle_unmirrored;
            pathEndPose = pathEndPose_unmirrored;
            robotEndAngle = robotEndAngle_unmirrored;
            pathWaypoints = pathWaypoints_unmirrored;
        }
        
        
        // start path at current location of robot - use x,y only!
        // set start angle of path to provided parameter
        Pose2d pathStartPose = new Pose2d(currentPos.getTranslation(),
                pathStartAngle);

        // create trajectory
        // use try-catch block in case trajectory cannot be formed using given points
        // if trajectory cannot formed then set to null. Command will detect and exit
        try {
            trajectory = TrajectoryGenerator.generateTrajectory(pathStartPose,
                    pathWaypoints,
                    pathEndPose,
                    config);
        }
        catch (Exception e) {
            trajectory = null;
        }

        // set trajectory in odometry for display on dashboard
        RobotContainer.mainShufflePage.setFieldTrajectory(trajectory);

        // reset PID controllers - to be ready to run path
        xController.reset();
        yController.reset();
        thetaController.reset();

        // reset manually-implemented x and y PID controller state values
        x_ierror=0.0;
        y_ierror = 0.0;

        // reset path timer
        pathTime.reset();
        pathTime.start();

        // set initial robot rotation target (in rads)
        startRobotAngle = currentPos.getRotation().getRadians();

        // determine rotation rate along path (in rads/s)
        // turn robot by smallest angle (either in +ve or -ve rotation)
        if (trajectory!=null&& trajectory.getTotalTimeSeconds() !=0.0)
            rotationRate = Math.toRadians(Utils.AngleDifference(Math.toDegrees(startRobotAngle),
                robotEndAngle.getDegrees())) / trajectory.getTotalTimeSeconds();
        else
            rotationRate = 0.0;
    }



    // This method is called periodically while command is active
    @Override
    public void execute() {

        // if we have valid path then execute. Otherwise do nothing.
        if (trajectory!=null) {

            // get next reference state from trajectory path
            State targetPathState = trajectory.sample(pathTime.get());

            // get current robot position from odometry
            Pose2d currentPos = RobotContainer.odometry.getPose2d();

            // determine target robot angle
            double targetRobotAngle;
            if (pathTime.get() < trajectory.getTotalTimeSeconds())
                targetRobotAngle = startRobotAngle + (rotationRate * pathTime.get());
            else
                targetRobotAngle = startRobotAngle + (rotationRate * trajectory.getTotalTimeSeconds());

            // commented out for now.  Will try using manual implementation of PID
            // execute PID controllers - error = reference - actual
            // each controller outputs resulting speeds to be given to drive subsystem
            //double dX = -xController.calculate(targetPathState.poseMeters.getX() -
            //        currentPos.getX());

            // manual implementation of x-position PI controller
            double error = targetPathState.poseMeters.getX() - currentPos.getX();
            x_ierror += 0.1 * error;
            if (Math.abs(error) > 0.10)
                x_ierror = 0.0;
            double dX = 4.0 * error + x_ierror;

            // commented out for now.  Will try using manual implementation of PID
            // double dY = -yController.calculate(targetPathState.poseMeters.getY() -
            //                currentPos.getY());

            // manual implementation of y-position PI controller
            double y_error = targetPathState.poseMeters.getY() - currentPos.getY();
            y_ierror += 0.1 * y_error;
            if (Math.abs(y_error) > 0.10)
                y_ierror = 0.0;
            double dY = 4.0 * y_error + y_ierror;

            // limits speeds
            //if (dX > MaxPathSpeed) dX=MaxPathSpeed;
            //if (dX < -MaxPathSpeed) dX=-MaxPathSpeed;
            //if (dY > MaxPathSpeed) dY=MaxPathSpeed;
            //if (dY <-MaxPathSpeed) dY=-MaxPathSpeed;



            // robot rotation controller
            double omega = -thetaController.calculate(Utils.AngleDifferenceRads(currentPos.getRotation().getRadians(), targetRobotAngle));

            // go ahead and drive robot
            RobotContainer.drivesystem.FieldDrive(dX, dY, omega, isScheduled());

        }   // end if we have valid trajectory to follow
    }

    // This method to return true only when command is to finish. Otherwise return false
    @Override
    public boolean isFinished() {

        // command is finished if no valid path or when time in command exceeds expected time of path
        return ( trajectory==null ||
                (pathTime.get() > (trajectory.getTotalTimeSeconds()+2.0)) );

    }

    // This method is called once when command is finished.
    @Override
    public void end(boolean interrupted) {

        // this command is done. Remove trajectory in odometry from display on dashboard
        RobotContainer.mainShufflePage.deleteFieldTrajectory();

        // if end speed is 0m/s, then stop drive to ensure robot drive is not left 'creeping'
        // use 0.01 in case of any floating point rounding error that would make
        // a endSpeed==0.0 condition fail.
        if (endSpeed<=0.01)
            RobotContainer.drivesystem.FieldDrive(0.0, 0.0, 0.0,false);

    }

}