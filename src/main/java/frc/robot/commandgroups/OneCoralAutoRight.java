package frc.robot.commandgroups;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.commands.ApproachReef;
import frc.robot.commands.CoralOutake;
import frc.robot.commands.MoveToPose;
import frc.robot.commands.Pause;
import frc.robot.utils.AutoFunctions;

// Example Sequential Command Group
// There are also:
// ParallelCommandGroup
// ParallelRaceGroup
// ParallelDeadlineGroup
public class OneCoralAutoRight extends SequentialCommandGroup {

    // constructor
    public OneCoralAutoRight() {

        addCommands (

        // depending on start position, set odometry
        new InstantCommand(()-> {
            int startposn = RobotContainer.mainShufflePage.getStartPositionIndex();

            Pose2d startpose;
            switch (startposn) {
                case 0:
                    startpose = new Pose2d(7.55, 0.4, new Rotation2d(Math.toRadians(180.0)));
                    break;
                case 1:
                    startpose = new Pose2d(7.1, 2.0, new Rotation2d(Math.toRadians(180.0)));
                    break;
                case 2:
                    startpose = new Pose2d(7.55, 4.02, new Rotation2d(Math.toRadians(180.0)));
                    break;
                case 3:
                    startpose = new Pose2d(7.55, 6.02, new Rotation2d(Math.toRadians(180.0)));
                    break;
                case 4:
                    startpose = new Pose2d(7.55, 7.65, new Rotation2d(Math.toRadians(180.0)));
                    break;
                default:
                    startpose = new Pose2d(7.55, 4.02, new Rotation2d(Math.toRadians(180.0)));
            };

            // convert for red vs blue
            startpose = AutoFunctions.redVsBlue(startpose);
            
            // set odometry
            RobotContainer.odometry.setPose(startpose);
        } ),

        new InstantCommand(()-> RobotContainer.odometry.EnableApriltagProcessing(false)),
       // new InstantCommand(()-> RobotContainer.algaeRemover.ResetTilt()),
        
        new MoveToPose( //change pos then up speed 
            2.0, 
            1.5,
            new Pose2d(7.3,2.0, new Rotation2d(Math.toRadians(180.0)))
        ),

        // move to in front of reef target  (tag22 for blue, tag 9 for red)
        new MoveToPose( //change pos then up speed 
            2.0, 
            1.5,
            new Pose2d(5.3,2.3, new Rotation2d(Math.toRadians(120.0)))
        ),

        new InstantCommand(()-> RobotContainer.odometry.EnableApriltagProcessing(true)),
        
        new InstantCommand(()-> {
            if  (DriverStation.getAlliance().get()==Alliance.Blue)
                RobotContainer.camr.SetPriorityTagID(22);
            else 
                RobotContainer.camr.SetPriorityTagID(9);
            }),

        new ApproachReef(false), 

        new InstantCommand(()-> RobotContainer.odometry.EnableApriltagProcessing(false)),

        new InstantCommand(()->RobotContainer.elevator.Level4()),

        new Pause(1.25),

        new InstantCommand(()->RobotContainer.intake.intakeRun(-1)),

        new Pause(0.75),

        new InstantCommand(()->RobotContainer.intake.intakeRun(0)),

        new InstantCommand(()->RobotContainer.elevator.Level0()),

        new Pause(1.25)


        );
    }

}

