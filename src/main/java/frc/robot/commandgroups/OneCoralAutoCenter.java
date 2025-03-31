package frc.robot.commandgroups;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.InstantCommand;
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
public class OneCoralAutoCenter extends SequentialCommandGroup {

    // constructor
    public OneCoralAutoCenter() {

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
                    startpose = new Pose2d(7.55, 2.0, new Rotation2d(Math.toRadians(180.0)));
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

        

        // move to in front of reef target  (tag21 for blue, tag 10 for red)
        //new InstantCommand(()->RobotContainer.snapToReef = false),
        new InstantCommand(()-> RobotContainer.odometry.EnableApriltagProcessing(false)),
        new InstantCommand(()-> RobotContainer.algaeRemover.AlgaeBloom()),
        // moving to veiw point 
        new MoveToPose( // check point then speed up 
            2.0,
            1.5,
            new Pose2d(6.4, 3.9, new Rotation2d(Math.toRadians(180.0)))
        ),

        new InstantCommand(()-> RobotContainer.odometry.EnableApriltagProcessing(true)),

        // filtering tag
        new InstantCommand(()-> {
            if  (DriverStation.getAlliance().get()==Alliance.Blue)
                RobotContainer.camr.SetPriorityTagID(21);
            else 
                RobotContainer.camr.SetPriorityTagID(10);
            }),

        // approch right 
        new ApproachReef(false), 

        new InstantCommand(()-> RobotContainer.odometry.EnableApriltagProcessing(false)),

        new InstantCommand(()->RobotContainer.elevator.Level4()),

        new Pause(1.25),

        new InstantCommand(()->RobotContainer.intake.intakeRun(-0.7)),

        new Pause(0.75),

        new InstantCommand(()->RobotContainer.intake.intakeRun(0)),

        new InstantCommand(()->RobotContainer.elevator.Level0()),

        new Pause(1.25)

        );
    }

}

