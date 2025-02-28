package frc.robot.commandgroups;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
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
public class OneCoralAutoLeft extends SequentialCommandGroup {

    // constructor
    public OneCoralAutoLeft() {

        addCommands (

        // depending on start position, set odometry
        new InstantCommand(()-> {
            int startposn = RobotContainer.mainShufflePage.getStartPositionIndex();

            Pose2d startpose;
            switch (startposn) {
                case 0:
                    startpose = new Pose2d(7.55, 0.4, new Rotation2d(0.0));
                    break;
                case 1:
                    startpose = new Pose2d(7.55, 2.0, new Rotation2d(0.0));
                    break;
                case 2:
                    startpose = new Pose2d(7.55, 4.02, new Rotation2d(0.0));
                    break;
                case 3:
                    startpose = new Pose2d(7.55, 6.02, new Rotation2d(0.0));
                    break;
                case 4:
                    startpose = new Pose2d(7.55, 7.65, new Rotation2d(0.0));
                    break;
                default:
                    startpose = new Pose2d(7.55, 4.02, new Rotation2d(0.0));
            };

            // convert for red vs blue
            startpose = AutoFunctions.redVsBlue(startpose);
            
            // set odometry
            RobotContainer.odometry.setPose(startpose);
        } ),

        // move to in front of reef target  (tag22 for blue, tag 9 for red)
        new MoveToPose(0.5, 1.5,
                new Pose2d(4.76,5.65, new Rotation2d(Math.toRadians(-120.0)))),

        
        new ApproachReef(false), 

        new Pause(2),

        new InstantCommand(()->RobotContainer.elevator.Level4()),

        new CoralOutake()
        
    
        // To Do
        // KN: drop reef approach and stop stuff here
    



        
        // //fill in position robot needs to go to based on the auto layout
        // new MoveToPose(1, 
        //                1,
        //                new Pose2d (5.5,5.5, new Rotation2d(Math.toRadians(-119)))),

        // // run at same time as drive and fill in position for Level four placement 
        // new InstantCommand(()->RobotContainer.elevator.Level4()),

        // // coral grabber not completed yet but will work like elevator
        // new InstantCommand(()->RobotContainer.intake.OutakeRun(1)), 

        // // Drives to the humen station at end to be ready for teleop (fill in pos please)
        // new MoveToPose(1, 
        //                0.5,
        //                new Pose2d (1.5,6.8, new Rotation2d(Math.toRadians(-51)))),
        
        // // at the same time as driving lower slides back to zero 
        // new InstantCommand(()->RobotContainer.elevator.returnToIntake())

        );
    }

}

