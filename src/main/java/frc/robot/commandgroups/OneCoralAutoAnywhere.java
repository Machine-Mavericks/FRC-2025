package frc.robot.commandgroups;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.commands.MoveToPose;

// Example Sequential Command Group
// There are also:
// ParallelCommandGroup
// ParallelRaceGroup
// ParallelDeadlineGroup
public class OneCoralAutoAnywhere extends SequentialCommandGroup {

    // constructor
    public OneCoralAutoAnywhere() {

        addCommands (

        //fill in position robot needs to go to based on the auto layout
        new MoveToPose(1, 
                       1,
                       new Pose2d (5.5,5.5, new Rotation2d(Math.toRadians(-119)))),

        // run at same time as drive and fill in position for Level four placement 
        new InstantCommand(()->RobotContainer.elevator.Level4()),

        // coral grabber not completed yet but will work like elevator
        new InstantCommand(()->RobotContainer.intake.OutakeRun(1)), 

        // Drives to the humen station at end to be ready for teleop (fill in pos please)
        new MoveToPose(1, 
                       0.5,
                       new Pose2d (1.5,6.8, new Rotation2d(Math.toRadians(-51)))),
        
        // at the same time as driving lower slides back to zero 
        new InstantCommand(()->RobotContainer.elevator.returnToIntake())

        );
    }

}

