package frc.robot.commandgroups;

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
        new MoveToPose(1, 0.5, null),

        // run at same time as drive and fill in position for Level four placement 
        new InstantCommand(()->RobotContainer.elevator.moveToPosition(1)),

        // coral grabber not completed yet but will work like elevator
        //new InstantCommand(()->RobotContainer.coralGrabber.moveToPosition(1)), 

        // Drives to the humen station at end to be ready for teleop (fill in pos please)
        new MoveToPose(1, 0.5, null),
        
        // at the same time as driving lower slides back to zero 
        new InstantCommand(()->RobotContainer.elevator.moveToPosition(0))

        );
    }

}

