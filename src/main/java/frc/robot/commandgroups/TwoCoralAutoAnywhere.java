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
public class TwoCoralAutoAnywhere extends SequentialCommandGroup {

    // constructor
    public TwoCoralAutoAnywhere() {

        addCommands (

        //fill in position robot needs to go to based on the auto layout
        new MoveToPose(1, 0.5, null),// add redVsBlue x, y, robot angle

        // run at same time as drive and fill in position for Level four placement 
        new InstantCommand(()->RobotContainer.elevator.moveToPosition(1)),

        // coral grabber not completed yet but will work like elevator
        //new InstantCommand(()->RobotContainer.coralGrabber.moveToPosition(1)), 

        // Drives to the humen station to pick up (fill in pos please)
        new MoveToPose(1, 0.5, null),// add redVsBlue x, y, robot angle
        
        // at the same time as driving lower slides to a position for intake 
        new InstantCommand(()->RobotContainer.elevator.moveToPosition(0)),

        // run intake 
        //new InstantCommand(()->RobotContainer.coralGrabber.moveToPosition(1)), 

        //fill in position robot needs to go to based on the auto layout (second place pose)
        new MoveToPose(1, 0.5, null),// add redVsBlue x, y, robot angle

        // at the same time as driving raise slides to a position for high placment
        new InstantCommand(()->RobotContainer.elevator.moveToPosition(1)),
        
        // run outake 
        //new InstantCommand(()->RobotContainer.coralGrabber.moveToPosition(1)),

        //fill in position robot needs to go to based on the auto layout 
        //(park in humen station at end of auto so can start cycleing in tele)
        new MoveToPose(1, 0.5, null),// add redVsBlue x, y, robot angle

        // at the same time as driving lower slides to zero 
        new InstantCommand(()->RobotContainer.elevator.moveToPosition(0))

        // end of auto



        );
    }

}

