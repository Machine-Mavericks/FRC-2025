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
public class ThreeCoralAutoAnywhere extends SequentialCommandGroup {

    // constructor
    public ThreeCoralAutoAnywhere() {

        addCommands (

        //fill in position robot needs to go to based on the auto layout
        new MoveToPose(1, 
                       1,
                       new Pose2d (5.5,5.5, new Rotation2d(Math.toRadians(-119)))),// add redVsBlue x, y, robot angle

        // run at same time as drive and fill in position for Level four placement 
        new InstantCommand(()->RobotContainer.elevator.Level4()),

        // coral grabber not completed yet but will work like elevator
        new InstantCommand(()->RobotContainer.intake.OutakeRun(1)),

        // Drives to the humen station to pick up (fill in pos please)
        new MoveToPose(1, 
                       0.5,
                      new Pose2d (1.5,6.8, new Rotation2d(Math.toRadians(-51)))),
        
        
        // at the same time as driving lower slides to a position for intake 
        new InstantCommand(()->RobotContainer.elevator.returnToIntake()),

        // run intake 
        new InstantCommand(()->RobotContainer.intake.intakeRun(1)),

        //fill in position robot needs to go to based on the auto layout (second place pose)
        new MoveToPose(1, 
        1,
        new Pose2d (3.4,5.6, new Rotation2d(Math.toRadians(-60)))),

        // at the same time as driving raise slides to a position for high placment
        new InstantCommand(()->RobotContainer.elevator.Level4()),
        
        // run outake 
        new InstantCommand(()->RobotContainer.intake.OutakeRun(1)),
        //fill in position robot needs to go to based on the auto layout 
        //(park in humen station at end of auto so can start cycleing in tele)
        new MoveToPose(1, 
        1,
        new Pose2d (1.5,6.8, new Rotation2d(Math.toRadians(-51)))),

        // at the same time as driving lower slides to zero 
        new InstantCommand(()->RobotContainer.elevator.returnToIntake()),

        // run intake 
        new InstantCommand(()->RobotContainer.intake.intakeRun(1)),

        new MoveToPose(1, 
        1,
        new Pose2d (3.4,5.6, new Rotation2d(Math.toRadians(-60)))),

        // at the same time as driving raise slides to a position for high placment
        new InstantCommand(()->RobotContainer.elevator.Level3()),
        
        // run outake 
        new InstantCommand(()->RobotContainer.intake.OutakeRun(1)),
        // coral station
        new MoveToPose(1, 
        1,
        new Pose2d (1.5,6.8, new Rotation2d(Math.toRadians(-51))))) ;
    }

}

