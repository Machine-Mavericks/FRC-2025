package frc.robot.commandgroups;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.commands.MoveToPose;
import frc.robot.commands.Pause;
import frc.robot.utils.AutoFunctions;

// Example Sequential Command Group
// There are also:
// ParallelCommandGroup
// ParallelRaceGroup
// ParallelDeadlineGroup
public class TemplateCommandGroup extends SequentialCommandGroup {

    // constructor
    public TemplateCommandGroup() {

        addCommands (
        
        // Turn off tag detection
        new InstantCommand(()->RobotContainer.odometry.TagEnable=false),

        // set robot pose
        new InstantCommand(()->RobotContainer.odometry.setPose(AutoFunctions.redVsBlue(new Pose2d(7.55, 1.5, new Rotation2d(0))))),
        
        // move to spot 1.25m in front of tag #22
        new MoveToPose(2.0, 
                       2.5,
                       new Pose2d (5.98,2.68, new Rotation2d(Math.toRadians(120)))),

        
        // Turn on tag detection
        new InstantCommand(()->RobotContainer.odometry.TagEnable=true),

        new Pause(0.5),

        
    
        // move slowly to exact spot               
        new MoveToPose(0.5, 
                       1.0,
                       new Pose2d (5.40,2.75, new Rotation2d(Math.toRadians(120)))),

        
        // Turn off tag detection
        new InstantCommand(()->RobotContainer.odometry.TagEnable=false)
        
        // new command1
        // new command2
        // new command3
        );
    }

}


