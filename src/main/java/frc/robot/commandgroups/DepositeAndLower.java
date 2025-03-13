package frc.robot.commandgroups;

import java.util.ArrayList;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.commands.FollowPath;
import frc.robot.commands.GoToPose;
import frc.robot.commands.MoveToPose;
import frc.robot.commands.Pause;
import frc.robot.utils.AutoFunctions;

// Example Sequential Command Group
// There are also:
// ParallelCommandGroup
// ParallelRaceGroup
// ParallelDeadlineGroup
public class DepositeAndLower extends SequentialCommandGroup {

    // constructor
    public DepositeAndLower() {

        addCommands (
        // Turn off tag detection
        //new InstantCommand(()->RobotContainer.odometry.TagEnable=false)
            new Pause(1.0), 

            new InstantCommand(()->RobotContainer.intake.intakeRun(-0.7)),
    
            new Pause(1),
    
            new InstantCommand(()->RobotContainer.intake.intakeRun(0)),
    
            new Pause(1),

             // lower elevator 
            new InstantCommand(()->RobotContainer.elevator.Level0()),
    
            new Pause(1.0)
        

       
        );
    }

}


