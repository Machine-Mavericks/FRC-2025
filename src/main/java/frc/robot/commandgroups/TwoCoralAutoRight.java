package frc.robot.commandgroups;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.Odometry;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.commands.ApproachReef;
import frc.robot.commands.CoralIntake;
import frc.robot.commands.CoralOutake;
import frc.robot.commands.MoveToPose;
import frc.robot.commands.Pause;
import frc.robot.utils.AutoFunctions;

// Example Sequential Command Group
// There are also:
// ParallelCommandGroup
// ParallelRaceGroup
// ParallelDeadlineGroup
public class TwoCoralAutoRight extends SequentialCommandGroup {

    // constructor
    public TwoCoralAutoRight() {

        addCommands (
            // depending on start position, set odometry
            new InstantCommand(()-> {
                int startposn = RobotContainer.mainShufflePage.getStartPositionIndex();
    
                Pose2d startpose;
                switch (startposn) {
                    case 0:
                        startpose = new Pose2d(7.55, 0.4, new Rotation2d((Math.toRadians(180.0))));
                        break;
                    case 1:
                        startpose = new Pose2d(7.55, 2.0, new Rotation2d((Math.toRadians(180.0))));
                        break;
                    case 2:
                        startpose = new Pose2d(7.55, 4.02, new Rotation2d((Math.toRadians(180.0))));
                        break;
                    case 3:
                        startpose = new Pose2d(7.55, 6.02, new Rotation2d((Math.toRadians(180.0))));
                        break;
                    case 4:
                        startpose = new Pose2d(7.55, 7.65, new Rotation2d((Math.toRadians(180.0))));
                        break;
                    default:
                        startpose = new Pose2d(7.55, 4.02, new Rotation2d((Math.toRadians(180.0))));
                };
    
                // convert for red vs blue
                startpose = AutoFunctions.redVsBlue(startpose);
                
                // set odometry
                RobotContainer.odometry.setPose(startpose);
            } ),
    
            // move to in front of reef target  (tag22 for blue, tag 9 for red)
            // move to veiw pose 
            //new InstantCommand(()->RobotContainer.snapToReef = false),
           new InstantCommand(()-> RobotContainer.odometry.TagEnable = false),


            new MoveToPose(
                1.0,
                0.5,
                new Pose2d(3.7,2.3, new Rotation2d(Math.toRadians(60.0)))
            ),
    
            // approach reef 
            //new ApproachReef(true),
    
            new Pause(1.0),
    
            // raise to level 4 height
            new InstantCommand(()->RobotContainer.elevator.Level4()),
    
            new Pause(1.0),
    
            // deposite 
            new InstantCommand(()->RobotContainer.intake.intakeRun(-0.7)),
    
            new Pause(1.0),
    
            new InstantCommand(()->RobotContainer.intake.intakeRun(0)),
    
            new Pause(1.0),
    
            //lower elevator for travel 
            new InstantCommand(()->RobotContainer.elevator.Level0()),
    
            new Pause(1.0),
    
            // move to pickup 
            new MoveToPose(
                1, 
                0.5,
                new Pose2d (1.3,1.3, new Rotation2d(Math.toRadians(-15)))
            ),
    
            new Pause(1.0),
    
    
            // intake new peice 
            new InstantCommand(()->RobotContainer.intake.intakeRun(0.7)),
    
            new Pause(2),
    
            new InstantCommand(()->RobotContainer.intake.intakeRun(0)),
    
            new Pause(1),
    
            // move to veiw point number two
            new MoveToPose(
                0.5, 
                1.5,
                new Pose2d(3.7,2.3, new Rotation2d(Math.toRadians(60)))
            ),
    
            // apprach reef 
           // new ApproachReef(false), 
    
            new Pause(1),
    
            // rais elevator to level 4 
            new InstantCommand(()->RobotContainer.elevator.Level4()),
    
            new Pause(1),
    
            // deposite 
            new InstantCommand(()->RobotContainer.intake.intakeRun(-0.7)),
    
            new Pause(1),
    
            new InstantCommand(()->RobotContainer.intake.intakeRun(0)),
    
            new Pause(1),
    
            // lower elevator 
            new InstantCommand(()->RobotContainer.elevator.Level0()),
    
            new Pause(1.0)
    
            // drive back to humen station 
    
            //new MoveToPose(
            //     0.5, 
            //     1.5,
            //     new Pose2d(1.06,7.3, new Rotation2d(Math.toRadians(-56)))
            // ),
    
            // intake peice for tele
            // new InstantCommand(()->RobotContainer.intake.intakeRun(0.7)),
    
            // new Pause(1),
    
            // new InstantCommand(()->RobotContainer.intake.intakeRun(0)),
    
            );
        }
    
    }
    
    