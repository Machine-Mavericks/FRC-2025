package frc.robot.commandgroups;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.Odometry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
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
                        startpose = new Pose2d(7.1, 2.0, new Rotation2d((Math.toRadians(180.0))));
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
    
            new InstantCommand(()-> RobotContainer.odometry.EnableApriltagProcessing(false)),

            new MoveToPose(
                4.0,
                6.0,
                new Pose2d(3.7,2.3, new Rotation2d(Math.toRadians(60.0)))// was y 2.4
            ),
    
            // approach reef
            new InstantCommand(()-> RobotContainer.odometry.EnableApriltagProcessing(true)),

            new InstantCommand(()-> {
            if  (DriverStation.getAlliance().get()==Alliance.Blue)
                RobotContainer.camr.SetPriorityTagID(17);
            else 
                RobotContainer.camr.SetPriorityTagID(8);
            }),
            
            // raise to level 4 height
            new InstantCommand(()->RobotContainer.elevator.Level4()),

            new ApproachReef(false),
    
            new InstantCommand(()-> RobotContainer.odometry.EnableApriltagProcessing(false)),
            // deposite 
            new Pause(0.3),

            new InstantCommand(()->RobotContainer.intake.intakeRun(-1.0)),
    
            new Pause(0.25),
    
            new InstantCommand(()->RobotContainer.intake.intakeRun(0)),
    
            new Pause(0.25),

             // lower elevator 
            new InstantCommand(()->RobotContainer.elevator.Level0()),

            new ParallelCommandGroup(new CoralIntake(),
            // move to pickup 
                new MoveToPose(//waiting on point
                    4.0, 
                    6.0,
                    new Pose2d (1.145,1.059, new Rotation2d(Math.toRadians(54))))
            ),         
    
            // move to veiw point number two
            new MoveToPose(
                4.0, 
                6.0,
                new Pose2d(3.4,2.9, new Rotation2d(Math.toRadians(60)))
            ),
    
            new InstantCommand(()-> RobotContainer.odometry.EnableApriltagProcessing(true)),

            new InstantCommand(()-> {
                if  (DriverStation.getAlliance().get()==Alliance.Blue)
                    RobotContainer.camleft.SetPriorityTagID(17);
                else 
                    RobotContainer.camleft.SetPriorityTagID(8);
                }),

            // apprach reef 
            new ApproachReef(true), 
    
            new InstantCommand(()-> RobotContainer.odometry.EnableApriltagProcessing(false)),
    
            // rais elevator to level 4 
            new InstantCommand(()->RobotContainer.elevator.Level4()),
    
            // deposite 
            new DepositeAndLower()

            //new InstantCommand(()-> RobotContainer.odometry.EnableApriltagProcessing(true))
    
            );
        }
    
    }
    
    