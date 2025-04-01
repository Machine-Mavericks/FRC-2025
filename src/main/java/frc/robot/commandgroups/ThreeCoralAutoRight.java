package frc.robot.commandgroups;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
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
public class ThreeCoralAutoRight extends SequentialCommandGroup {

    // constructor
    public ThreeCoralAutoRight() {

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
            new InstantCommand(()-> RobotContainer.algaeRemover.AlgaeBloom()),

            new InstantCommand(()-> {
                if  (DriverStation.getAlliance().get()==Alliance.Blue)
                    RobotContainer.camr.SetPriorityTagID(17);
                else 
                    RobotContainer.camr.SetPriorityTagID(8);
            }),

            new MoveToPose(
                4.0,
                10.0,
                new Pose2d(3.7,2.3, new Rotation2d(Math.toRadians(60.0)))// was y 2.4
            ),
    
            // approach reef
            new InstantCommand(()-> RobotContainer.odometry.EnableApriltagProcessing(true)),

            
            // raise to level 4 height
            new InstantCommand(()->RobotContainer.elevator.Level4()),

            new ApproachReef(false),
    
            new InstantCommand(()-> RobotContainer.odometry.EnableApriltagProcessing(false)),

            new DepositeAndLower(),

            new ParallelCommandGroup(new CoralIntake(),
            // move to pickup 
                new MoveToPose(//waiting on point
                    4.0, 
                    8.0,
                    new Pose2d (1.039,0.8622, new Rotation2d(Math.toRadians(54.0))))
            ),         
    
            new InstantCommand(()-> {
                if  (DriverStation.getAlliance().get()==Alliance.Blue)
                    RobotContainer.camleft.SetPriorityTagID(17);
                else 
                    RobotContainer.camleft.SetPriorityTagID(8);
                }),

            // move to veiw point number two
            new MoveToPose(
                4.0, 
                8.0,
                new Pose2d(3.4,2.9, new Rotation2d(Math.toRadians(60)))
            ),
    
            new InstantCommand(()-> RobotContainer.odometry.EnableApriltagProcessing(true)),
            
            // rais elevator to level 4 
            new InstantCommand(()->RobotContainer.elevator.Level4()),

            // apprach reef 
            new ApproachReef(true), 
    
            new InstantCommand(()-> RobotContainer.odometry.EnableApriltagProcessing(false)),
    
            // deposite 
            new DepositeAndLower(),  

            new ParallelCommandGroup(new CoralIntake(),
            // move to pickup 
                new MoveToPose(//waiting on point
                    4.0, 
                    8.0,
                    new Pose2d (1.039,0.8622, new Rotation2d(Math.toRadians(54))))
            ),  
            
            new InstantCommand(()-> {
                if  (DriverStation.getAlliance().get()==Alliance.Blue)
                    RobotContainer.camr.SetPriorityTagID(18);
                else 
                    RobotContainer.camr.SetPriorityTagID(7);
            }),

            new MoveToPose(
                4.0, 
                8.0,
                new Pose2d(3.05,4.35, new Rotation2d(Math.toRadians(0.0)))
            ),

            new InstantCommand(()-> RobotContainer.odometry.EnableApriltagProcessing(true)),
            

            // rais elevator to level 4 
            new InstantCommand(()->RobotContainer.elevator.Level4()),

            // apprach reef 
            new ApproachReef(false), 
    
            new InstantCommand(()-> RobotContainer.odometry.EnableApriltagProcessing(false)),
    
            //new Pause(0.2),
            
            new InstantCommand(()->RobotContainer.intake.intakeRun(-1.0)),
    
            new Pause(0.2),
    
            new InstantCommand(()->RobotContainer.intake.intakeRun(0)),
    
            new Pause(0.2),

             // lower elevator 
            new InstantCommand(()->RobotContainer.elevator.Level0()),
            
            new Pause(2.0)
    
            );
        }
    
    }
    
    