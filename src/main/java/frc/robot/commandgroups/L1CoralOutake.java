package frc.robot.commandgroups;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.commands.ApproachReef;
import frc.robot.commands.CoralOutake;
import frc.robot.commands.L1CoralOutakePart1;
import frc.robot.commands.L1CoralOutakePart2;
import frc.robot.commands.MoveElevator;
import frc.robot.commands.MoveToPose;
import frc.robot.commands.Pause;
import frc.robot.utils.AutoFunctions;
import frc.robot.utils.ElevatorPositions;

// Example Sequential Command Group
// There are also:
// ParallelCommandGroup
// ParallelRaceGroup
// ParallelDeadlineGroup
public class L1CoralOutake extends SequentialCommandGroup {

    // constructor
    public L1CoralOutake() {

        addCommands (

            new MoveElevator(ElevatorPositions.LEVEL_1),

            new Pause(0.25),

            new L1CoralOutakePart1(),

            new L1CoralOutakePart2(),

            new Pause(0.25),

            new MoveElevator(ElevatorPositions.LEVEL_0)

        );
    }

}

