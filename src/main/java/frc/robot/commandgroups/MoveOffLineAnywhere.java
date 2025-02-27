package frc.robot.commandgroups;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.commands.MoveToPose;
import frc.robot.commands.RobotDriveFixedTime;
import frc.robot.utils.AutoFunctions;

// Example Sequential Command Group
// There are also:
// ParallelCommandGroup
// ParallelRaceGroup
// ParallelDeadlineGroup
public class MoveOffLineAnywhere extends SequentialCommandGroup {

    // constructor
    public MoveOffLineAnywhere() {

        addCommands (

        // zero gyro (mirroring for red vs blue as necessary)
        new InstantCommand(()-> {
            Pose2d pos = RobotContainer.odometry.getPose2d();
            Rotation2d newHeading = AutoFunctions.redVsBlue(new Rotation2d(0.0));
            RobotContainer.odometry.setPose(pos.getX(), pos.getY(), newHeading.getRadians(), newHeading.getRadians());
        } ),

        // drive robot backwards at 0.5m/s for 2.0s duration
        new RobotDriveFixedTime(-0.5, 0.0, 2.0)

        );
    }

}

