package frc.robot.commandgroups;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.MoveToPose;
import frc.robot.utils.AutoFunctions;

// Example Sequential Command Group
// There are also:
// ParallelCommandGroup
// ParallelRaceGroup
// ParallelDeadlineGroup
public class OneStationDefense extends SequentialCommandGroup {

  // constructor
  public OneStationDefense() {

    addCommands(
        new MoveToPose(
            1,
            1,
            AutoFunctions.redVsBlue(new Pose2d(15.5, 6.8, new Rotation2d(Math.toRadians(55))))),
        
        new MoveToPose(
            1,
            1,
            AutoFunctions.redVsBlue(new Pose2d(16.4, 6.2, new Rotation2d(Math.toRadians(48)))))
        

    );
  }

}

// Example #1: Lily's 2023 FRC super cube auto
/*
 * // enable arm, and lift to stow position
 * new InstantCommand(() -> RobotContainer.arm.SetEnableArm(true)),
 * 
 * // move arm back to drop off cube
 * new InstantCommand(() ->
 * RobotContainer.arm.SetArmPosition(RobotContainer.arm.HIGH_DEG)),
 * 
 * // delay until arm gets back
 * new DelayCommand(1.0),
 * 
 * // place cube
 * new InstantCommand(() -> RobotContainer.grabber.setClose()),
 * 
 * // delay for gripper to close
 * new DelayCommand(0.7),
 * 
 * // move arm to 'forward position' but inside robot bumper)
 * // move to 135deg
 * new InstantCommand(() -> RobotContainer.arm.SetArmPosition(135.0)),
 * 
 * // delay for arm to get to stow
 * new DelayCommand(1.5),
 * 
 * // ensure arm is stowed before it is allow to begin moving over charge
 * station
 * new SafetyCheckStowPosition(),
 * 
 * // drive right
 * // new DrivetoRelativePose(new Pose2d(0,-2.0, new Rotation2d(0.0)), 1.0, 0.1,
 * 5.0),
 * 
 * // drive straight
 * new DrivetoRelativePose(new Pose2d(5.0, 0, new Rotation2d(0.0)),1.8,0.1,
 * 7.0),
 * 
 * // pick up cube from floor :)
 * new AutoFloorCubePickup(),
 * 
 * // delay
 * new DelayCommand(0.5),
 * 
 * // drive back
 * //new DrivetoRelativePose(new Pose2d(1.0,0, new Rotation2d(0.0)), 1.0, 0.1,
 * 2.0),
 * 
 * // drive left to center
 * new DrivetoRelativePose(new Pose2d(-1.0,2.0, new Rotation2d(0.0)), 1.8, 0.1,
 * 5.0),
 * 
 * // drive straight onto charge station
 * new DrivetoRelativePose(new Pose2d(-1.5, 0, new Rotation2d(0.0)),1.0,0.1,
 * 30.0),
 * 
 * // balance
 * new AutoBalance()
 * 
 * // Example #2: Matthew's 2024 shoot donut sequence.
 * This sequence contains parallel and parallelrace subgroups within an overall
 * series command
 * 
 * addCommands(
 * new ParallelRaceGroup(
 * new AimToSpeaker(),
 * new SpinupSpeaker()
 * ),
 * new ParallelCommandGroup(
 * new WaitForEffectorAngle(),
 * new WaitForShooterSpinup()
 * ),
 * 
 * new ShootSpeaker()
 * );
 * 
 */
