package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.utils.ElevatorPositions;


// command template
public class MoveElevator extends Command {
    private ElevatorPositions targetLevel;
    // constructor
    public  MoveElevator(ElevatorPositions position) {
        targetLevel = position;
        // add subsystem requirements (if any) - for example:
        //addRequirements(RobotContainer.drivesystem);
    }

    // This method is called once when command is started
    @Override
    public void initialize() {

    }

    // This method is called periodically while command is active
    @Override
    public void execute() {
        switch (targetLevel) {
            case INTAKE:
                RobotContainer.elevator.returnToIntake();
                break;
            case LEVEL_1:
                RobotContainer.elevator.Level1();
                break;
            case LEVEL_2:
                RobotContainer.elevator.Level2();
                break;
            case LEVEL_3:
                RobotContainer.elevator.Level3();
                break;
            case LEVEL_4:
                RobotContainer.elevator.Level4();
                break;
            default:
                break;
        }
    }
   
    // This method to return true only when command is to finish. Otherwise return false
    @Override
    public boolean isFinished() {

        return true;

    }

    // This method is called once when command is finished.
    @Override
    public void end(boolean interrupted) {
      

    }

}