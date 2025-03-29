package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;


// command template
public class RemoveAlgae extends Command {

    // constructor
    public RemoveAlgae() {

        // add subsystem requirements (if any) - for example:
        addRequirements(RobotContainer.algaeRemover);
    }

    // This method is called once when command is started
    @Override
    public void initialize() {

    }

    // This method is called periodically while command is active
    @Override
    public void execute() {
        RobotContainer.algaeRemover.RemoveAlgae(0.25);
    }

    // This method to return true only when command is to finish. Otherwise return false
    @Override
    public boolean isFinished() {
      
        return false;

    }

    // This method is called once when command is finished.
    @Override
    public void end(boolean interrupted) {
        RobotContainer.algaeRemover.RemoveAlgae(0.0);
    }

}