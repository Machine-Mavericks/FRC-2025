package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;


// command template
public class TemplateCommand extends Command {

    // constructor
    public TemplateCommand() {

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

    }

    // This method to return true only when command is to finish. Otherwise return false
    @Override
    public boolean isFinished() {

        return false;

    }

    // This method is called once when command is finished.
    @Override
    public void end(boolean interrupted) {

    }

}