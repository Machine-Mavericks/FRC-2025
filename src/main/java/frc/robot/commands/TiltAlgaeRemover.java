package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.AlgaeRemover;


// command template
public class TiltAlgaeRemover extends Command {
    //double deltaY = 0;
    double deltaY = RobotContainer.toolOp.getLeftY();
    // constructor
    public TiltAlgaeRemover() {

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
        //deltaY = RobotContainer. toolOp.getLeftY();
       RobotContainer.algaeRemover.Tilt(-0.15* RobotContainer.toolOp.getLeftY());
    }

    // This method to return true only when command is to finish. Otherwise return false
    @Override
    public boolean isFinished() {
        if (RobotContainer. toolOp.getLeftY() == 0){
            return true;
        }
        return false;

    }

    // This method is called once when command is finished.
    @Override
    public void end(boolean interrupted) {
        RobotContainer.algaeRemover.Tilt(0.0);
    }

}