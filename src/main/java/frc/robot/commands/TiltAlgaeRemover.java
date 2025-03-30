package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.AlgaeRemover;
import frc.robot.utils.AlgaePositions;


// command template
public class TiltAlgaeRemover extends Command {
    private AlgaePositions targetLevel;
    //double deltaY = 0;
    //double deltaY = RobotContainer.toolOp.getLeftY();
    
    // constructor
    public TiltAlgaeRemover(AlgaePositions position) {
        targetLevel = position;
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
        //double Tilt = RobotContainer.toolOp.getRightTriggerAxis(); 
        
        //double RemoveAlgae = RobotContainer.toolOp.getLeftTriggerAxis();
        //RobotContainer.algaeRemover.RemoveAlgae(0.15*RemoveAlgae);
        
        //if (RobotContainer.toolOp.getRightTriggerAxis()>0.5){
        //    RobotContainer.algaeRemover.Tilt(Tilt);  
        //}
        //else{
        //    RobotContainer.algaeRemover.Tilt(-1.0);
        //}
        switch (targetLevel) {
            case TILT_UP:
                RobotContainer.algaeRemover.ResetTilt();
                break;
            case TILT_DOWN:
                RobotContainer.algaeRemover.RemoveAlgae();
                break;
        }
    }

    // This method to return true only when command is to finish. Otherwise return false
    @Override
    public boolean isFinished() {
        // if (RobotContainer.toolOp.getRightTriggerAxis() == 0){
        //     return true;
        // }
        // return false;
        return true;

    }

    // This method is called once when command is finished.
    @Override
    public void end(boolean interrupted) {
        RobotContainer.algaeRemover.ResetTilt();
    }

    
}