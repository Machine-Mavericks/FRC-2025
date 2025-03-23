package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;


// command template
public class ClimbCommand extends Command {

    private boolean m_Climb;
    // constructor
    public ClimbCommand(boolean climbNoClimb) {

        m_Climb = climbNoClimb;
        // add subsystem requirements (if any) - for example:
        //addRequirements(RobotContainer.drivesystem);
        addRequirements(RobotContainer.climb);
    }

    // This method is called once when command is started
    @Override
    public void initialize() {
       if (m_Climb == true){
        RobotContainer.climb.SetSpeed(-0.15);
       }else{ 
        RobotContainer.climb.SetSpeed(0.15);
       }
    }

    // This method is called periodically while command is active
    @Override
    public void execute() {

    }

    // This method to return true only when command is to finish. Otherwise return false
    @Override
    public boolean isFinished() {

        if(RobotContainer.climb.getSwitchState()){
            return true;
        }else{
            return false;
        }

    }

    // This method is called once when command is finished.
    @Override
    public void end(boolean interrupted) {
        RobotContainer.climb.SetSpeed(0);
    }

    

}