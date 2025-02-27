package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;



// command template
public class CoralOutake extends Command {

    Timer delay;

    // constructor
    public CoralOutake() {

        // add subsystem requirements (if any) - for example:
        addRequirements(RobotContainer.intake);
        delay = new Timer();
    }

    // This method is called once when command is started
    @Override
    public void initialize() {
        RobotContainer.intake.intakeRun(-1);
    }

    // This method is called periodically while command is active
    @Override
    public void execute() {
       
    }
    // This method to return true only when command is to finish. Otherwise return false
    @Override
    public boolean isFinished() {
        if (delay.get() >= 2){
            return true;
        }else{
            return false;
        }
        
        

    }

    // This method is called once when command is finished.
    @Override
    public void end(boolean interrupted) {
        RobotContainer.intake.intakeRun(0);
    }

}