package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;


// command template
public class L1CoralOutakePart2 extends Command {

    Timer delay;
   
    // constructor
    public L1CoralOutakePart2() {

        // add subsystem requirements (if any) - for example:
        addRequirements(RobotContainer.intake);
       
        delay = new Timer();
    }

    // This method is called once when command is started
    @Override
    public void initialize() {
        delay.reset();
        delay.start();

        RobotContainer.intake.intakeRunBottom(-0.6); 
        RobotContainer.intake.intakeRunTop(0.0);

    }

    // This method is called periodically while command is active
    @Override
    public void execute() {
      
    }
    // This method to return true only when command is to finish. Otherwise return false
    @Override
    public boolean isFinished() {
        
        // run for 1.0s before coral has left
        return delay.hasElapsed(0.4); 
    }

    // This method is called once when command is finished.
    @Override
    public void end(boolean interrupted) {
        RobotContainer.intake.intakeRunBottom(0.0);
        RobotContainer.intake.intakeRunTop(0.0);
    }

}