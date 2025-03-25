package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;



// command template
public class L1CoralOuttake extends Command {

    Timer delay;
    Timer timeaftersensor;

    boolean elevatorAtDestination;

    // constructor
    public L1CoralOuttake() {

        // add subsystem requirements (if any) - for example:
        addRequirements(RobotContainer.intake);
        delay = new Timer();

    }

    // This method is called once when command is started
    @Override
    public void initialize() {
        delay.reset();
        delay.start();
        
        // if (RobotContainer.elevator.isElevatorAtDestination()) 
        // {
        //     RobotContainer.intake.intakeRun(-0.2);    
        //     elevatorAtDestination = true;
        // }
        // else
        //     elevatorAtDestination = false;

    }

    // This method is called periodically while command is active
    @Override
    public void execute() {
        RobotContainer.intake.intakeRun(-0.2);    
        
        if (!RobotContainer.intake.getSensorState()) 
        {
            RobotContainer.intake.sillyIntakeRun(1);    
        }
       

    }
    // This method to return true only when command is to finish. Otherwise return false
    @Override
    public boolean isFinished() {
        // command is finished if elevator not at its destination, or time has expired or intakesensor 
        // no longer shows coral is present + 0.2s
        if (!elevatorAtDestination || delay.get() >= 2.0 ){
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