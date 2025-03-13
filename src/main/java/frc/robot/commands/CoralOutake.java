package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;



// command template
public class CoralOutake extends Command {

    Timer delay;
    Timer timeaftersensor;

    boolean elevatorAtDestination;

    // constructor
    public CoralOutake() {

        // add subsystem requirements (if any) - for example:
        addRequirements(RobotContainer.intake);
        delay = new Timer();
        timeaftersensor = new Timer();

    }

    // This method is called once when command is started
    @Override
    public void initialize() {
        delay.reset();
        delay.start();

        timeaftersensor.reset();
        timeaftersensor.stop();
        
        if (RobotContainer.elevator.isElevatorAtDestination()) 
        {
            RobotContainer.intake.intakeRun(-1);    
            elevatorAtDestination = true;
        }
        else
            elevatorAtDestination = false;

    }

    // This method is called periodically while command is active
    @Override
    public void execute() {
       if  (!RobotContainer.intake.getSensorState())
        timeaftersensor.start();
    }
    // This method to return true only when command is to finish. Otherwise return false
    @Override
    public boolean isFinished() {
        // command is finished if elevator not at its destination, or time has expired or intakesensor 
        // no longer shows coral is present + 0.2s
        if (!elevatorAtDestination || delay.get() >= 2.0 || timeaftersensor.get() > 0.2){
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