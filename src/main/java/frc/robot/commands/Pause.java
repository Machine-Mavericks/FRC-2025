package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;


// command template
public class Pause extends Command {

    private Timer delay;
    private double time;
    
    // constructor
    public Pause(double time) {

        delay = new Timer();
        this.time = time;
    }

    // This method is called once when command is started
    @Override
    public void initialize() {
        delay.reset();
        delay.start();
    }

    // This method is called periodically while command is active
    @Override
    public void execute() {

    }

    // This method to return true only when command is to finish. Otherwise return false
    @Override
    public boolean isFinished() {

        return delay.get() >= time; 

    }

    // This method is called once when command is finished.
    @Override
    public void end(boolean interrupted) {

    }

}