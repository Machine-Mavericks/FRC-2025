package frc.robot;

import frc.robot.commandgroups.TemplateCommandGroup;
import frc.robot.commands.TemplateCommand;
import frc.robot.subsystems.NavX;
import frc.robot.subsystems.TemplateSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;


public class RobotContainer {
  
  // create driver and operator xBox controllers
  CommandXboxController driverOp;
  CommandXboxController toolOp;

  
  // make pointers to robot subsystems here

  // NavX gyro
  public static NavX gyro;

  // example
  public static TemplateSubsystem mySubsystem;
  // and so on



  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    
    // create driver(port 0) and operator(port 1) controllers
    driverOp = new CommandXboxController(RobotMap.GamePadPorts.DriverID);
    toolOp = new CommandXboxController(RobotMap.GamePadPorts.OperatorID);
    
    // create instances of subsystems here
    gyro = new NavX();
    mySubsystem = new TemplateSubsystem();
    
    // and so on
    

    // attach commands to controller buttons
    configureBindings();
  }


  /** Use this method to define your trigger->command mappings. */
  private void configureBindings() {
    
    // attach commands to buttons
    // for example:

    // on press of driver controller A button, run example TemplateCommand
    driverOp.a().onTrue(new TemplateCommand());
    // on press of operator controller X button, run example TemplateGroupCommand
    toolOp.x().onTrue(new TemplateCommandGroup());
  
    
    // description of commands available:
    // .onTrue - runs command when button changes from not-pressed to pressed.
    // .onFalse - runs command when button changes from pressed to not-pressed.
    // .onChange - runs command when state changes either way
    // .whileTrue - runs command only while button is pressed - command does not restart if finished
    // .whileFalse - runs command only while button is not pressed - command does not restart if finished

    // to have command automatically repeat if it finishes while button is pressed or whatever
    // toolOp.back().whileTrue(new RepeatCommand(new TemplateCommand()));

    // to debounce the trigger event
    // driverOp.y().debounce(0.5).onTrue(new TemplateCommand());

    // to use a trigger as a button - note: analog triggers should be debounced as well
    // driverOp.rightTrigger(0.5).debounce(0.25).onTrue(new TemplateCommand());
  }


  /** Use this function to return pointer to the command the robot is to follow in autonomous
   * @return the command to run in autonomous */
  public Command getAutonomousCommand() {
    
    // get autonomous path to run
    // for example, a subsystem could made responsible for returning selected path
    // from a list populated in shuffleboard.
    int index = 0; //RobotContainer.autopathselect.GetSelectedPath();
    
    // return autonomous command to be run in autonomous
    if (index == 0)
      return new TemplateCommandGroup();
    else if (index == 1)
      return new TemplateCommandGroup();
    else
      return null;
    
  }
}
