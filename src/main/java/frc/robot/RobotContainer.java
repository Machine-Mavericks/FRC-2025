package frc.robot;

import frc.robot.commandgroups.TwoStationDefense;
import frc.robot.commandgroups.MoveOffLineAnywhere;
import frc.robot.commandgroups.OneCoralAutoCenter;
import frc.robot.commandgroups.OneCoralAutoLeft;
import frc.robot.commandgroups.OneCoralAutoRight;
import frc.robot.commandgroups.OneStationDefense;
import frc.robot.commandgroups.TemplateCommandGroup;
import frc.robot.commandgroups.ThreeCoralAutoAnywhere;
import frc.robot.commandgroups.TwoCoralAutoAnywhere;
import frc.robot.commands.ApproachReef;
import frc.robot.commands.CoralBack;
import frc.robot.commands.CoralIntake;
import frc.robot.commands.CoralOutake;
import frc.robot.commands.MoveElevator;
import frc.robot.commands.MoveToPose;
import frc.robot.commands.Pause;
import frc.robot.commands.TemplateCommand;
import frc.robot.subsystems.CoralGrabber;
import frc.robot.subsystems.DeadWheel;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.LED;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Pigeon;
import frc.robot.subsystems.SwerveDrive;
import frc.robot.subsystems.TemplateSubsystem;
import frc.robot.subsystems.Odometry;
import frc.robot.utils.AutoFunctions;
import frc.robot.utils.ElevatorPositions;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SelectCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;


public class RobotContainer {
  
    // pointer to robot object
    public static Robot robot;

    // create driver and operator xBox controllers
    public static CommandXboxController driverOp;
    public static CommandXboxController toolOp;

    // main shuffleboar page
    public static MainShuffleBoardTab mainShufflePage;
  
    // make pointers to robot subsystems here
    public static Pigeon gyro;
    public static SwerveDrive drivesystem;
    public static Odometry odometry;
    public static TemplateSubsystem mySubsystem;
    public static Limelight camera;
    public static DeadWheel encoder;
    public static Elevator elevator;
    public static CoralGrabber intake;
    public static LED LED;
    // and so on

    
    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer(Robot robotptr) {

        // record pointer to robot object
        robot = robotptr;

        // create driver(port 0) and operator(port 1) controllers
        driverOp = new CommandXboxController(RobotMap.GamePadPorts.DriverID);
        toolOp = new CommandXboxController(RobotMap.GamePadPorts.OperatorID);
    
        // main shuffleboard page
        mainShufflePage = new MainShuffleBoardTab();

        // create instances of subsystems here
        gyro = new Pigeon();
        drivesystem = new SwerveDrive();
        odometry = new Odometry();
        mySubsystem = new TemplateSubsystem();
        camera = new Limelight("camera", true);
        encoder = new DeadWheel();
        elevator = new Elevator();
        intake = new CoralGrabber();
        LED = new LED();

        
        // and so on
    

        // attach commands to controller buttons
        configureBindings();
    }


    /** Use this method to define your trigger->command mappings. */
    private void configureBindings() {
    
        // attach commands to buttons
    
        // reset odometry to appropriate angle when back pressed.
        driverOp.back().onTrue(new InstantCommand(()-> {
            Pose2d pos = odometry.getPose2d();
            Rotation2d newHeading = AutoFunctions.redVsBlue(new Rotation2d(0.0));
            odometry.setPose(pos.getX(), pos.getY(), newHeading.getRadians(), newHeading.getRadians());
        } ));
        

        driverOp.start().onTrue(new InstantCommand(()->encoder.ResetEncoder()));

        driverOp.leftBumper().whileTrue(new ApproachReef(true));
        driverOp.rightBumper().whileTrue(new ApproachReef(false));

        // operator controls 
        toolOp.leftBumper().whileTrue(new CoralIntake());
        toolOp.rightBumper().whileTrue(new CoralOutake());
        toolOp.back().whileTrue(new CoralBack());

        toolOp.a().onTrue(new MoveElevator(ElevatorPositions.LEVEL_2));
        toolOp.x().onTrue(new MoveElevator(ElevatorPositions.LEVEL_3));
        toolOp.b().onTrue(new MoveElevator(ElevatorPositions.LEVEL_1));
        toolOp.y().onTrue(new MoveElevator(ElevatorPositions.LEVEL_4));


        toolOp.a().onFalse(new MoveElevator(ElevatorPositions.LEVEL_0));
        toolOp.x().onFalse(new MoveElevator(ElevatorPositions.LEVEL_0));
        toolOp.b().onFalse(new MoveElevator(ElevatorPositions.LEVEL_0));
        toolOp.y().onFalse(new MoveElevator(ElevatorPositions.LEVEL_0));

        //driverOp.a().onTrue(new MoveElevator(ElevatorPositions.LEVEL_1));
        
        // examples:
        // on press of driver controller A button, run example TemplateCommand

        // on press of operator controller X button, run example TemplateGroupCommand
        
        //driverOp.leftTrigger().whileTrue(new TwoStationDefense());
        //driverOp.rightTrigger().whileTrue(new OneStationDefense())

    
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
        int index = RobotContainer.mainShufflePage.getSelectedAutoIndex(); //RobotContainer.autopathselect.GetSelectedPath();

        Command chosenCommand =  null; 

    
        
        // return autonomous command to be run in autonomous
        if (index == 0)
            chosenCommand = new Pause(20.0); // do nothing command 
        else if (index == 1)
            chosenCommand = new MoveOffLineAnywhere(); // drive off the line 
            else if (index == 2)
            chosenCommand = new OneCoralAutoRight();
            else if (index == 3)
            chosenCommand = new OneCoralAutoCenter();
            else if (index == 4)
            chosenCommand = new OneCoralAutoLeft();
        

        return new SequentialCommandGroup(
            new Pause(RobotContainer.mainShufflePage.getAutoDelay()),
            chosenCommand);
  }
  
}
