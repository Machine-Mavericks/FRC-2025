// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.utils.AutoFunctions;
import frc.robot.utils.Utils;

// command controls mecanum in manual mode
public class ManualDrive extends Command {

    // PID controller used to counteract rotational drift due to misalignment of wheels
    // note: FRC 2024 used P=0.01, I=0.0, D=0.0
    // FTC 2024 used P=0.07, I=0.0005, D=0.0
    private PIDController m_headingPID = new PIDController(0.08, 0.2, 0);  // p=0.04 i=0.0005
    private Double m_PIDTarget = null;    // Use Double class so it can be set to null
    private long m_pidDelay = -1;

    

    double powerFactor;
    double basePowerFacter = 0.12;
    double boostPowerFacter = 0.88;

    final double MAX_ACCEL = 100.0;  // max accel in m/s2

    double old_dX, old_dY;

    Timer deltat;

    // rotate to reef controller
    PIDController omegaControl;

    
    // constructor
    public ManualDrive() {

        // this command requires swerve drive subsystem
        addRequirements(RobotContainer.drivesystem);

        deltat = new Timer();

        omegaControl = new PIDController(0.09, 0.001, 0.0000);
    }

    // This method is called once when command is started
    @Override
    public void initialize() {
        m_headingPID.setIntegratorRange(-7.0, 7.0);

        
        
        m_PIDTarget = null;
        m_pidDelay = 10;
        old_dX=0.0;
        old_dY = 0.0;
        deltat.reset();
        deltat.start();
        omegaControl.reset();

    }

    // This method is called periodically while command is active
    @Override
    public void execute() {

        // get joystick input - for competition
        // Note: FRC joystick is oriented +ve to left and +ve down
        double dX = -RobotContainer.driverOp.getLeftY();
        double dY = -RobotContainer.driverOp.getLeftX();
        double omega = -2.0 * RobotContainer.driverOp.getRightX();
        double speedTrigger = RobotContainer.driverOp.getRightTriggerAxis();
        
        //boolean Park = RobotContainer.driverOp.leftBumper().getAsBoolean();

        // if on red team, reverse x and y movements to match FRC field coordinates
        if (DriverStation.getAlliance().get() == Alliance.Red)
        {
            dX = dX * -1;
            dY = dY * -1;
        }


        // implement dead-zoning of joystick inputs
        // made the dead zones a circle instead of a square, old equation commented below
        dX = Math.sqrt(Math.pow(dX, 2) + Math.pow(dY, 2)) > 0.1 ? dX : 0;
        dY = Math.sqrt(Math.pow(dX, 2) + Math.pow(dY, 2)) > 0.1 ? dY : 0;
        omega = Math.sqrt(Math.pow(omega, 2) + Math.pow(RobotContainer.driverOp.getRightY(), 2)) > 0.1  ? omega : 0;
        // dX = Math.abs(dX) > 0.1 ? dX : 0;
        // dY = Math.abs(dY) > 0.1 ? dY : 0;
        // omega = Math.abs(omega) > 0.2 ? omega : 0;
        

        boolean EnableDriftCorrection = true;
        if (RobotContainer.intake.getSensorState() && RobotContainer.snapToReef)
        {
            
            // determine distance from center of reef
            Translation2d CurrentPosition = RobotContainer.odometry.getPose2d().getTranslation();
            Translation2d CenterReef = AutoFunctions.redVsBlue(new Translation2d(4.489, 4.0259));

            // heading to center of reef
            double ReefAngle = CenterReef.minus(CurrentPosition).getAngle().getDegrees();
            
            // current robot angle
            double RobotAngle = RobotContainer.gyro.getYawAngle();

            /*double TargetAngle=0.0;
            if (ReefAngle > -30.0 && ReefAngle <=30.0)
                TargetAngle = 0.0;
            else if (ReefAngle > -90.0 && ReefAngle <=-30.0)
                TargetAngle = -60.0;
            else if (ReefAngle > -150.0 && ReefAngle <=-90.0)
                TargetAngle = -120.0;
            else if (ReefAngle <= -150.0 || ReefAngle >= 150.0)
                TargetAngle = -180.0;
            else if (ReefAngle >30.0 && ReefAngle <=90.0)
                TargetAngle = 60.0;
            else if (ReefAngle >90.0 && ReefAngle < 150.0)
                TargetAngle = 120.0; */


            double TargetAngle=0.0;
            if (ReefAngle > -35.0 && ReefAngle <=35.0)
                TargetAngle = 0.0;
            else if (ReefAngle > -95.0 && ReefAngle <=-25.0)
                TargetAngle = -60.0;
            else if (ReefAngle > -155.0 && ReefAngle <=-85.0)
                TargetAngle = -120.0;
            else if (ReefAngle <= -145.0 || ReefAngle >= 145.0)
                TargetAngle = -180.0;
            else if (ReefAngle >25.0 && ReefAngle <=95.0)
                TargetAngle = 60.0;
            else if (ReefAngle >85.0 && ReefAngle < 155.0)
                TargetAngle = 120.0;
            
                double error = Utils.AngleDifference(TargetAngle,RobotAngle);
                error = Math.abs(error) > 1.0 ? error : 0;
            
            omega = omegaControl.calculate(error);
             //omega = omegaControl.calculate(Utils.AngleDifference(30.0,RobotAngle));
            
            // deadband the control
            //omega = Math.abs(omega) > 0.1 ? omega : 0;
            
            if (omega > 3.0)
                    omega = 3.0;
            if (omega < -3.0)
                    omega = -3.0;

            EnableDriftCorrection = false;
        }
        else{
            omegaControl.reset();
        }
        
    

        // --------- Correct robot angle for gyro angle wander --------
        if(EnableDriftCorrection && omega == 0.0 && !RobotContainer.driverOp.back().getAsBoolean())
        {
            if (m_pidDelay > 0)
                m_pidDelay --;
            else
            {
                // If the target is unset, set it to current heading
                if(m_PIDTarget == null)
                {
                    m_PIDTarget = RobotContainer.gyro.getYawAngle();
                    m_headingPID.reset(); // Clear existing integral term as may accumulate while not in use
                }

                omega = m_headingPID.calculate(Utils.AngleDifference(m_PIDTarget, RobotContainer.gyro.getYawAngle()));
            }
        }
        else
        {
            // there is rotational input, or gyro reset was pressed, set target to null so it's properly reset next time
            m_PIDTarget = null;
            m_pidDelay = 10;
        }
        // --------- End Correct robot angle for gyro angle wander --------


        // Need to deadzone omega again. Anti-wander function above always results in
        // very small non-zero robot rotation. This resutls in wheels always resetting to 
        // robot rotate position when robot in rest position.
        // With deadzoning, this allows wheel orientation to remain as it was.
        omega = Math.abs(omega) > 0.07 ? omega : 0;


        powerFactor = basePowerFacter + (speedTrigger * boostPowerFacter);
        // Since the drive was shifted to closed loop (i.e. requested velocities), change joystick input max values
        // to MAX_SPEED values.
        powerFactor = powerFactor * RobotContainer.drivesystem.MAX_SPEED;

        // include power factor to get full x,y and omega speeds beings requested
        dX *= powerFactor;
        dY *= powerFactor;
        omega *= powerFactor;

        // limit acceleration of robot to MAX_ACCEL - to reduce chance of wheel-slide
        double t = deltat.get();
        if ((dX > old_dX + t*MAX_ACCEL) && (old_dX>0))
            dX = old_dX + t*MAX_ACCEL;
        if ((dX <  old_dX - t*MAX_ACCEL) && (old_dX<0))
            dX = old_dX - t*MAX_ACCEL;

        if ((dY > old_dY + t*MAX_ACCEL) && (old_dY>0))
            dY = old_dY + t*MAX_ACCEL;
        if ((dY < old_dY - t*MAX_ACCEL) && (old_dY<0))
            dY = old_dY - t*MAX_ACCEL;

        // save speeds for use next time
        old_dX = dX;
        old_dY = dY;
        deltat.reset();

        // drive robot
        //if (!Park)
            RobotContainer.drivesystem.FieldDrive(dX, dY, omega, false);
        //else
            //RobotContainer.drivesystem.FieldDrive(0.0, 0.0, 0.0, true);
    }

    // This method to return true only when command is to finish. Otherwise return false
    @Override
    public boolean isFinished() {

        return false;
    }

    // This method is called once when command is finished.
    @Override
    public void end(boolean interrupted) {

    }

}