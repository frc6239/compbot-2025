// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import com.revrobotics.spark.config.LimitSwitchConfig;
import frc.robot.Constants.OuttakeConstants;
import frc.robot.subsystems.Outtake;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to each mode, as
 * described in the TimedRobot documentation. If you change the name of this class or the package after creating this
 * project, you must also update the build.gradle file in the project.
 */
public class Robot extends TimedRobot
{
  public boolean m_elevatorEnabled = true;
  private static Robot   instance;
  private        Command m_autonomousCommand;

  private RobotContainer m_robotContainer;
  

  private Timer disabledTimer;

  public Robot()
  {
    instance = this;
  }

  public static Robot getInstance()
  {
    return instance;
  }

  /**
   * This function is run when the robot is first started up and should be used for any initialization code.
   */
  @Override
  public void robotInit()
  {
  
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();
    m_robotContainer.m_LedController.set_Orange();

    // Create a timer to disable motor brake a few seconds after disable.  This will let the robot stop
    // immediately when disabled, but then also let it be pushed more 
    disabledTimer = new Timer();

    if (isSimulation())
    {
      DriverStation.silenceJoystickConnectionWarning(true);
    }
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics that you want ran
   * during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic()
  {
     SmartDashboard.putNumber("Actual Position", m_robotContainer.m_climberSubsystem.getPosition());
    SmartDashboard.putNumber( "Actual Velocity", m_robotContainer.m_climberSubsystem.getVelocity());
    SmartDashboard.putNumber( "Drive Scaled Speed", m_robotContainer.drivebase.getScaleSpeed());


    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
  }

  /**
   * This function is called once each time the robot enters Disabled mode.
   */
  @Override
  public void disabledInit()
  {
    m_robotContainer.setMotorBrake(true);
    disabledTimer.reset();
    disabledTimer.start();
  }

  @Override
  public void disabledPeriodic()
  {
    if (disabledTimer.hasElapsed(Constants.DrivebaseConstants.WHEEL_LOCK_TIME))
    {
      m_robotContainer.setMotorBrake(false);
      disabledTimer.stop();
    }
  }

  /**
   * This autonomous runs the autonomous command selected by your {@link RobotContainer} class.
   */
  @Override
  public void autonomousInit()
  {
    m_robotContainer.drivebase.zeroGyro();
    m_robotContainer.setMotorBrake(true);
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null)
    {
      m_autonomousCommand.schedule();
    }
  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic()
  {
     // Check the state of the beam break sensor
     if (m_robotContainer.m_OuttakeSubsystem.beamBreakCleared()) {
      // if beam is clear, enable elevator
      m_elevatorEnabled = true;  // Motor stops
      m_robotContainer.m_ElevatorSubsystem.enableElevator();
      m_robotContainer.m_LedController.set_Green();
  } else {
    // If beam is blocked, disable elevator
    m_elevatorEnabled = false;
    m_robotContainer.m_ElevatorSubsystem.disableElevator();
    m_robotContainer.m_LedController.set_RedOrange();
  
  }
  }

  @Override
  public void teleopInit()
  {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.

    m_robotContainer.configureDriveCommand();

    if (m_autonomousCommand != null)
    {
      m_autonomousCommand.cancel();
    } else
    {
      CommandScheduler.getInstance().cancelAll();
    }

    
    
  }

  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic()
  {

     // Check the state of the beam break sensor
     if (m_robotContainer.m_OuttakeSubsystem.beamBreakCleared() == true) {
      // if beam is clear, enable elevator
      m_elevatorEnabled = true;  // Motor stops
      m_robotContainer.m_ElevatorSubsystem.enableElevator();
      m_robotContainer.m_LedController.set_Green();

    //System.out.println("elevator enabled " + m_elevatorEnabled);
  } else {
    // If beam is blocked, disable elevator
    m_elevatorEnabled = false;
    m_robotContainer.m_ElevatorSubsystem.disableElevator();
    m_robotContainer.m_LedController.set_RedOrange();
    //System.out.println("elevator enabled " + m_elevatorEnabled);
  
  }
  m_robotContainer.m_scaleSpeed = m_robotContainer.drivebase.getScaleSpeed();


    /*if (SmartDashboard.getBoolean("Control Mode", false)) {
      /*
       * Get the target position from SmartDashboard and set it as the setpoint
       * for the closed loop controller with MAXMotionPositionControl as the
       * control type.
       */
     // double targetPosition = SmartDashboard.getNumber("Target Position", 0);
     // m_robotContainer.m_climberSubsystem.setPosition(targetPosition); }
      
  }

  @Override
  public void testInit()
  {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic()
  {
  }

  /**
   * This function is called once when the robot is first started up.
   */
  @Override
  public void simulationInit()
  {
  }

  /**
   * This function is called periodically whilst in simulation.
   */
  @Override
  public void simulationPeriodic()
  {
  }
}
