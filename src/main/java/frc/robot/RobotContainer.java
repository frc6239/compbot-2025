// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.HttpCamera;
import edu.wpi.first.cscore.VideoSink;
import edu.wpi.first.cscore.VideoSource.ConnectionStrategy;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import java.io.File;
import java.util.Map;

import swervelib.SwerveInputStream;
import frc.robot.subsystems.LEDController;
import frc.robot.subsystems.Climber;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.ClimberConstants;
import frc.robot.Constants.DrivebaseConstants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Outtake;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a "declarative" paradigm, very
 * little robot logic should actually be handled in the {@link Robot} periodic methods (other than the scheduler calls).
 * Instead, the structure of the robot (including subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer
{
  // The robot's subsystems and commands are defined here...
  public final Climber m_climberSubsystem = new Climber();
  public final Elevator m_ElevatorSubsystem = new Elevator();
  public final Outtake m_OuttakeSubsystem = new Outtake();

  // Replace with CommandPS4Controller or CommandJoystick if needed
  final         CommandXboxController driverXbox = new CommandXboxController(0);
  public double m_scaleSpeed = DrivebaseConstants.FAST_SPEED;
  // The robot's subsystems and commands are defined here...
  public final SwerveSubsystem       drivebase  = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),
                                                                                "swerve/neo.6239"));
  public LEDController m_LedController = new LEDController();

  /**
   * Converts driver input into a field-relative ChassisSpeeds that is controlled by angular velocity.
   */
  
  SwerveInputStream driveAngularVelocityBlue = SwerveInputStream.of(drivebase.getSwerveDrive(),
                                                                () -> driverXbox.getLeftY() * -1 * m_scaleSpeed,
                                                                () -> driverXbox.getLeftX() * -1 * m_scaleSpeed)
                                                            .withControllerRotationAxis(() -> driverXbox.getRightX() * -1)
                                                            .deadband(OperatorConstants.DEADBAND)
                                                            .scaleTranslation(0.8)
                                                            .allianceRelativeControl(true);

  SwerveInputStream driveAngularVelocityRed = SwerveInputStream.of(drivebase.getSwerveDrive(),
                                                                () -> driverXbox.getLeftY() * -1 * m_scaleSpeed,
                                                                () -> driverXbox.getLeftX() * -1 * m_scaleSpeed)
                                                            .withControllerRotationAxis(() -> driverXbox.getRightX() * -1)
                                                            .deadband(OperatorConstants.DEADBAND)
                                                            .scaleTranslation(0.8)
                                                            .allianceRelativeControl(true);

  /**
   * Clone's the angular velocity input stream and converts it to a fieldRelative input stream.
   */
  SwerveInputStream driveDirectAngle = driveAngularVelocityBlue.copy().withControllerHeadingAxis(driverXbox::getRightX,
                                                                                             driverXbox::getRightY)
                                                           .headingWhile(true);

  /**
   * Clone's the angular velocity input stream and converts it to a robotRelative input stream.
   */
  SwerveInputStream driveRobotOriented = driveAngularVelocityBlue.copy().robotRelative(true)
                                                             .allianceRelativeControl(false);

  SwerveInputStream driveAngularVelocityKeyboard = SwerveInputStream.of(drivebase.getSwerveDrive(),
                                                                        () -> -driverXbox.getLeftY(),
                                                                        () -> -driverXbox.getLeftX())
                                                                    .withControllerRotationAxis(() -> driverXbox.getRawAxis(
                                                                        2))
                                                                    .deadband(OperatorConstants.DEADBAND)
                                                                    .scaleTranslation(0.8)
                                                                    .allianceRelativeControl(true);
  // Derive the heading axis with math!
  SwerveInputStream driveDirectAngleKeyboard     = driveAngularVelocityKeyboard.copy()
                                                                               .withControllerHeadingAxis(() ->
                                                                                                              Math.sin(
                                                                                                                  driverXbox.getRawAxis(
                                                                                                                      2) *
                                                                                                                  Math.PI) *
                                                                                                              (Math.PI *
                                                                                                               2),
                                                                                                          () ->
                                                                                                              Math.cos(
                                                                                                                  driverXbox.getRawAxis(
                                                                                                                      2) *
                                                                                                                  Math.PI) *
                                                                                                              (Math.PI *
                                                                                                               2))
                                                                               .headingWhile(true);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer()
  {
    // Configure the trigger bindings
    configureBindings();
    configurePathPlannerCommands();
    configureSmartDashboard();
   
    // We are displaying cameras via HTML webpage located under
    // shuffleboard/6239_Cameras.html
    // Turnning off cameras from going to shuffleboard
    //configureCameraServer();
    m_scaleSpeed = DrivebaseConstants.FAST_SPEED;

    DriverStation.silenceJoystickConnectionWarning(true);
    NamedCommands.registerCommand("test", Commands.print("I EXIST"));

  }

  public void configureSmartDashboard()
  {
    SmartDashboard.putData("Deploy arm", Commands.runOnce(m_climberSubsystem::deploy, m_climberSubsystem));
    SmartDashboard.putData("Lift robot", Commands.runOnce(m_climberSubsystem::lift, m_climberSubsystem));
    SmartDashboard.putData("Reset arm encoder", Commands.runOnce(m_climberSubsystem::resetEncoder));
  
    if (DriverStation.isTest()){
        SmartDashboard.putData("Retract", Commands.runOnce(m_climberSubsystem::retract,m_climberSubsystem));
    }

    /*Shuffleboard.getTab("Autonomous")
    .addCamera("Driver Cam", "Climber", "mjpg:http://photonvision.local:1182/?sction=stream")
    .withProperties(Map.of("showControls",false))
    .withPosition(2, 0)
    .withSize(3,3);

    Shuffleboard.getTab("Autonomous")
    .addCamera("Climb Cam", "Climber", "mjpg:http://photonvision.local:1181/?sction=stream")
    .withProperties(Map.of("showControls",false))
    .withPosition(2, 0)
    .withSize(3,3);*/




  }


  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary predicate, or via the
   * named factories in {@link edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
   * {@link CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller PS4}
   * controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight joysticks}.
   */
  private void configureBindings()
  {

    Command driveFieldOrientedDirectAngle      = drivebase.driveFieldOriented(driveDirectAngle);
    Command driveFieldOrientedAnglularVelocity = drivebase.driveFieldOriented(driveAngularVelocityBlue);
    Command driveRobotOrientedAngularVelocity  = drivebase.driveFieldOriented(driveRobotOriented);
    Command driveSetpointGen = drivebase.driveWithSetpointGeneratorFieldRelative(
        driveDirectAngle);
    Command driveFieldOrientedDirectAngleKeyboard      = drivebase.driveFieldOriented(driveDirectAngleKeyboard);
    Command driveFieldOrientedAnglularVelocityKeyboard = drivebase.driveFieldOriented(driveAngularVelocityKeyboard);
    Command driveSetpointGenKeyboard = drivebase.driveWithSetpointGeneratorFieldRelative(
        driveDirectAngleKeyboard);

    if (RobotBase.isSimulation())
    {
      drivebase.setDefaultCommand(driveFieldOrientedDirectAngleKeyboard);
    } else
    {
      configureDriveCommand();
      
    }

    if (Robot.isSimulation())
    {
      driverXbox.start().onTrue(Commands.runOnce(() -> drivebase.resetOdometry(new Pose2d(3, 3, new Rotation2d()))));
      driverXbox.button(1).whileTrue(drivebase.sysIdDriveMotorCommand());

    }
    if (DriverStation.isTest())
    {
      //drivebase.setDefaultCommand(driveFieldOrientedAnglularVelocity); // Overrides drive command above!

     // driverXbox.x().whileTrue(Commands.runOnce(drivebase::lock, drivebase).repeatedly());
     // driverXbox.y().whileTrue(drivebase.driveToDistanceCommand(1.0, 0.2));
     // driverXbox.start().onTrue((Commands.runOnce(drivebase::zeroGyro)));
      //driverXbox.back().whileTrue(drivebase.centerModulesCommand());
      //driverXbox.leftBumper().onTrue(Commands.none());
     // driverXbox.rightBumper().onTrue(Commands.none());
    } else
    {
      //driverXbox.a().onTrue((Commands.runOnce(drivebase::zeroGyro)));
      //driverXbox.x().onTrue(Commands.runOnce(drivebase::addFakeVisionReading));
      //driverXbox.b().whileTrue(
          //drivebase.driveToPose(
              //new Pose2d(new Translation2d(4, 4), Rotation2d.fromDegrees(0)))
                              //);
      driverXbox.start().whileTrue(Commands.none());
      driverXbox.back().whileTrue(Commands.none());
      //driverXbox.leftBumper().whileTrue(Commands.runOnce(drivebase::lock, drivebase).repeatedly());
      //driverXbox.rightBumper().onTrue(Commands.none());

      //Climber Buttons
      driverXbox.povRight().onTrue(Commands.runOnce(() -> { m_climberSubsystem.setPosition(ClimberConstants.kDeployPosition);}, m_climberSubsystem));
      driverXbox.povLeft().onTrue(Commands.runOnce(() -> { m_climberSubsystem.setPosition(ClimberConstants.kLiftPosition);}, m_climberSubsystem));
      driverXbox.povLeft().onTrue(Commands.runOnce(() -> {m_ElevatorSubsystem.setGoal(ElevatorConstants.position_Climb);}, m_ElevatorSubsystem));
      driverXbox.povUp().onTrue(Commands.runOnce(() -> { m_climberSubsystem.setPosition(ClimberConstants.kPreLatchPosition);}, m_climberSubsystem));
      driverXbox.povDown().onTrue(Commands.runOnce(() -> { m_climberSubsystem.lower();}, m_climberSubsystem));

      //Elevator Buttons
      driverXbox.a().onTrue(Commands.runOnce(() -> {m_ElevatorSubsystem.setGoal(ElevatorConstants.position_L2);}, m_ElevatorSubsystem));
      driverXbox.b().onTrue(Commands.runOnce(() -> {m_ElevatorSubsystem.setGoal(ElevatorConstants.position_L3);}, m_ElevatorSubsystem));
      driverXbox.x().onTrue(Commands.runOnce(() -> {m_ElevatorSubsystem.setGoal(ElevatorConstants.position_Home);}, m_ElevatorSubsystem));

      //Outtake Buttons
      driverXbox.rightTrigger().onTrue(Commands.runOnce(() -> {m_OuttakeSubsystem.manualShootCoral();}, m_OuttakeSubsystem));
      driverXbox.rightTrigger().onFalse(Commands.runOnce(() -> {m_OuttakeSubsystem.manualDisable();}, m_OuttakeSubsystem));
      driverXbox.leftTrigger().onTrue(Commands.runOnce(() -> {m_OuttakeSubsystem.manualFeedCoral();}, m_OuttakeSubsystem));
      driverXbox.leftTrigger().onFalse(Commands.runOnce(() -> {m_OuttakeSubsystem.manualDisable();}, m_OuttakeSubsystem));

      driverXbox.y().onTrue(Commands.runOnce(() -> {drivebase.toggleDriveSpeed();}, drivebase));
      driverXbox.back().onTrue(new InstantCommand(()->drivebase.zeroGyro()));
    

    }

    

// Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed,
// cancelling on release.
// m_driverController.b().whileTrue(m_armSubsystem.exampleMethodCommand());



  }

  public void configureDriveCommand() {

    //drivebase.setDefaultCommand(drivebase.driveFieldOriented(driveAngularVelocityBlue));
    
    // When PathPlanner is not used code below needed to invert controls for case of red aliance  
    var alliance = DriverStation.getAlliance();
            if (alliance.isPresent())
            {
              if (alliance.get() == DriverStation.Alliance.Red){
                drivebase.setDefaultCommand(drivebase.driveFieldOriented(driveAngularVelocityRed));
              }
              else{
                drivebase.setDefaultCommand(drivebase.driveFieldOriented(driveAngularVelocityBlue));
              }
            }
    
  }

  

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand()
  {
    var alliance = DriverStation.getAlliance();

    //return drivebase.driveToDistanceCommand(1.0, 3.0);


    //return drivebase.getAutonomousCommand("Strafe Auto");
     
    /*if(alliance.isPresent()){
      if(alliance.get() == DriverStation.Alliance.Red){
        return drivebase.driveToDistanceCommand(1.0, -3.0);
      } else {
        return drivebase.driveToDistanceCommand(1.0, -3.0);
      }
    }*/
  
    return drivebase.getAutonomousCommand("L1 Auto");




    
    // An example command will be run in autonomous
    //return drivebase.getAutonomousCommand("Leave Auto");
  }

  public void setMotorBrake(boolean brake)
  {
    drivebase.setMotorBrake(brake);
  }


  private void configurePathPlannerCommands() {
    NamedCommands.registerCommand("Enable Outtake", Commands.runOnce(m_OuttakeSubsystem::enable));
    NamedCommands.registerCommand("Disable Outtake", Commands.runOnce(m_OuttakeSubsystem::manualDisable));
    NamedCommands.registerCommand("Score", Commands.runOnce(m_OuttakeSubsystem::manualShootCoral));
    NamedCommands.registerCommand("Slow Score", Commands.runOnce(m_OuttakeSubsystem::manualFeedCoral));
    NamedCommands.registerCommand("Home", Commands.runOnce(m_ElevatorSubsystem::goToHome));
    NamedCommands.registerCommand("L2", Commands.runOnce(m_ElevatorSubsystem::goToL2));
    NamedCommands.registerCommand("L3", Commands.runOnce(m_ElevatorSubsystem::goToL3));
    NamedCommands.registerCommand("ResetGyro", Commands.runOnce(drivebase::zeroGyro));

  }

  private void configureCameraServer() {

    CameraServer.startAutomaticCapture();
    //HttpCamera httpCameraDrive = new HttpCamera("DriveCam", "http://photonvision.local:1184/stream.mjpg");
    HttpCamera httpCameraClimber = new HttpCamera("ClimbCam", "http://photonvision.local:1182/stream.mjpg");
    HttpCamera httpCameraReef = new HttpCamera("ReefCam", "http://photonvision.local:1186/stream.mjpg");

    httpCameraClimber.setConnectionStrategy(ConnectionStrategy.kKeepOpen);
    //httpCameraDrive.setConnectionStrategy(ConnectionStrategy.kKeepOpen);
    httpCameraReef.setConnectionStrategy(ConnectionStrategy.kKeepOpen);
    
  
    //Shuffleboard.getTab("Camera")
    //.add(httpCameraDrive);

    Shuffleboard.getTab("Camera")
    .add(httpCameraClimber);

    Shuffleboard.getTab("Camera")
    .add(httpCameraReef);

  }
}
