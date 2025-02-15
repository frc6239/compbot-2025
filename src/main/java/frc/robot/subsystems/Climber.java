// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ClimberConstants;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;

public class Climber extends SubsystemBase {

  /**
   * Instance variables for class
   */

  private SparkMax m_motor;
  private SparkMaxConfig m_motorConfig;
  private SparkClosedLoopController m_closedLoopController;
  private RelativeEncoder m_encoder;
  private boolean m_enabled;

  /** Creates a new ArmSubsystem. */
  public Climber() {
    m_motor = new SparkMax(ClimberConstants.kCANidMotor, MotorType.kBrushless);
    m_closedLoopController = m_motor.getClosedLoopController();
    m_encoder = m_motor.getEncoder();
this.Enabled();
   // m_enabled = true;

    /*
     * Create a new SPARK MAX configuration object. This will store the
     * configuration parameters for the SPARK MAX that we will set below.
     */
    m_motorConfig = new SparkMaxConfig();

    /*
     * Configure the encoder. For this specific example, we are using the
     * integrated encoder of the NEO, and we don't need to configure it. If
     * needed, we can adjust values like the position or velocity conversion
     * factors.
     */
    m_motorConfig.encoder
        .positionConversionFactor(1)
        .velocityConversionFactor(1);

    /*
     * Configure the closed loop controller. We want to make sure we set the
     * feedback sensor as the primary encoder.
     */
    m_motorConfig.closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        // Set PID values for position control. We don't need to pass a closed
        // loop slot, as it will default to slot 0.
        .p(ClimberConstants.kP)
        .i(0)
        .d(0)
        .outputRange(-1, 1);

    m_motorConfig.closedLoop.maxMotion
        
        // Set MAXMotion parameters for position control. We don't need to pass
        // a closed loop slot, as it will default to slot 0.
        .maxVelocity(ClimberConstants.kMotorRpm)
        .maxAcceleration(ClimberConstants.kMotorRpm)
        .allowedClosedLoopError(1);

    /*
     * Apply the configuration to the SPARK MAX.
     *
     * kResetSafeParameters is used to get the SPARK MAX to a known state. This
     * is useful in case the SPARK MAX is replaced.
     *
     * kPersistParameters is used to ensure the configuration is not lost when
     * the SPARK MAX loses power. This is useful for power cycles that may occur
     * mid-operation.
     */
    m_motor.configure(m_motorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

    // Initialize dashboard values
    SmartDashboard.setDefaultNumber("Arm Increment", ClimberConstants.kArmIncrement);
    
    SmartDashboard.setDefaultNumber("Target Position", ClimberConstants.kMinPosition);
    SmartDashboard.setDefaultBoolean("Arm Enabled", true);
    SmartDashboard.setDefaultBoolean("Reset Encoder", false);

  }

  public void Enabled() {
    m_enabled = true;
  }
  public void disabled() {
    m_enabled = false;
  }

  /**
   * Example command factory method.
   *
   * @return a command
   */
  public Command exampleMethodCommand() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          /* one-time action goes here */
        });
  }

  /**
   * An example method querying a boolean state of the subsystem (for example, a digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
  public boolean exampleCondition() {
    // Query some boolean state, such as a digital sensor.
    return false;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

  public void setPosition(double targetPosition) {
    if (m_enabled) {
      System.out.println(" In set position " + targetPosition);
       m_closedLoopController.setReference(targetPosition, ControlType.kMAXMotionPositionControl,
          ClosedLoopSlot.kSlot0);
    } else {
      // set an alert
    }
  }

  public void lower() {
    double currentPosition = m_encoder.getPosition();

    if (currentPosition < (ClimberConstants.kMaxPosition - ClimberConstants.kArmIncrement)) {
      setPosition(currentPosition + ClimberConstants.kArmIncrement);
    }
  }

  public void raise() {
    double currentPosition = m_encoder.getPosition();

    if (currentPosition > (ClimberConstants.kMaxPosition + ClimberConstants.kArmIncrement)) {
      setPosition(currentPosition + ClimberConstants.kArmIncrement);
    }
  }

  public void deploy() {
    if (m_enabled) {
      setPosition(ClimberConstants.kDeployPosition);
    } else {
      // send an alert
    }
  }

  public void lift() {
    if (m_enabled) {
      setPosition(ClimberConstants.kLiftPosition);
    } else {
      // send an alert
    }
  }

  public void retract() {
    if (m_enabled) {
      setPosition(ClimberConstants.kMinPosition);
    } else { 
      // send an alert
    }
  }

  public void lowerByIncrement() {
    if (m_enabled) {
      // check if enough space left to increment if so lower
    }

  }

  public void raiseByIncrement() {
    if (m_enabled) {
      // check if enoguht space left to raise if so increase
    }
  }

  public double getPosition() {
    return(m_encoder.getPosition());
  }

  public double getVelocity() {
    return(m_encoder.getVelocity());
  }

  public void resetEncoder() {
    m_encoder.setPosition(0);

  }
}
