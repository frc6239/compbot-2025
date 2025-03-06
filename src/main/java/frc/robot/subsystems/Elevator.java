// RobotBuilder Version: 6.1
//
// This file was generated by RobotBuilder. It contains sections of
// code that are automatically generated and assigned by robotbuilder.
// These sections will be updated in the future when you export to
// Java from RobotBuilder. Do not put any code or make any change in
// the blocks indicating autogenerated code or it will be lost on an
// update. Deleting the comments indicating the section will prevent
// it from being updated in the future.

// ROBOTBUILDER TYPE: Subsystem.

package frc.robot.subsystems;


import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.LimitSwitchConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
// BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=IMPORTS
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.motorcontrol.Spark;

import frc.robot.Constants.ElevatorConstants;;




// END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=IMPORTS


/**
 *
 */
public class Elevator extends SubsystemBase {
    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTANTS

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTANTS

    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DECLARATIONS
private SparkMax m_LeftMotor;
private SparkMax m_RightMotor;
private LimitSwitchConfig m_levelOneLimitSwitch;
private LimitSwitchConfig m_levelTwoLimitSwitch;

private RelativeEncoder m_leftencoder;
private RelativeEncoder m_rightencoder;

public boolean m_elevatorEnabled = true;

//Limit switch for the max height
DigitalInput maxLimitSwitch = new DigitalInput(0);

  

   private SparkMaxConfig m_leftMotorConfig;
   private SparkMaxConfig m_rightMotorConfig;

   public boolean m_enabled;

   private final TrapezoidProfile.Constraints m_constraints =
      new TrapezoidProfile.Constraints(ElevatorConstants.kMaxVelocity, ElevatorConstants.kMaxAcceleration);
  private final ProfiledPIDController m_controller =
      new ProfiledPIDController(ElevatorConstants.kP, ElevatorConstants.kI, ElevatorConstants.kD, m_constraints, ElevatorConstants.kDt);
  private final ElevatorFeedforward m_feedforward = new ElevatorFeedforward(ElevatorConstants.kS, ElevatorConstants.kG, ElevatorConstants.kV);
    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DECLARATIONS
    
    /**
    *
    */
    public Elevator() {
        // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTRUCTORS

        



        m_enabled=true;
        boolean leftInverted=false;
        m_LeftMotor = new SparkMax(ElevatorConstants.kCANIdLeftMotor, MotorType.kBrushless);
        m_leftMotorConfig = new SparkMaxConfig();
        m_leftMotorConfig.inverted(leftInverted);
        m_leftMotorConfig.limitSwitch.forwardLimitSwitchEnabled(true);
        m_leftMotorConfig.limitSwitch.reverseLimitSwitchEnabled(true);
        m_LeftMotor.configure(m_leftMotorConfig, (SparkBase.ResetMode)null, (SparkBase.PersistMode)null);
        m_leftencoder = m_LeftMotor.getEncoder();

        m_RightMotor = new SparkMax(ElevatorConstants.kCANIdRightMotor, MotorType.kBrushless);
        m_rightMotorConfig = new SparkMaxConfig();
        m_rightMotorConfig.limitSwitch.forwardLimitSwitchEnabled(true);
        m_rightMotorConfig.limitSwitch.reverseLimitSwitchEnabled(true);
        m_RightMotor.configure(m_rightMotorConfig, (SparkBase.ResetMode)null, (SparkBase.PersistMode)null);
        m_rightencoder = m_RightMotor.getEncoder();
        m_rightMotorConfig.follow(ElevatorConstants.kCANIdLeftMotor,true);

        m_rightMotorConfig.encoder
        .positionConversionFactor((Math.PI * 1.5) / ElevatorConstants.kGearboxRatio)
        .velocityConversionFactor(1);
    
        m_leftMotorConfig.encoder
        .positionConversionFactor((Math.PI * 1.5) / ElevatorConstants.kGearboxRatio)
        .velocityConversionFactor(1);
 


        // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTRUCTORS
    }

     public void setGoal(double position){
        
        //Max height of the elevator is 25.5
        if (position >= 25.5) {
          m_controller.setGoal(25.5);
          System.out.println("Goal is above max height");
        }
        //Min height of the elevator is 0
        else if (position <= 0) {
          m_controller.setGoal(0);
          System.out.println("Goal is bellow min height");
        }
        else {
          m_controller.setGoal(position);
        }
     }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run

        //FIX ME Need to verify limit switch
        //Default logic for digitalIO is high also true
        if (!maxLimitSwitch.get()) {
          System.out.println("LimitSwitch toggle encoder is " + m_leftencoder.getPosition());
          //m_controller.setGoal(25.5);
        }

        m_LeftMotor.setVoltage(
            m_controller.calculate(m_leftencoder.getPosition())
                 + m_feedforward.calculate(m_controller.getSetpoint().velocity));


    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run when in simulation

    }

    public void enableElevator(boolean isEnabled) {
      m_elevatorEnabled = isEnabled;
  }

  public boolean isEnabled() {
    return m_elevatorEnabled;
  }

  public void disableElevator() {
    m_elevatorEnabled = false;
}

public void enableElevator() {
  m_elevatorEnabled = true;
}

     
    
      public void goToL2() {             
        setGoal(ElevatorConstants.position_L2);
         
      }
    
      public void goToL3() {        
        setGoal(ElevatorConstants.position_L3);
      }

      public void goToHome() {        
        setGoal(ElevatorConstants.position_Home);
      }

      public double getPosition() {
        return(m_rightencoder.getPosition());
      }
    
      public void resetEncoder() {
        m_rightencoder.setPosition(0);
    
      }
    // Put methods for controlling this subsystem
    // here. Call these from Commands.

}