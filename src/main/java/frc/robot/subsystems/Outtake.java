package frc.robot.subsystems;


import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.LimitSwitchConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.motorcontrol.Spark;

import frc.robot.Constants.OuttakeConstants;;

public class Outtake extends SubsystemBase{
//Instance Variables

    private DigitalInput outtakeLimitSwitch = new DigitalInput(1);
    DigitalInput LEDTransmiter = new DigitalInput(2);
    private SparkMax m_leftMotor;
    private SparkMax m_rightMotor;
    private LimitSwitchConfig m_levelOneLimitSwitch;
    private LimitSwitchConfig m_levelTwoLimitSwitch;

    private RelativeEncoder m_leftencoder;
    private RelativeEncoder m_rightencoder;

    private SparkMaxConfig m_leftMotorConfig;
    private SparkMaxConfig m_rightMotorConfig;

    private boolean m_running;

    private double m_speed;

    private boolean m_manualControl;

    public Outtake(){

        m_manualControl = false;
        m_running=true;
        boolean leftInverted=true;
        m_leftMotor = new SparkMax(OuttakeConstants.kCANIdLeftMotor, MotorType.kBrushless);
        m_leftMotorConfig = new SparkMaxConfig();
        m_leftMotorConfig.inverted(leftInverted);
        m_leftMotorConfig.limitSwitch.forwardLimitSwitchEnabled(true);
        m_leftMotorConfig.limitSwitch.reverseLimitSwitchEnabled(true);
        m_leftMotor.configure(m_leftMotorConfig, (SparkBase.ResetMode)null, (SparkBase.PersistMode)null);
        m_leftencoder = m_leftMotor.getEncoder();
        m_leftMotorConfig.idleMode(IdleMode.kBrake);

        m_rightMotor = new SparkMax(OuttakeConstants.kCANIdRightMotor, MotorType.kBrushless);
        m_rightMotorConfig = new SparkMaxConfig();
        m_rightMotorConfig.limitSwitch.forwardLimitSwitchEnabled(false);
        m_rightMotorConfig.limitSwitch.reverseLimitSwitchEnabled(false);
        m_rightMotor.configure(m_rightMotorConfig, (SparkBase.ResetMode)null, (SparkBase.PersistMode)null);
        m_rightencoder = m_rightMotor.getEncoder();
        m_rightMotorConfig.follow(OuttakeConstants.kCANIdLeftMotor,!leftInverted);
        m_rightMotorConfig.idleMode(IdleMode.kBrake);

        m_rightMotorConfig.encoder
        .positionConversionFactor(1)
        .velocityConversionFactor(1);
    
        m_leftMotorConfig.encoder
        .positionConversionFactor(1)
        .velocityConversionFactor(1);

    }
    public void run() {
        m_leftMotor.set(m_speed);
        m_rightMotor.set(m_speed);
      }
    
      public void stop() {
        m_leftMotor.set(0);
        m_rightMotor.set(0);
      }
    
      public void invert() {
        m_speed = -m_speed;
      }
    
      public boolean beamBreakCleared() {

        return outtakeLimitSwitch.get();  // Return true if beam is cleared (no game piece), false if blocked
    }


      @Override
      public void periodic() {
        // This method will be called once per scheduler run

        //System.out.println("sensor value " + outtakeLimitSwitch.get());
        if ((outtakeLimitSwitch.get() == false) && (m_manualControl == false)){
          feedCoral();
          //System.out.println("Feeding");
         
        }
        
        if ((outtakeLimitSwitch.get() == true) && (m_manualControl == false)){
          disable();
        }

        if (m_running){
          run();
        } else { 
          stop();
        }
      }
    
      public void enable () {
        m_running = true;
      }
    
      public void disable () {
        m_running = false;
      }
    
      public void setspeed(double speed){
        m_speed = speed;
      }
    
      public double getspeed(){
        return m_speed;
      }
    
      public void shootCoral() {
          m_speed = OuttakeConstants.kCoralShootSpeed;
          enable();
        }
      
    
      public void feedCoral() {
        m_speed = OuttakeConstants.kCoralFeedSpeed;
          enable();
        }

      public void manualShootCoral() {
        m_manualControl = true;
        shootCoral();
      }

      public void manualFeedCoral() {
        m_manualControl = true;
        feedCoral();
      }

      public void manualDisable() {
        m_manualControl = false;
        disable();
      }
        
      
    
    
}
