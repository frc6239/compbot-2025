package frc.robot.subsystems;


import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.PWM;


public class LEDController extends SubsystemBase
{

private PWM blinkin;
private static final double orange = 0.65;
private static final double red_orange = 0.63;
private static final double green = 0.77;

private boolean m_enabled;



public LEDController() {

  m_enabled = true;
  blinkin = new PWM(0);

}

public void set_Orange() {
blinkin.setSpeed(orange);
}

public void set_RedOrange() {
blinkin.setSpeed(red_orange);
}

public void set_Green() {
  blinkin.setSpeed(green);
}

@Override
public void periodic() {
  // When vision is enabled we must manually update odometry in SwerveDrive
//blinkin.setSpeed(0.45);
}

public void enable() {
  m_enabled = true;
  set_Green();
}

public void disable() {
  m_enabled = false;
  set_RedOrange();
}

@Override
public void simulationPeriodic()
{
} 

}
