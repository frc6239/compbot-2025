package frc.robot.subsystems;


import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.PWM;


public class LEDController extends SubsystemBase
{

private PWM blinkin;
private static final double orange =0.65;
private static final double red_orange =0.63;



public LEDController() {
  blinkin = new PWM(0);

}

public void set_Orange() {
blinkin.setSpeed(orange);
}

public void set_Color() {
blinkin.setSpeed(red_orange);
}

@Override
public void periodic()
{
  // When vision is enabled we must manually update odometry in SwerveDrive
blinkin.setSpeed(0.45);
}

@Override
public void simulationPeriodic()
{
} 

}