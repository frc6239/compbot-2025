// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.swervedrive2.commands.drivebase;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Constants.Drivebase.ModuleLocations;
import frc.robot.subsystems.swervedrive2.SwerveBase;
import frc.robot.subsystems.swervedrive2.SwerveController;
import java.util.function.DoubleSupplier;

/**
 * An example command that uses an example subsystem.
 */
public class AbsoluteDrive extends CommandBase
{

  private final SwerveBase     swerve;
  private final DoubleSupplier vX, vY;
  private final DoubleSupplier headingHorizontal, headingVertical;
  private final boolean          isOpenLoop;
  private       SwerveController swerveController;

  /**
   * Used to drive a swerve robot in full field-centric mode.  vX and vY supply translation inputs, where x is
   * torwards/away from alliance wall and y is left/right. headingHorzontal and headingVertical are the Cartesian
   * coordinates from which the robot's angle will be derived— they will be converted to a polar angle, which the robot
   * will rotate to.
   *
   * @param swerve            The swerve drivebase subsystem.
   * @param vX                DoubleSupplier that supplies the x-translation joystick input.  Should be in the range -1
   *                          to 1 with deadband already accounted for.  Positive X is away from the alliance wall.
   * @param vY                DoubleSupplier that supplies the y-translation joystick input.  Should be in the range -1
   *                          to 1 with deadband already accounted for.  Positive Y is towards the left wall when
   *                          looking through the driver station glass.
   * @param headingHorizontal DoubleSupplier that supplies the horizontal component of the robot's heading angle. In the
   *                          robot coordinate system, this is along the same axis as vY. Should range from -1 to 1 with
   *                          no deadband.  Positive is towards the left wall when looking through the driver station
   *                          glass.
   * @param headingVertical   DoubleSupplier that supplies the vertical component of the robot's heading angle.  In the
   *                          robot coordinate system, this is along the same axis as vX.  Should range from -1 to 1
   *                          with no deadband. Positive is away from the alliance wall.
   */
  public AbsoluteDrive(SwerveBase swerve, DoubleSupplier vX, DoubleSupplier vY, DoubleSupplier headingHorizontal,
                       DoubleSupplier headingVertical, boolean isOpenLoop)
  {
    this.swerve = swerve;
    this.vX = vX;
    this.vY = vY;
    this.headingHorizontal = headingHorizontal;
    this.headingVertical = headingVertical;
    this.isOpenLoop = isOpenLoop;

    addRequirements(swerve);
  }

  @Override
  public void initialize()
  {
    swerveController = new SwerveController();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute()
  {
    // Configure the last angle to be reset if the swerve drive had the angle reset.
    swerveController.configureLastAngle(swerve);

    // Get the desired chassis speeds based on a 2 joystick module.
    ChassisSpeeds desiredSpeeds = swerveController.getTargetSpeeds(vX.getAsDouble(), vY.getAsDouble(),
                                                                   headingHorizontal.getAsDouble(),
                                                                   headingVertical.getAsDouble(),
                                                                   swerve.getYaw().getRadians());

    // Limit velocity to prevent tippy
    Translation2d translation = SwerveController.getTranslation2d(desiredSpeeds);
    translation = limitVelocity(translation);
    SmartDashboard.putNumber("LimitedTranslation", translation.getX());
    SmartDashboard.putString("Translation", translation.toString());

    // Make the robot move
    swerve.drive(translation, desiredSpeeds.omegaRadiansPerSecond, true, isOpenLoop);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted)
  {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished()
  {
    return false;
  }

  /**
   * Calculates the maximum acceleration allowed in a direction without tipping the robot. Reads arm position from
   * NetworkTables and is passed the direction in question.
   *
   * @param angle The direction in which to calculate max acceleration, as a Rotation2d. Note that this is
   *              robot-relative.
   * @return Maximum acceleration allowed in the robot direction.
   */
  private double calcMaxAccel(Rotation2d angle)
  {
    // Get the position of the arm from NetworkTables 
    double armHeight    = SmartDashboard.getNumber("armHeight", Constants.dummyArmHieght);
    double armExtension = SmartDashboard.getNumber("armExtension", Constants.dummyArmX);

    double xMoment =
        (armExtension * Constants.MANIPULATOR_MASS) + (Constants.CHASSIS_CG.getX() * Constants.CHASSIS_MASS);
    double yMoment =
        (Constants.ARM_Y_POS * Constants.MANIPULATOR_MASS) + (Constants.CHASSIS_CG.getY() * Constants.CHASSIS_MASS);
    // Calculate the vertical mass moment using the floor as the datum.  This will be used later to calculate max
    // acceleration
    double zMoment =
        (armHeight * Constants.MANIPULATOR_MASS)
        + (Constants.CHASSIS_CG.getZ() * (Constants.CHASSIS_MASS));

    Translation3d robotCG      = new Translation3d(xMoment, yMoment, zMoment).div(Constants.ROBOT_MASS);
    Translation2d horizontalCG = robotCG.toTranslation2d();

    Translation2d projectedHorizontalCg = new Translation2d(
        (angle.getSin() * angle.getCos() * horizontalCG.getY()) + (Math.pow(angle.getCos(), 2) * horizontalCG.getX()),
        (angle.getSin() * angle.getCos() * horizontalCG.getX()) + (Math.pow(angle.getSin(), 2) * horizontalCG.getY())
    );

    // Projects the edge of the wheelbase onto the direction line.  Assumes the wheelbase is rectangular.
    // Because a line is being projected, rather than a point, one of the coordinates of the projected point is
    // already known.
    Translation2d projectedWheelbaseEdge;
    double        angDeg = angle.getDegrees();
    if (angDeg <= 45 && angDeg >= -45)
    {
      projectedWheelbaseEdge = new Translation2d(
          ModuleLocations.FRONT_LEFT_X,
          ModuleLocations.FRONT_LEFT_X * angle.getTan());
    } else if (135 >= angDeg && angDeg > 45)
    {
      projectedWheelbaseEdge = new Translation2d(
          ModuleLocations.FRONT_LEFT_Y / angle.getTan(),
          ModuleLocations.FRONT_LEFT_Y);
    } else if (-135 <= angDeg && angDeg < -45)
    {
      projectedWheelbaseEdge = new Translation2d(
          ModuleLocations.FRONT_RIGHT_Y / angle.getTan(),
          ModuleLocations.FRONT_RIGHT_Y);
    } else
    {
      projectedWheelbaseEdge = new Translation2d(
          ModuleLocations.BACK_LEFT_X,
          ModuleLocations.BACK_LEFT_X * angle.getTan());
    }

    double horizontalDistance = projectedHorizontalCg.plus(projectedWheelbaseEdge).getNorm();
    double maxAccel           = Constants.GRAVITY * horizontalDistance / robotCG.getZ();

    SmartDashboard.putNumber("calcMaxAccel", maxAccel);
    return maxAccel;
  }

  /**
   * Limits a commanded velocity to prevent exceeding the maximum acceleration given by
   * {@link AbsoluteDrive#calcMaxAccel(Rotation2d)}.  Note that this takes and returns field-relative velocities.
   *
   * @param commandedVelocity The desired velocity
   * @return The limited velocity.  This is either the commanded velocity, if attainable, or the closest attainable
   * velocity.
   */
  private Translation2d limitVelocity(Translation2d commandedVelocity)
  {
    // Get the robot's current field-relative velocity
    Translation2d currentVelocity = new Translation2d(
        swerve.getFieldVelocity().vxMetersPerSecond,
        swerve.getFieldVelocity().vyMetersPerSecond);
    SmartDashboard.putNumber("currentVelocity", currentVelocity.getX());

    // Calculate the commanded change in velocity by subtracting current velocity
    // from commanded velocity
    Translation2d deltaV = commandedVelocity.minus(currentVelocity);
    SmartDashboard.putNumber("deltaV", deltaV.getX());

    // Creates an acceleration vector with the direction of delta V and a magnitude
    // of the maximum allowed acceleration in that direction 
    Translation2d maxAccel = new Translation2d(
        calcMaxAccel(deltaV
                         // Rotates the velocity vector to convert from field-relative to robot-relative
                         .rotateBy(swerve.getPose().getRotation().unaryMinus())
                         .getAngle()),
        deltaV.getAngle());

    // Calculate the maximum achievable velocity by the next loop cycle.
    // delta V = Vf - Vi = at
    Translation2d maxAchievableDeltaVelocity = maxAccel.times(Constants.LOOP_TIME);

    if (deltaV.getNorm() > maxAchievableDeltaVelocity.getNorm())
    {
      return maxAchievableDeltaVelocity.plus(currentVelocity);
    } else
    {
      // If the commanded velocity is attainable, use that.
      return commandedVelocity;
    }
  }
}
