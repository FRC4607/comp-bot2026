// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import static edu.wpi.first.units.Units.Inches;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveRequest.FieldCentric;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class WheelRadiusCalibration extends Command {

  private double m_angle;
  private double m_driveStartPosition;
  private CommandSwerveDrivetrain m_drivetrain;
  private SwerveRequest.FieldCentric m_drive;

  /** This command will spin the robot 360 degrees.
   * To calculate wheel radius:
   * Find a drive motor in Tuner X.
   * Run this command, and track the rotations:
   * Convert motor rotations to wheel rotations using the swerve gear ratio:
   * 
   * Find the distace of the wheel to the center of the robot:
   * Multiply this by 6.283 for an approximate circumference:
   * 
   * Divide the circumference by the number of wheel rotations to get wheel circumference:
   * Divide wheel circumference by 6.283 to get wheel radius:
   */
  public WheelRadiusCalibration(CommandSwerveDrivetrain drivetrain, SwerveRequest.FieldCentric drive) {
    m_drivetrain = drivetrain;
    m_drive = drive;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_angle = m_drivetrain.getPigeon2().getYaw().getValueAsDouble();
    m_driveStartPosition = m_drivetrain.getModule(1).getDriveMotor().getPosition().getValueAsDouble();

    m_drivetrain.applyRequest(() -> m_drive.withVelocityX(0)
      .withVelocityY(0)
      .withRotationalRate(0.5));

    System.out.println("Started");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
    System.out.println("going at rate: " + m_drive.RotationalRate);
    System.out.println(m_angle - m_drivetrain.getPigeon2().getYaw().getValueAsDouble());

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if (!interrupted) {
      m_drivetrain.applyRequest(() -> m_drive.withVelocityX(0)
      .withVelocityY(0)
      .withRotationalRate(0));

      double circumference = 
      // Circumference of the path of the drive wheel
      ((Math.abs(m_drivetrain.getPigeon2().getYaw().getValueAsDouble() - m_angle) / 360) *
      (Math.PI * 2 * Math.sqrt(
        Math.abs(MathUtil.copyDirectionPow(9.625 /* distance between FL to FR swerve modules divided by 2 in inches */, 2))
         + Math.abs(MathUtil.copyDirectionPow(9.625 /* distance between FL to BL swerve modules divided by 2 in inches */, 2)))))

      // Divided by wheel rotations
      /  (Math.abs(m_driveStartPosition - m_drivetrain.getModule(1).getDriveMotor().getPosition().getValueAsDouble())
      / TunerConstants.kDriveGearRatio);

      System.out.println("Wheel Circumference: " + circumference);
      System.out.println("Wheel Radius" + (circumference / (Math.PI * 2)));
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (m_angle - m_drivetrain.getPigeon2().getYaw().getValueAsDouble()) > 1440;
  }
}
