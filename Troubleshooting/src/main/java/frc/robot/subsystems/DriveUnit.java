// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.ADIS16448_IMU;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.ctre.phoenix.motorcontrol.ControlMode;

public class DriveUnit extends SubsystemBase {
  /** Creates a new Subsystem for troubleshooting motors. */

  private final CANSparkMax m_driveTS;
  private final RelativeEncoder m_driveEncoder;
  private final VictorSPX m_turnTS;
  private final Encoder m_turnEncoder;
  private final ADIS16448_IMU m_imu;

  public DriveUnit() {
    m_driveTS = new CANSparkMax(1, MotorType.kBrushless);
    m_driveEncoder = m_driveTS.getEncoder();
    m_turnTS = new VictorSPX(5);
    m_turnEncoder = new Encoder(6, 7);
    m_imu = new ADIS16448_IMU();

  }

  /**
   * Example command factory method.
   *
   * @return a command
   */
  public CommandBase exampleMethodCommand() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          /* one-time action goes here */
        });
  }

  /**
   * An action to return movement from the drive motors.
   * All motors will be set to the same speed
   * @return
   */
  public void DriveForward() {
    m_driveTS.set(0.25);
    
  }
  
  /**
   * An action to return movement from the drive motors.
   * All motors will be set to the same speed
   * @return
   */
  public void TurnForward() {
        m_turnTS.set(ControlMode.PercentOutput, 0.25);
  }

  public double DriveEncoderPosition() {
    double encoderpos = m_driveEncoder.getPosition();
    return encoderpos;
  }

  public double TurnEncoderCount() {
    double encodercount = m_turnEncoder.get();
    return encodercount;
  }

  public double getAngle() {
    double angle = m_imu.getAngle();
    return angle;
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
}
