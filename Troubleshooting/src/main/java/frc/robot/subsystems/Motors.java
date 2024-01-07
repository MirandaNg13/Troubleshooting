// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.ctre.phoenix.motorcontrol.ControlMode;

public class Motors extends SubsystemBase {
  /** Creates a new Subsystem for troubleshooting motors. */

  private final CANSparkMax m_leftfrontDrive;
  private final CANSparkMax m_leftbackDrive;
  private final CANSparkMax m_rightfrontDrive;
  private final CANSparkMax m_rightbackDrive;

  private final VictorSPX m_leftfrontTurn;
  private final VictorSPX m_leftbackTurn;
  private final VictorSPX m_rightfrontTurn;
  private final VictorSPX m_rightbackTurn;

  public Motors() {
    m_leftfrontDrive = new CANSparkMax(1, MotorType.kBrushless);
    m_leftbackDrive = new CANSparkMax(2, MotorType.kBrushless);
    m_rightfrontDrive = new CANSparkMax(3, MotorType.kBrushless);
    m_rightbackDrive = new CANSparkMax(4, MotorType.kBrushless);

    m_leftfrontTurn = new VictorSPX(5);
    m_leftbackTurn = new VictorSPX(6);
    m_rightfrontTurn = new VictorSPX(7);
    m_rightbackTurn = new VictorSPX(8);

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
    m_leftfrontDrive.set(0.25);
    m_leftbackDrive.set(0.25);
    m_rightfrontDrive.set(0.25);
    m_rightbackDrive.set(0.25);
  }
  
  /**
   * An action to return movement from the drive motors.
   * All motors will be set to the same speed
   * @return
   */
  public void TurnForward() {
    m_leftfrontTurn.set(ControlMode.PercentOutput, 0.25);
    m_leftbackTurn.set(ControlMode.PercentOutput, 0.25);
    m_rightfrontTurn.set(ControlMode.PercentOutput, 0.25);
    m_rightbackTurn.set(ControlMode.PercentOutput, 0.25);
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
