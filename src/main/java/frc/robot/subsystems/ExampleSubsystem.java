// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.sensors.CANCoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ExampleSubsystem extends SubsystemBase {
  XboxController controller = new XboxController(0);

  CANSparkMax driveMotor1 = new CANSparkMax(1, MotorType.kBrushless);
  CANSparkMax turnMotor2 = new CANSparkMax(2, MotorType.kBrushless);
  CANCoder encoder1 = new CANCoder(1);

  double offsetPosition;
  double offset;

  CANSparkMax driveMotor3 = new CANSparkMax(3, MotorType.kBrushless);
  CANSparkMax turnMotor4 = new CANSparkMax(4, MotorType.kBrushless);
  CANCoder encoder2 = new CANCoder(2);

  CANSparkMax driveMotor5 = new CANSparkMax(5, MotorType.kBrushless);
  CANSparkMax turnMotor6 = new CANSparkMax(6, MotorType.kBrushless);
  CANCoder encoder3 = new CANCoder(3);

  CANSparkMax driveMotor7 = new CANSparkMax(7, MotorType.kBrushless);
  CANSparkMax turnMotor8 = new CANSparkMax(8, MotorType.kBrushless);
  CANCoder encoder4 = new CANCoder(4);

  public ExampleSubsystem() {

    encoder1.configFactoryDefault();
    offset = encoder1.getAbsolutePosition();
  }

  PIDController controllerIDK = new PIDController(.0025, 0, 0);

  public void TurnToAngle(double targetAngle) {

    double theta = targetAngle - offsetPosition;

    // System.out.println(theta);

    turnMotor2.set(controllerIDK.calculate(-theta));
  }

  @Override
  public void periodic() {

    offsetPosition = encoder1.getAbsolutePosition() - offset;

    // Map offset position from range 0-360
    offsetPosition = (offsetPosition + 360) % 360;

    SmartDashboard.putNumber("Offset Angle", offsetPosition);
    SmartDashboard.putNumber("Actual Angle", encoder1.getAbsolutePosition());

    double x = controller.getRawAxis(0);
    double y = controller.getRawAxis(1);

    double epsilon = .9;

    if(Math.abs(x) + Math.abs(y) < epsilon)
    {
        return;
    }
    
    double angle = Math.toDegrees(Math.atan2(-y , x));

    // Map the angle to the correct thing
    angle = (angle + 270) % 360;


    TurnToAngle(angle);
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
