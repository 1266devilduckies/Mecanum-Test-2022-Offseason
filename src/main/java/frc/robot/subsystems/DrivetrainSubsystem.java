package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.MecanumDriveKinematics;
import edu.wpi.first.math.kinematics.MecanumDriveWheelSpeeds;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class DrivetrainSubsystem extends SubsystemBase {

  private final WPI_VictorSPX motorBR;
  private final WPI_VictorSPX motorFR;
  private final WPI_VictorSPX motorFL;
  private final WPI_VictorSPX motorBL;
  private final MecanumDriveKinematics kinematics = new MecanumDriveKinematics(
    Constants.FLWheelOffsetMeters, Constants.FRWheelOffsetMeters,
    Constants.BLWheelOffsetMeters, Constants.BRWheelOffsetMeters
  );
  private final SimpleMotorFeedforward drivetrainFeedForwardCalculator = new SimpleMotorFeedforward(
    Constants.kS_drivetrain,
    Constants.kV_drivetrain,
    Constants.kA_drivetrain
    );
  
  public DrivetrainSubsystem() {
    motorBR = new WPI_VictorSPX(Constants.CANID_motorBR);
    motorFR = new WPI_VictorSPX(Constants.CANID_motorFR);
    motorFL = new WPI_VictorSPX(Constants.CANID_motorFL);
    motorBL = new WPI_VictorSPX(Constants.CANID_motorBL);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    double strafe = RobotContainer.driverJoystick.getRawAxis(0);
    double move = -RobotContainer.driverJoystick.getRawAxis(1);
    double turn = RobotContainer.driverJoystick.getRawAxis(2);

    //robot centric
    ChassisSpeeds movementState = new ChassisSpeeds(move, -strafe, turn);

    MecanumDriveWheelSpeeds wheelSpeeds = kinematics.toWheelSpeeds(movementState);

    motorBR.set(ControlMode.PercentOutput, getPercentOutputFromChassisSpeed(wheelSpeeds.rearRightMetersPerSecond));
    motorBL.set(ControlMode.PercentOutput, getPercentOutputFromChassisSpeed(wheelSpeeds.rearLeftMetersPerSecond));
    motorFR.set(ControlMode.PercentOutput, getPercentOutputFromChassisSpeed(wheelSpeeds.frontRightMetersPerSecond));
    motorFL.set(ControlMode.PercentOutput, getPercentOutputFromChassisSpeed(wheelSpeeds.frontLeftMetersPerSecond));
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

  //method assumes 4 inch diameter meccanum wheel
  private double getPercentOutputFromChassisSpeed(double metersPerSecond) {
    //gains measured in rps
    double rps = (4*Math.PI)/Math.abs(metersPerSecond);
    double voltageOutput = Math.signum(metersPerSecond)*drivetrainFeedForwardCalculator.calculate(rps);
    //return voltageOutput/RobotController.getBatteryVoltage();

    //assume a meter a second is max voltage
    System.out.println(metersPerSecond);
    return metersPerSecond;
  }
}
