package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class DrivetrainSubsystem extends SubsystemBase {
        
        private final WPI_VictorSPX motorBR;
        private final WPI_VictorSPX motorFR;
        private final WPI_VictorSPX motorFL;
        private final WPI_VictorSPX motorBL;
        private final MecanumDrive drivetrain;
        
        public DrivetrainSubsystem(){
                motorBR = new WPI_VictorSPX(Constants.CANID_motorBR);
                motorFR = new WPI_VictorSPX(Constants.CANID_motorFR);
                motorFL = new WPI_VictorSPX(Constants.CANID_motorFL);
                motorBL = new WPI_VictorSPX(Constants.CANID_motorBL);
                drivetrain = new MecanumDrive(motorFL, motorBL, motorFR, motorBR);
                motorBR.setInverted(true);
                motorFR.setInverted(true);
        }

        @Override
        public void periodic(){
                double strafe = RobotContainer.driverJoystick.getRawAxis(0);
                double move = -RobotContainer.driverJoystick.getRawAxis(1);
                double turn = RobotContainer.driverJoystick.getRawAxis(2);

                drivetrain.driveCartesian(move, strafe, turn);
        }

}
