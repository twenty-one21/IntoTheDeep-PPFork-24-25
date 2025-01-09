package mechanisms;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class Drivetrain_fieldcentric {
    DcMotor rightFront;
    DcMotor leftFront;
    DcMotor rightRear;
    DcMotor leftRear;
    IMU imu;
    double minSpeed = -0.8;
    double maxSpeed = 0.8;
    double MICROSPEED = 0.3;
    boolean slowMode = false;
    public void init(HardwareMap hm){
        rightFront = hm.get(DcMotor.class, "rightFront");
        leftFront = hm.get(DcMotor.class, "leftFront");
        rightRear = hm.get(DcMotor.class, "rightRear");
        leftRear = hm.get(DcMotor.class, "leftRear");
        rightFront.setDirection(DcMotorSimple.Direction.REVERSE);
        rightRear.setDirection(DcMotorSimple.Direction.REVERSE);
        leftFront.setDirection(DcMotorSimple.Direction.FORWARD);
        leftRear.setDirection(DcMotorSimple.Direction.FORWARD);
        IMU imu = hm.get(IMU.class, "imu");
        // Adjust the orientation parameters to match your robot
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP));
        // Without this, the REV Hub's orientation is assumed to be logo up / USB forward
        imu.initialize(parameters);
    }

    public void slowModeON(){
        slowMode = true;
    }

    public void slowModeOFF(){
        slowMode = false;
    }



    public void Loop(Gamepad gp1, Gamepad gp2){
        double y = -gp1.left_stick_y; // Remember, Y stick value is reversed
        double x = gp1.left_stick_x;
        double rx = gp1.right_stick_x;

        // This button choice was made so that it is hard to hit on accident,
        // it can be freely changed based on preference.
        // The equivalent button is start on Xbox-style controllers.
        if (gp1.options) {
            imu.resetYaw();
        }

        double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

        // Rotate the movement direction counter to the bot's rotation
        double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
        double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

        rotX = rotX * 1.1;  // Counteract imperfect strafing

        // Denominator is the largest motor power (absolute value) or 1
        // This ensures all the powers maintain the same ratio,
        // but only if at least one is out of the range [-1, 1]
        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
        double frontLeftPower = (rotY + rotX + rx) / denominator;
        double backLeftPower = (rotY - rotX + rx) / denominator;
        double frontRightPower = (rotY - rotX - rx) / denominator;
        double backRightPower = (rotY + rotX - rx) / denominator;



        rightFront.setPower(Range.clip(frontRightPower, minSpeed, maxSpeed));
        rightRear.setPower(Range.clip(backRightPower, minSpeed, maxSpeed));
        leftFront.setPower(Range.clip(frontLeftPower, minSpeed, maxSpeed));
        leftRear.setPower(Range.clip(backLeftPower, minSpeed, maxSpeed));
    }
}
