package testmechanisms;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="Intake + IntakeWrist", group = "testing")
public class IntakeTestOpmode extends LinearOpMode {
    MultipleTelemetry dashboardTelemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
    public IntakeTest intake = new IntakeTest();
    public IntakeWristTest intakeWrist = new IntakeWristTest()
    @Override
    public void waitForStart() {
        super.waitForStart();
    }
    @Override
    public void runOpMode() {
        intakeWrist.init(hardwareMap);
        intake.init(hardwareMap);

        waitForStart();

        while(opModeIsActive() && !isStopRequested()) {
            intake.Loop();
            intakeWrist.Loop();
            dashboardTelemetry.update();
        }
    }
}
