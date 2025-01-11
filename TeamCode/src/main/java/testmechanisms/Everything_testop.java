package testmechanisms;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Everything", group = "testing")
public class Everything_testop extends LinearOpMode {
    MultipleTelemetry dashboardTelemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
    public BarTest bar = new BarTest();
    public WristTest wrist = new WristTest();
    public ClawTest claw = new ClawTest();
    public SlidesTest slides = new SlidesTest();
    public ExtendoTest extendo = new ExtendoTest();
    public IntakeTest intake = new IntakeTest();
    public IntakeWristTest intakeWrist = new IntakeWristTest();

    @Override
    public void waitForStart() {
        super.waitForStart();
    }

    @Override
    public void runOpMode() {
        bar.init(hardwareMap);
        claw.init(hardwareMap);
        wrist.init(hardwareMap);
        slides.init(hardwareMap);
        extendo.init(hardwareMap);
        intakeWrist.init(hardwareMap);
        intake.init(hardwareMap);
        waitForStart();

        while(opModeIsActive() && !isStopRequested()) {
            bar.Loop();
            claw.Loop();
            wrist.Loop();
            slides.Loop();
            extendo.Loop();
            intakeWrist.Loop();
            intake.Loop();
//            dashboardTelemetry.addData("Last commanded position:", "%.3f", bar.POS);
            dashboardTelemetry.update();
        }
    }
}
