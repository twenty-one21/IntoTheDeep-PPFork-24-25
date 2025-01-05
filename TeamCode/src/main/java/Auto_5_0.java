import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.Path;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.Constants;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import mechanisms.Bar;
import mechanisms.Claw;
import mechanisms.Extendo;
import mechanisms.Intake;
import mechanisms.IntakeWrist;
import mechanisms.Slides;
import mechanisms.Wrist;
import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;

@Autonomous(name = "5+0", group = "Auto")
public class Auto_5_0 extends OpMode {
    private Bar bar;
    private Claw claw;
    private Extendo extendo;
    private Intake intake;
    private IntakeWrist intakeWrist;
    private Slides slides;
    private Wrist wrist;


    private Follower follower;
    private Timer pathTime, totalTime;
    private int pathState = 0;


    //Clip x: 41.29411764705882
    private final Pose STARTPOSE = new Pose(6.75,60, Math.toRadians(0));
    private final Pose PRELOADPOSE = new Pose(41.29411764705882,64.5,Math.toRadians(0));
    private final Pose PREPARE1POSE = new Pose(37.513786764705884,42.88235294117648,Math.toRadians(-47));
    private final Pose PREPARE1CONTROL = new Pose(19.235294117647058,57.88235294117647);
    private final Pose PUSH1POSE = new Pose(25.513786764705884,43.235294117647065,Math.toRadians(-121));
    private final Pose PREPARE2POSE = new Pose(37.513786764705884,32.88235,Math.toRadians(-47));
    private final Pose PUSH2POSE = new Pose(25.513786764705884,32.88235,Math.toRadians(-121));
    private final Pose PREPARE3POSE = new Pose(37.513786764705884,22.88235,Math.toRadians(-47));
    private final Pose PUSH3POSE = new Pose(10.337316176470589,34.05882352941177,Math.toRadians(180));
    private final Pose WALLPOSE = new Pose(6.75,34.05882352941177,Math.toRadians(180));
    private final Pose SCORE1POSE = new Pose(41.29411764705882,66.5,Math.toRadians(0));
    private final Pose SCORE2POSE = new Pose(41.29411764705882,68.5,Math.toRadians(0));
    private final Pose SCORE3POSE = new Pose(41.29411764705882,70.5,Math.toRadians(0));
    private final Pose SCORE4POSE = new Pose(41.29411764705882,72.5,Math.toRadians(0));
    private final Pose PARKPOSE = new Pose(8.396139705882353,6.882352941176478,Math.toRadians(0));
    private final Pose PARKCONTROL = new Pose(7.235294117647059,70.23529411764707);


    private Path scorePreload; //Define paths

    private void buildPaths() {

    }
    private void updatePaths() {
        switch (pathState) {
            //Auto goes here

        }
    }
    private void setPathState(int n) {
        pathState = n;
        pathTime.resetTimer();
    }
    @Override
    public void init() {
        pathTime = new Timer();
        totalTime = new Timer();
        totalTime.resetTimer();
        Constants.setConstants(FConstants.class, LConstants.class);
        follower = new Follower(hardwareMap);
        follower.setStartingPose(STARTPOSE);
        buildPaths();

        bar = new Bar();
        claw = new Claw();
        extendo = new Extendo();
        intake = new Intake();
        intakeWrist = new IntakeWrist();
        slides = new Slides();
        wrist = new Wrist();

        bar.init(hardwareMap);
        claw.init(hardwareMap);
        extendo.init(hardwareMap);
        intake.init(hardwareMap);
        intakeWrist.init(hardwareMap);
        slides.init(hardwareMap);
        wrist.init(hardwareMap);

        claw.setState(Claw.ClawState.CLOSE);
        bar.setState(Bar.BarState.NEUTRAL);
        wrist.setState(Wrist.wristState.NEUTRAL);

    }
    @Override
    public void start() {
        totalTime.resetTimer();
        setPathState(0);
    }
    @Override
    public void loop() {
        follower.update();
        updatePaths();
        telemetry.addData("path state", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.update();
    }
}
