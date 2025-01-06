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

@Autonomous(name = "5+0 Specimen", group = "Auto")
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
    private final Pose PARKPOSE = new Pose(8.396139705882353,6.882352941176478,Math.toRadians(-10));
    private final Pose PARKCONTROL = new Pose(7.235294117647059,70.23529411764707);


    private PathChain scorePreload, prepare1, push1, prepare2, push2, prepare3, push3, push3ToWall, score1, score1ToWall, score2, score2ToWall, score3, score3ToWall, score4, park; //Define paths

    private void buildPaths() {
        scorePreload = follower.pathBuilder()
                .addPath(new Path(new BezierLine(new Point(STARTPOSE), new Point(PRELOADPOSE))))
                .setLinearHeadingInterpolation(STARTPOSE.getHeading(), PRELOADPOSE.getHeading())
                .build();
        prepare1 = follower.pathBuilder()
                .addPath(new Path(new BezierCurve(new Point(PRELOADPOSE), new Point(PREPARE1CONTROL), new Point(PREPARE1POSE))))
                .setLinearHeadingInterpolation(PRELOADPOSE.getHeading(), PREPARE1POSE.getHeading())
                .build();
        push1 = follower.pathBuilder()
                .addPath(new Path(new BezierLine(new Point(PREPARE1POSE), new Point(PUSH1POSE))))
                .setLinearHeadingInterpolation(PREPARE1POSE.getHeading(), PUSH1POSE.getHeading())
                .build();
        prepare2 = follower.pathBuilder()
                .addPath(new Path(new BezierLine(new Point(PUSH1POSE), new Point(PREPARE2POSE))))
                .setLinearHeadingInterpolation(PUSH1POSE.getHeading(), PREPARE2POSE.getHeading())
                .build();
        push2 = follower.pathBuilder()
                .addPath(new Path(new BezierLine(new Point(PREPARE2POSE), new Point(PUSH2POSE))))
                .setLinearHeadingInterpolation(PREPARE2POSE.getHeading(), PUSH2POSE.getHeading())
                .build();
        prepare3 = follower.pathBuilder()
                .addPath(new Path(new BezierLine(new Point(PUSH2POSE), new Point(PREPARE3POSE))))
                .setLinearHeadingInterpolation(PUSH2POSE.getHeading(), PREPARE3POSE.getHeading())
                .build();
        push3 = follower.pathBuilder()
                .addPath(new Path(new BezierLine(new Point(PREPARE3POSE), new Point(PUSH3POSE))))
                .setLinearHeadingInterpolation(PREPARE3POSE.getHeading(), PUSH3POSE.getHeading())
                .build();
        push3ToWall = follower.pathBuilder()
                .addPath(new Path(new BezierLine(new Point(PUSH3POSE), new Point(WALLPOSE))))
                .setConstantHeadingInterpolation(WALLPOSE.getHeading())
                .build();
        score1 = follower.pathBuilder()
                .addPath(new Path(new BezierLine(new Point(WALLPOSE), new Point(SCORE1POSE))))
                .setLinearHeadingInterpolation(WALLPOSE.getHeading(), SCORE1POSE.getHeading())
                .build();
        score1ToWall = follower.pathBuilder()
                .addPath(new Path(new BezierLine(new Point(SCORE1POSE), new Point(WALLPOSE))))
                .setLinearHeadingInterpolation(SCORE1POSE.getHeading(), WALLPOSE.getHeading())
                .build();
        score2 = follower.pathBuilder()
                .addPath(new Path(new BezierLine(new Point(WALLPOSE), new Point(SCORE2POSE))))
                .setLinearHeadingInterpolation(WALLPOSE.getHeading(), SCORE2POSE.getHeading())
                .build();
        score2ToWall = follower.pathBuilder()
                .addPath(new Path(new BezierLine(new Point(SCORE2POSE), new Point(WALLPOSE))))
                .setLinearHeadingInterpolation(SCORE2POSE.getHeading(), WALLPOSE.getHeading())
                .build();
        score3 = follower.pathBuilder()
                .addPath(new Path(new BezierLine(new Point(WALLPOSE), new Point(SCORE3POSE))))
                .setLinearHeadingInterpolation(WALLPOSE.getHeading(), SCORE3POSE.getHeading())
                .build();
        score3ToWall = follower.pathBuilder()
                .addPath(new Path(new BezierLine(new Point(SCORE3POSE), new Point(WALLPOSE))))
                .setLinearHeadingInterpolation(SCORE3POSE.getHeading(), WALLPOSE.getHeading())
                .build();
        score4 = follower.pathBuilder()
                .addPath(new Path(new BezierLine(new Point(WALLPOSE), new Point(SCORE4POSE))))
                .setLinearHeadingInterpolation(WALLPOSE.getHeading(), SCORE4POSE.getHeading())
                .build();
        park = follower.pathBuilder()
                .addPath(new Path(new BezierCurve(new Point(SCORE4POSE), new Point(PARKCONTROL), new Point(PARKPOSE))))
                .setLinearHeadingInterpolation(SCORE4POSE.getHeading(), PARKPOSE.getHeading())
                .build();
    }
    private void updatePaths() {
        switch (pathState) {
            case 0:
                extendo.setTargetPos(Extendo.MIN);
                intakeWrist.setState(IntakeWrist.intakeWristState.IN);
                slides.setTargetPos(Slides.MED);
                bar.setState(Bar.BarState.CLIP);
                wrist.setState(Wrist.wristState.CLIP);
                claw.setState(Claw.ClawState.CLOSE);
                follower.followPath(scorePreload, true);
                setPathState(1);
                break;
            case 1:
                if ((Math.abs(PRELOADPOSE.getX() - follower.getPose().getX()) <= 1) && (Math.abs(PRELOADPOSE.getY() - follower.getPose().getY()) <= 1)) {
                    slides.setTargetPos(Slides.LOW);
                    setPathState(2);
                }
                break;
            case 2:
                if (pathTime.getElapsedTimeSeconds()>0.22) {
                    claw.setState(Claw.ClawState.OPEN);
                    follower.followPath(prepare1, true);
                    setPathState(3);
                }
                break;
            case 3:
                if ((Math.abs(PREPARE1POSE.getX() - follower.getPose().getX()) <= 3) && (Math.abs(PREPARE1POSE.getY() - follower.getPose().getY()) <= 3)) {
                    extendo.setTargetPos(Extendo.MAX);
                    intakeWrist.setState(IntakeWrist.intakeWristState.OUT);
                    setPathState(4);
                }
                break;
            case 4:
                if ((Math.abs(PREPARE1POSE.getX() - follower.getPose().getX()) <= 1) && (Math.abs(PREPARE1POSE.getY() - follower.getPose().getY()) <= 1)) {
                    follower.followPath(push1);
                    setPathState(5);
                }
                break;
            case 5:
                if ((Math.abs(PUSH1POSE.getX() - follower.getPose().getX()) <= 1) && (Math.abs(PUSH1POSE.getY() - follower.getPose().getY()) <= 1)) {
                    follower.followPath(prepare2);
                    setPathState(6);
                }
                break;
            case 6:
                if ((Math.abs(PREPARE2POSE.getX() - follower.getPose().getX()) <= 1) && (Math.abs(PREPARE2POSE.getY() - follower.getPose().getY()) <= 1)) {
                    follower.followPath(push2);
                    setPathState(7);
                }
                break;
            case 7:
                if ((Math.abs(PUSH2POSE.getX() - follower.getPose().getX()) <= 1) && (Math.abs(PUSH2POSE.getY() - follower.getPose().getY()) <= 1)) {
                    follower.followPath(prepare3);
                    setPathState(8);
                }
                break;
            case 8:
                if ((Math.abs(PREPARE3POSE.getX() - follower.getPose().getX()) <= 1) && (Math.abs(PREPARE3POSE.getY() - follower.getPose().getY()) <= 1)) {
                    follower.followPath(push3);
                    setPathState(9);
                }
                break;
            case 9:
                if (pathTime.getElapsedTimeSeconds() > 0.5) { //expected to be too early as of jan 5
                    extendo.setTargetPos(Extendo.MIN);
                    bar.setState(Bar.BarState.WALL);
                    wrist.setState(Wrist.wristState.WALL);
                    setPathState(10);
                }
                break;
            case 10:
                if ((Math.abs(PUSH3POSE.getX() - follower.getPose().getX()) <= 1) && (Math.abs(PUSH3POSE.getY() - follower.getPose().getY()) <= 1)) {
                    follower.followPath(push3ToWall);
                    setPathState(11);
                }
                break;
            case 11:
                if ((Math.abs(WALLPOSE.getX() - follower.getPose().getX()) <= 1) && (Math.abs(WALLPOSE.getY() - follower.getPose().getY()) <= 1)) {
                    slides.setTargetPos(Slides.MED);
                    bar.setState(Bar.BarState.CLIP);
                    wrist.setState(Wrist.wristState.CLIP);
                    claw.setState(Claw.ClawState.CLOSE);
                    follower.followPath(score1);
                    setPathState(12);
                }
                break;
            case 12:
                if ((Math.abs(SCORE1POSE.getX() - follower.getPose().getX()) <= 1) && (Math.abs(SCORE1POSE.getY() - follower.getPose().getY()) <= 1)) {
                    slides.setTargetPos(Slides.LOW);
                    setPathState(13);
                }
                break;
            case 13:
                if (pathTime.getElapsedTimeSeconds()>0.22) {
                    claw.setState(Claw.ClawState.OPEN);
                    follower.followPath(score1ToWall, true);
                    setPathState(14);
                }
                break;
            case 14:
                if (pathTime.getElapsedTimeSeconds() > 0.5) {
                    bar.setState(Bar.BarState.WALL);
                    wrist.setState(Wrist.wristState.WALL);
                    setPathState(15);
                }
                break;
            case 15:
                if ((Math.abs(WALLPOSE.getX() - follower.getPose().getX()) <= 1) && (Math.abs(WALLPOSE.getY() - follower.getPose().getY()) <= 1)) {
                    slides.setTargetPos(Slides.MED);
                    bar.setState(Bar.BarState.CLIP);
                    wrist.setState(Wrist.wristState.CLIP);
                    claw.setState(Claw.ClawState.CLOSE);
                    follower.followPath(score2);
                    setPathState(16);
                }
                break;
            case 16:
                if ((Math.abs(SCORE2POSE.getX() - follower.getPose().getX()) <= 1) && (Math.abs(SCORE2POSE.getY() - follower.getPose().getY()) <= 1)) {
                    slides.setTargetPos(Slides.LOW);
                    setPathState(17);
                }
                break;
            case 17:
                if (pathTime.getElapsedTimeSeconds() > 0.22) {
                    claw.setState(Claw.ClawState.OPEN);
                    follower.followPath(score2ToWall, true);
                    setPathState(18);
                }
                break;
            case 18:
                if (pathTime.getElapsedTimeSeconds() > 0.5) {
                    bar.setState(Bar.BarState.WALL);
                    wrist.setState(Wrist.wristState.WALL);
                    setPathState(19);
                }
                break;
            case 19:
                if ((Math.abs(WALLPOSE.getX() - follower.getPose().getX()) <= 1) && (Math.abs(WALLPOSE.getY() - follower.getPose().getY()) <= 1)) {
                    slides.setTargetPos(Slides.MED);
                    bar.setState(Bar.BarState.CLIP);
                    wrist.setState(Wrist.wristState.CLIP);
                    claw.setState(Claw.ClawState.CLOSE);
                    follower.followPath(score3);
                    setPathState(20);
                }
                break;
            case 20:
                if ((Math.abs(SCORE3POSE.getX() - follower.getPose().getX()) <= 1) && (Math.abs(SCORE3POSE.getY() - follower.getPose().getY()) <= 1)) {
                    slides.setTargetPos(Slides.LOW);
                    setPathState(21);
                }
                break;
            case 21:
                if (pathTime.getElapsedTimeSeconds() > 0.22) {
                    claw.setState(Claw.ClawState.OPEN);
                    follower.followPath(score3ToWall, true);
                    setPathState(22);
                }
                break;
            case 22:
                if (pathTime.getElapsedTimeSeconds() > 0.5) {
                    bar.setState(Bar.BarState.WALL);
                    wrist.setState(Wrist.wristState.WALL);
                    setPathState(23);
                }
                break;
            case 23:
                if ((Math.abs(WALLPOSE.getX() - follower.getPose().getX()) <= 1) && (Math.abs(WALLPOSE.getY() - follower.getPose().getY()) <= 1)) {
                    slides.setTargetPos(Slides.MED);
                    bar.setState(Bar.BarState.CLIP);
                    wrist.setState(Wrist.wristState.CLIP);
                    claw.setState(Claw.ClawState.CLOSE);
                    follower.followPath(score4);
                    setPathState(24);
                }
                break;
            case 24:
                if ((Math.abs(SCORE4POSE.getX() - follower.getPose().getX()) <= 1) && (Math.abs(SCORE4POSE.getY() - follower.getPose().getY()) <= 1)) {
                    slides.setTargetPos(Slides.LOW);
                    setPathState(25);
                }
                break;
            case 25:
                if (pathTime.getElapsedTimeSeconds() > 0.22) {
                    claw.setState(Claw.ClawState.OPEN);
                    follower.followPath(park, true);
                    setPathState(26);
                }
                break;
            case 26:
                if (pathTime.getElapsedTimeSeconds() > 0.5) { //set bar wrist to a init-able position
                    setPathState(27);
                }
                break;
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
