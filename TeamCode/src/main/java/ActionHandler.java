import android.util.Log;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.concurrent.TimeUnit;

import mechanisms.*;

public class ActionHandler {
    private Slides slides;
    private Extendo extendo;
    private Bar bar;
    private Wrist wrist;
    private Intake intake;
    private Claw claw;
    private IntakeWrist intakeWrist;
    private Colorsensor colorSensor;

    private static boolean intaking, transferring = false;
    private static boolean extendoout = false;

    private String alliance;

    private ElapsedTime timer = new ElapsedTime();
    private ElapsedTime intakeTimer = new ElapsedTime();
    private boolean waitingForSecondCheck = false;

    private ActionState currentActionState = ActionState.IDLE;

    enum ActionState {
        IDLE,
        TRANSFER_STAGE_1, //intakewrist in BEFORE, stage1: claw close when wait is done
        TRANSFER_STAGE_2, //flywheel out
        TRANSFER_STAGE_3, //barwrist transfer, flywheel stop
        TRANSFER_STAGE_4, //clawopen
        TRANSFER_STAGE_5,
        CLIP, //delay to wrist move
        WALLPICKUP,
        HIGHBUCKET, //slides up BEFORE
        SLIDESDOWN, //extendo in
        RESETEXTENDO,
        RESETINTAKEWRIST_STAGE_1, RESETINTAKEWRIST_STAGE_2,
        NUDGE1, NUDGE2, NUDGE3, CLIPPOS, NUDGE4
    }

    public void init(Slides s, Extendo e, Bar b, Wrist w, Intake f, Claw c, IntakeWrist iw, Colorsensor cs, String alliance) {
        slides = s;
        extendo = e;
        bar = b;
        wrist = w;
        intake = f;
        claw = c;
        intakeWrist = iw;
        colorSensor = cs;
        this.alliance = alliance;
        bar.setState(Bar.BarState.NEUTRAL);
        claw.setState(Claw.ClawState.CLOSE);
        wrist.setState(Wrist.wristState.NEUTRAL);
        extendo.setTargetPos(Extendo.MIN);
    }

    public void Loop(Gamepad gp1, Gamepad gp2) {
        //clip
        if (gp2.x) {
            wallPickup();
        }
        if (gp2.left_bumper) {
            claw.setState(Claw.ClawState.CLOSE);
        }

        if (gp2.y) {
            clippos();
        }
        if (gp2.a) {
            clip_down();
        }
        if (gp2.b){
            bar.setState(Bar.BarState.NEUTRAL);
            wrist.setState(Wrist.wristState.NEUTRAL);
        }

        //intake
        if (gp1.y && !intaking) {
            intake();
        }
        intakeCheck();

        if (gp1.left_bumper && !transferring) {
            transfer();
            transferring = true;
        }
        if (gp2.left_stick_button && gp2.right_stick_button) {
            nudge();
        }

        if (gp2.dpad_up) {
            highBucket();
        }
        if (gp2.right_bumper){
            claw.setState(Claw.ClawState.OPEN);
        }

        if (gp2.dpad_down) {
            slidesDown();
        }

        //extendo
        if (gp1.right_bumper){
            extendo.setTargetPos(Extendo.MAX);
            extendoout = true;
        }
        if (gp1.right_trigger > 0.8){
            extendo.setTargetPos(Extendo.MAX);
            extendoout = true;
            if (intakeWrist.currentState != IntakeWrist.intakeWristState.IN) {
                if (isExtendoout()) {
                    intakeWrist.setState(IntakeWrist.intakeWristState.OUT);
                } else {
                    intakeWrist.setState(IntakeWrist.intakeWristState.SUPEROUT);
                }
            }
        }
        if (gp1.left_trigger > 0.8){
            intakeWrist.setState(IntakeWrist.intakeWristState.IN);
            extendo.setTargetPos(Extendo.MIN);
            extendoout = false;
        }
        if (gp1.options) {
            intake.setState(Intake.intakeState.OUT);
        }

        //reset
//        if (gp1.left_trigger > 0.5) {
//            resetIntakeWrist();
//        }
        if (gp1.touchpad_finger_1 && gp1.touchpad_finger_2){
            intakeWrist.setState(IntakeWrist.intakeWristState.IN);
            intake.setState(Intake.intakeState.STOP);
            intaking = false;
            gp1.rumbleBlips(3);
            gp2.rumbleBlips(3);
        }
        if (gp1.dpad_down) {
            resetExtendo();
        }

        TimedActions();
    }

    public void TimedActions() {
        long elapsedMs = timer.time(TimeUnit.MILLISECONDS);

        switch (currentActionState) {
            //transfer
            case TRANSFER_STAGE_1:
                if (elapsedMs >= 200) {
                    extendo.setTargetPos(Extendo.MIN);
                    extendoout = false;
                    currentActionState = ActionState.TRANSFER_STAGE_2;
                    timer.reset();
                }
                break;
            case TRANSFER_STAGE_2:
                if (elapsedMs >= 100) {
//                    intake.setState(Intake.intakeState.OUT);
                    claw.setState(Claw.ClawState.OPEN);
                    Log.d("hello", "hello!!");
                    currentActionState = ActionState.TRANSFER_STAGE_3;
                    timer.reset();
                }
                break;
            case TRANSFER_STAGE_3:
                if (elapsedMs >= 1000) {
                    bar.setState(Bar.BarState.TRANSFER);
                    wrist.setState(Wrist.wristState.TRANSFER);
                    currentActionState = ActionState.TRANSFER_STAGE_4;
                    timer.reset();
                }
                break;
            case TRANSFER_STAGE_4:
                if (elapsedMs >= 250) {
                    claw.setState(Claw.ClawState.CLOSE);
                    currentActionState = ActionState.TRANSFER_STAGE_5;
                    timer.reset();
                }
                break;
            case TRANSFER_STAGE_5:
                if (elapsedMs >= 500) {
//                    bar.setState(Bar.BarState.NEUTRAL);
                    currentActionState = ActionState.IDLE;
                    transferring = false;
                }
                break;

            //high bucket
            case HIGHBUCKET:
                if (elapsedMs >= 700) {
                    bar.setState(Bar.BarState.BUCKET);
                    wrist.setState(Wrist.wristState.BUCKET);
                    currentActionState = ActionState.IDLE;
                }
                break;
            case SLIDESDOWN:
                if (elapsedMs >= 200){
                    slides.setTargetPos(Slides.GROUND);
                    currentActionState = ActionState.IDLE;
                }
                break;

            //wall pickup
            case WALLPICKUP:
                if (elapsedMs >= 500){
                    bar.setState(Bar.BarState.WALL);
                    intakeWrist.setState(IntakeWrist.intakeWristState.IN);
                    currentActionState = ActionState.IDLE;
                }
                break;

            //clipping
            case CLIP:
                if (elapsedMs >= 200) {
                    claw.setState(Claw.ClawState.OPEN);
                    currentActionState = ActionState.IDLE;
                }
                break;

            //reset extendo
            case RESETEXTENDO:
                if (elapsedMs >= 1000) {
                    extendo.DANGEROUS_RESET_ENCODERS();
                    currentActionState = ActionState.IDLE;
                }
                break;

            //reset intake wrist
            case RESETINTAKEWRIST_STAGE_1:
                if (elapsedMs >= 500) {
                    intakeWrist.setState(IntakeWrist.intakeWristState.IN);
                    currentActionState = ActionState.RESETINTAKEWRIST_STAGE_2;
                    timer.reset();
                }
                break;
            case RESETINTAKEWRIST_STAGE_2:
                if (elapsedMs >= 500) {
                    intakeWrist.setState(IntakeWrist.intakeWristState.OUT);
                    intake.setState(Intake.intakeState.IN);
                    currentActionState = ActionState.IDLE;
                }
                break;

            //nudge sample in intake
            case NUDGE1:
                if (elapsedMs >= 100) {
                    bar.setState(Bar.BarState.TRANSFER);
                    currentActionState = ActionState.NUDGE2;
                    timer.reset();
                }
                break;
            case NUDGE2:
                if (elapsedMs >= 100) {
                    bar.setState(Bar.BarState.NEUTRAL);
                    currentActionState = ActionState.NUDGE3;
                    timer.reset();
                }
                break;
            case NUDGE3:
                if (elapsedMs >= 100) {
                    bar.setState(Bar.BarState.TRANSFER);
                    currentActionState = ActionState.NUDGE4;
                    timer.reset();
                }
                break;
            case NUDGE4:
                if (elapsedMs >= 200){
                    claw.setState(Claw.ClawState.OPEN);
                    bar.setState(Bar.BarState.NEUTRAL);
                    currentActionState = ActionState.IDLE;
                }
                break;
            case CLIPPOS:
                if (elapsedMs >= 500) {
                    bar.setState(Bar.BarState.CLIP);
                    wrist.setState(Wrist.wristState.CLIP);
                    currentActionState = ActionState.IDLE;
                }
                break;

            default:
                currentActionState = ActionState.IDLE;
                break;
        }
    }

    private void wallPickup() {
        slides.setTargetPos(Slides.GROUND);
        wrist.setState(Wrist.wristState.WALL);
        currentActionState = ActionState.WALLPICKUP;
        timer.reset();
    }
    public void clippos() {
        slides.setTargetPos(Slides.MED);
        currentActionState = ActionState.CLIPPOS;
        timer.reset();
    }
    public void clip_down(){
        slides.setTargetPos(Slides.GROUND);
        currentActionState = ActionState.CLIP;
        timer.reset();
    }

    private void intake() {
        intaking = true;
        intake.setState(Intake.intakeState.IN);
        if (extendoout) {
            intakeWrist.setState(IntakeWrist.intakeWristState.OUT);
        } else {
            intakeWrist.setState(IntakeWrist.intakeWristState.SUPEROUT);
        }
    }

    private void resetIntakeWrist() {
        intake.setState(Intake.intakeState.STOP);
        intaking = false;
        currentActionState = ActionState.RESETINTAKEWRIST_STAGE_1;
        timer.reset();
    }

    private void transfer() {
        bar.setState(Bar.BarState.NEUTRAL);
        wrist.setState(Wrist.wristState.TRANSFER);
        intakeWrist.setState(IntakeWrist.intakeWristState.IN);
        currentActionState = ActionState.TRANSFER_STAGE_1;
        timer.reset();
        intake.setState(Intake.intakeState.STOP);
        intaking = false;
    }

    private void nudge(){
        claw.setState(Claw.ClawState.CLOSE);
        bar.setState(Bar.BarState.NEUTRAL);
        wrist.setState(Wrist.wristState.TRANSFER);
        currentActionState = ActionState.NUDGE1;
        timer.reset();
    }

    public void intakeCheck() { //Thanks chatgpt
        if (intaking) {
            // Wait for 300ms before checking again
            if (intakeTimer.milliseconds() >= 600) {
                if (!waitingForSecondCheck) {
                    // First check: Determine if the color is correct
                    boolean correctColor = (alliance.equals("red") && (colorSensor.sensorIsRed() || colorSensor.sensorIsYellow()))
                            || (alliance.equals("blue") && (colorSensor.sensorIsBlue() || colorSensor.sensorIsYellow()));

                    if (correctColor) {
                        // Found the correct color, initiate second check
                        waitingForSecondCheck = true;
                        intakeTimer.reset(); // Reset timer for second check
                    } else {
                        // Handle wrong color immediately
                        boolean wrongColor = (alliance.equals("red") && colorSensor.sensorIsBlue())
                                || (alliance.equals("blue") && colorSensor.sensorIsRed());

                        if (wrongColor) {
                            intake.setState(Intake.intakeState.OUT); // Reverse flywheel to eject
                            intaking = false; // Stop intaking
                        }
                        intakeTimer.reset(); // Reset timer for the next cycle
                    }
                } else {
                    // Second check after 300ms
                    boolean correctColor = (alliance.equals("red") && (colorSensor.sensorIsRed() || colorSensor.sensorIsYellow()))
                            || (alliance.equals("blue") && (colorSensor.sensorIsBlue() || colorSensor.sensorIsYellow()));

                    if (correctColor) {
                        // Confirmed correct color, stop flywheel and intaking
                        intake.setState(Intake.intakeState.STOP);
                        intaking = false;
                    }

                    // Reset state and timer for the next loop
                    waitingForSecondCheck = false;
                    intakeTimer.reset();
                }
            }
        }
    }

    private void highBucket() {
        slides.setTargetPos(Slides.HIGH);
        currentActionState = ActionState.HIGHBUCKET;
        timer.reset();
    }

    private void slidesDown() {
        bar.setState(Bar.BarState.NEUTRAL);
        wrist.setState(Wrist.wristState.TRANSFER);
        currentActionState = ActionState.SLIDESDOWN;
        timer.reset();
    }

    private void resetExtendo() {
        extendo.setTargetPos(-700);
        currentActionState = ActionState.RESETEXTENDO;
        timer.reset();
    }
    public boolean isIntaking() {
        return intaking;
    }
    public boolean isTransferring() {return transferring;}
    public boolean isExtendoout() {return extendoout;}
}