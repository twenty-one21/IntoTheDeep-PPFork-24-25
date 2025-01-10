package mechanisms;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Bar {
    private Servo barServoRight;
    public enum BarState {TRANSFERUP, TRANSFER, WALL, BUCKET, CLIP, NEUTRAL}
    public BarState currentState = BarState.NEUTRAL;

    public final double TRANSFERUP = 0.9; //figure out
    public final double TRANSFER = 0.85; //as of jan 3
    public final double WALL = 0.9;
    public final double BUCKET = 0.20;
    public final double CLIP = 0.402;
    public final double NEUTRAL = 0.70;

    public void init(HardwareMap hm) {
        barServoRight = hm.get(Servo.class, "bar");
        barServoRight.setDirection(Servo.Direction.REVERSE);
    }

    public void Loop() {
        switch(currentState) {
            case TRANSFERUP:
                setPos(TRANSFERUP);
            case TRANSFER:
                setPos(TRANSFER);
                break;
            case BUCKET:
                setPos(BUCKET);
                break;
            case WALL:
                setPos(WALL);
                break;
            case CLIP:
                setPos(CLIP);
                break;
            case NEUTRAL:
                setPos(NEUTRAL);
                break;
            default:
                setPos(NEUTRAL);
                break;
        }
    }

    private void setPos(double pos) {
        barServoRight.setPosition(pos);
    }

    public void setState(BarState state) {
        this.currentState = state;
    }

    public String getState(){
        return currentState.name();
    }
}
