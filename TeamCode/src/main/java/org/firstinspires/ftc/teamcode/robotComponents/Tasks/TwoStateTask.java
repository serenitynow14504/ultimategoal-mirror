package org.firstinspires.ftc.teamcode.robotComponents.Tasks;

public abstract class TwoStateTask extends Task {
    protected boolean state;

    public TwoStateTask(boolean state) {
        this.state = state;
    }
}
