package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

public class DcMotorGroup {
    DcMotorEx m1, m2;

    public DcMotorGroup(DcMotorEx _m1, DcMotorEx _m2) {
        m1 = _m1;
        m2 = _m2;
    }

    public void setZeroPowerBehavior(DcMotor.ZeroPowerBehavior zpb) {
        m1.setZeroPowerBehavior(zpb);
        m2.setZeroPowerBehavior(zpb);
    }

    public void setMode(DcMotor.RunMode rm) {
        m1.setMode(rm);
        m2.setMode(rm);
    }

    public void setTargetPosition(int pos) {
        m1.setTargetPosition(pos);
        m2.setTargetPosition(pos);
    }

    public void setPower(double pow) {
        m1.setPower(pow);
        m2.setPower(pow);
    }

    public void setTolerance(int tick) {
        m1.setTargetPositionTolerance(tick);
        m2.setTargetPositionTolerance(tick);
    }

    public boolean atTargetPosition() {
        return m1.getCurrentPosition() - m1.getTargetPosition() < m1.getTargetPositionTolerance();
    }
}
