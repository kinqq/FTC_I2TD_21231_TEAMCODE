package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Time;
import com.acmerobotics.roadrunner.Twist2dDual;

/**
 * Interface for localization methods.
 */
public interface Localizer {
    Twist2dDual<Time> update();
}