package org.firstinspires.ftc.teamcode

import com.amarcolini.joos.command.AbstractComponent
import com.amarcolini.joos.hardware.Motor
import com.amarcolini.joos.hardware.Servo

class Intake(val motor: Motor, val servo: Servo) : AbstractComponent() {
    init {
        motor.runMode = Motor.RunMode.RUN_WITHOUT_ENCODER
        subcomponents += listOf(motor, servo)
    }

    enum class MotorState {
        ACTIVE, REVERSE, STOP
    }

    var motorState = MotorState.STOP
        set(value) {
            when (value) {
                MotorState.ACTIVE -> motor.power = 1.0
                MotorState.REVERSE -> motor.power = -0.5
                MotorState.STOP -> motor.power = 0.0
            }
            field = value
        }
}