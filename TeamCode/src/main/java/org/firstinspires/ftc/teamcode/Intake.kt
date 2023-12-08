package org.firstinspires.ftc.teamcode

import com.amarcolini.joos.command.AbstractComponent
import com.amarcolini.joos.hardware.Motor
import com.amarcolini.joos.hardware.Servo

class Intake(val motor: Motor, val servo: Servo) : AbstractComponent() {
    init {
        subcomponents += listOf(motor, servo)
    }
}