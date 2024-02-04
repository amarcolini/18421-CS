package org.firstinspires.ftc.teamcode

import com.amarcolini.joos.command.AbstractComponent
import com.amarcolini.joos.dashboard.JoosConfig
import com.amarcolini.joos.hardware.Servo

@JoosConfig
class DroneLauncher(private val servo: Servo) : AbstractComponent() {
    companion object {
        var closePosition = 0.4
        var openPosition = 1.0
    }

    init {
        subcomponents += servo
    }

    fun reset() {
        servo.position = closePosition
    }

    fun launch() {
        servo.position = openPosition
    }
}