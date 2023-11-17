package org.firstinspires.ftc.teamcode

import com.amarcolini.joos.command.AbstractComponent
import com.amarcolini.joos.command.Command
import com.amarcolini.joos.dashboard.JoosConfig
import com.amarcolini.joos.hardware.Servo

@JoosConfig
class PixelPlopper(private val servo: Servo) : AbstractComponent() {
    init {
        subcomponents += servo
    }

    companion object {
        var closePosition = 0.1
        var openPosition = 0.5
    }

    var isPrimed = false
        private set

    fun plop(): Command = Command.of {
        isPrimed = false
        servo.position = openPosition
    }
        .wait(0.5)
        .then {
            servo.position = closePosition
            isPrimed = true
        }
        .requires(this).setInterruptable(false)

    fun prime() {
        servo.position = closePosition
        isPrimed = true
    }

    fun open() {
        servo.position = openPosition
        isPrimed = false
    }
}