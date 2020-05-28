class Ultra96IO:
    def __init__(self, params, encoder_pins=None, motor_pins=None, limit_pins=None):

        # Implement default list arguments this way so there aren't any problems
        # if the class is used multiple times
        if encoder_pins is None:
            encoder_pins = [0, 1]
        if motor_pins is None:
            motor_pins = [508, 509]
        if limit_pins is None:
            limit_pins = [510, 511]

        self.encoder_scale = params["encoder_scale"]["val"]
        self.params = params

        # Save pin numbers so we can use them later...
        self.encoder_pins = encoder_pins
        self.motor_pins = motor_pins
        self.limit_pins = limit_pins

        # Prepare GPIO pins for use
        self.__export_pin(self.params.gpio_device_prefix, self.motor_pins[0])
        self.__export_pin(self.params.gpio_device_prefix, self.motor_pins[1])
        self.__export_pin(self.params.gpio_device_prefix, self.limit_pins[0])
        self.__export_pin(self.params.gpio_device_prefix, self.limit_pins[1])
        self.__set_pin_direction(self.motor_pins[0], "out")
        self.__set_pin_direction(self.motor_pins[1], "out")
        self.__set_pin_direction(self.limit_pins[0], "in")
        self.__set_pin_direction(self.limit_pins[1], "in")

        # Prepare PWM for use
        self.__export_pin(self.params.pwm_device_prefix, 0)
        self.setMotorV(0)
        self.__write_pwm_pin("enable", 0, 1)

        # Save default values which will get overwritten soon enough
        self.encoder = [0.0, 0.0]
        self.encoder_offset = [0.0, 0.0]
        self.limit = [False, False]

    def getEncoder(self, index):
        return self.encoder[index] + self.encoder_offset[index]

    def setEncoderOffset(self, index, offset):
        self.encoder_offset[index] += offset

    def getSwitch(self, index):
        return self.limit[index]

    def read(self):
        self.limit[0] = int(self.__read_pin("value", self.limit_pins[0]))
        self.limit[1] = int(self.__read_pin("value", self.limit_pins[1]))
        self.encoder[0] = float(self.__read_encoder_pin("in_steps0_raw", 0))
        self.encoder[1] = float(self.__read_encoder_pin("in_steps0_raw", 1))

    def setMotorV(self, voltage):
        # Determine which direction to turn the motor
        duty_cycle = voltage / self.params.motor_vcc_voltage * self.params.motor_pwm_period
        if duty_cycle == 0:
            self.__set_motor_direction(0)
        elif duty_cycle < 0:
            self.__set_motor_direction(-1)
        else:
            self.__set_motor_direction(1)

        # Set the duty cycle to achieve the proper voltage
        self.__write_pwm_pin("duty_cycle", 0, abs(duty_cycle))
        

    def __export_pin(self, prefix, pin):
        handle = open(prefix + "/export", "w")
        print(str(pin), file=handle)
        handle.close()

    def __set_pin_direction(self, pin, direction):
        handle = open(self.params.gpio_device_prefix + "/gpio" + str(pin) + "/direction", "w")
        print(direction, file=handle)
        handle.close()

    def __read_pin(self, parameter, pin):
        handle = open(self.params.gpio_device_prefix + "/gpio" + str(pin) + "/" + parameter, "r")
        ret = handle.read()
        handle.close()
        return ret

    def __read_encoder_pin(self, parameter, pin):
        handle = open(self.params.iio_device_prefix + str(pin) + "/" + parameter)
        ret = handle.read()
        handle.close()
        return ret

    def __write_pin(self, parameter, pin, value):
        handle = open(self.params.gpio_device_prefix + "/gpio" + str(pin) + "/" + parameter, "w")
        print(str(value), file=handle)
        handle.close()

    def __write_pwm_pin(self, parameter, pin, value):
        handle = open(self.params.pwm_device_prefix + "/pwm" + str(pin) + "/" + parameter, "w")
        print(str(value), file=handle)
        handle.close()

    def __set_motor_direction(self, direction):
        outputs = [0, 0]
        if direction == -1:
            outputs[1] = 1
        elif direction == 1:
            outputs[0] = 1

        self.__write_pin("value", self.motor_pins[0], outputs[0])
        self.__write_pin("value", self.motor_pins[1], outputs[1])

