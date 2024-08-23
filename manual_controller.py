class ManualControllerMixin:
    # ----------------- MANUAL CONTROLLER ----------------- #

    def change_manual_steer_angle(self, inc):
        # self.set_autodrive(False)
        if inc == 0:
            self.manual_steering = 0
            self.set_steering_angle(0)

        new_manual_steering = self.manual_steering + inc
        if -25.0 <= new_manual_steering <= 25.0:
            self.manual_steering = new_manual_steering
            self.set_steering_angle(self.manual_steering * 0.02)

        if self.manual_steering == 0:
            pass
        else:
            pass

    def check_keyboard(self):
        key = self.keyboard.getKey()
        if key == self.keyboard.UP:
            self.set_speed(self.speed + 1.0)
        elif key == self.keyboard.DOWN:
            self.set_speed(self.speed - 1.0)
        elif key == self.keyboard.RIGHT:
            self.change_manual_steer_angle(+1)
        elif key == self.keyboard.LEFT:
            self.change_manual_steer_angle(-1)
        elif key == 'A':
            self.set_autodrive(True)
        elif not self.autodrive:
            self.change_manual_steer_angle(0)