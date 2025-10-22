import pygame
import time
import sys
import math
from spherov2 import scanner
from spherov2.types import Color
from spherov2.sphero_edu import SpheroEduAPI
from spherov2.commands.power import Power

PANEL_SIZE = 0.50         # each tile = 50 cm
SPEED_FACTOR = 0.85       # cm/s per speed unit (factor, in te stellen na test)
TRACK_SPEED = 120         # default race speed (in te stellen na test)

buttons = {
    '1': 0,
    '2': 1,
    '3': 2,
    '4': 3,
    'L1': 4,
    'L2': 6,
    'R1': 5,
    'R2': 7,
    'SELECT': 8,
    'START': 9
}

# Path van de track (panelen), te testen
TRACK_PATH = [
    {"panels": 4, "angle": 0},
    {"panels": 4, "angle": 270},
    {"panels": 2, "angle": 180},
    {"panels": 2, "angle": 90},
    {"panels": 4, "angle": 180},
    {"panels": 2, "angle": 270},
    {"panels": 2, "angle": 180},
    {"panels": 4, "angle": 90},
    {"panels": 3, "angle": 0},
]

def time_for_distance(meters, speed=TRACK_SPEED, k=SPEED_FACTOR):
    v_m_s = (k * speed) / 100.0 # convert meters to time
    return meters / v_m_s

def make_moves(start_angle, path, speed=TRACK_SPEED, panel_size=PANEL_SIZE, k=SPEED_FACTOR):
    # Convert a path {'panels', 'angle'} → (angle, speed, seconds)
    moves = []
    for step in path:
        angle = (int(step["angle"]) + int(start_angle)) % 360
        distance_m = float(step["panels"]) * panel_size
        duration_s = time_for_distance(distance_m, speed, k)
        moves.append((angle, speed, duration_s))
    return moves

class SpheroController:
    def __init__(self, joystick, color, player_number):
        self.toy = None
        self.speed = 50
        self.heading = 0
        self.base_heading = 0
        self.is_running = True
        self.calibration_mode = False
        self.joystick = joystick
        self.color = color
        self.number = player_number
        self.gameStartTime = time.time()
        self.gameOn = False
        self.boosterCounter = 0
        self.calibrated = False
        self.track_running = False
        # Prevent button spam
        self.prev_start = 0
        self.prev_select = 0

    # discover en connect de sphero
    def discover_toy(self, toy_name):
        try:
            self.toy = scanner.find_toy(toy_name=toy_name)
            print(f"Sphero toy '{toy_name}' discovered.")
        except Exception as e:
            print(f"Error discovering toy: {e}")

    def connect_toy(self):
        if self.toy:
            try:
                return SpheroEduAPI(self.toy)
            except Exception as e:
                print(f"Error connecting to toy: {e}")
        else:
            print("No toy discovered. Please run discover_toy() first.")
            return None

    def move(self, api, heading, speed):
        api.set_heading(heading)
        api.set_speed(speed)

    def run_track(self, api, moves):
        #Executes paths' moves(angle, speed, seconds)
        self.track_running = True
        try:
            for angle, speed, seconds in moves:
                api.set_heading(int(angle) % 360)
                api.set_speed(int(speed))
                time.sleep(max(0.0, float(seconds)))
            api.set_speed(0)
        finally:
            self.track_running = False

    # calibratie
    def toggle_calibration_mode(self, api, X):
        if not self.calibration_mode:
            self.enter_calibration_mode(api, X)
        else:
            self.exit_calibration_mode(api)

    def enter_calibration_mode(self, api, X):
        api.set_speed(0)
        self.gameStartTime = time.time()
        self.calibration_mode = True
        self.gameOn = False
        api.set_front_led(Color(255, 100, 150))  # pink = calibrating

        self.base_heading = api.get_heading()
        if X < -0.7:
            new_heading = self.base_heading - 5
        elif X > 0.7:
            new_heading = self.base_heading + 5
        else:
            new_heading = self.base_heading
        api.set_heading(new_heading)

    def exit_calibration_mode(self, api):
        self.calibrated = True
        self.calibration_mode = False
        self.gameOn = True
        self.boosterCounter = 0
        self.gameStartTime = time.time()
        api.set_front_led(Color(0, 255, 0))  # green = ready

    # LED display
    LED_PATTERNS = {1: '1', 2: '2', 3: '3', 4: '4', 5: '5'}

    def display_number(self, api):
        num_char = self.LED_PATTERNS.get(self.number)
        if num_char:
            api.set_matrix_character(num_char, self.color)

    def print_battery_level(self, api):
        v = Power.get_battery_voltage(self.toy)
        print(f"Battery of player {self.number}: {v:.2f} V")
        if v > 4.1:
            api.set_front_led(Color(0, 255, 0))
        elif 3.9 < v <= 4.1:
            api.set_front_led(Color(255, 255, 0))
        elif 3.7 < v <= 3.9:
            api.set_front_led(Color(255, 100, 0))
        elif 3.5 < v <= 3.7:
            api.set_front_led(Color(255, 0, 0))
        else:
            sys.exit("Battery too low")

    def control_toy(self):
        try:
            with self.connect_toy() as api:
                if api is None:
                    return
                last_battery_check = time.time()
                self.display_number(api)
                hillCounter = 0 

                while self.is_running:
                    pygame.event.pump()
                    now = time.time()
                    if now - last_battery_check >= 30:
                        self.print_battery_level(api)
                        last_battery_check = now

                    # motion monitoring
                    if self.gameOn:
                        acceleration_data = api.get_acceleration()
                        if acceleration_data is not None:
                            x_acc = acceleration_data['x']
                            z_acc = acceleration_data['z']
                            angle = math.degrees(math.atan2(x_acc, z_acc))
                            if abs(angle) >= 30:
                                hillCounter += 1
                                if hillCounter > 10:
                                    seconds = now - self.gameStartTime
                                    print(f"Player {self.number} going wild")
                            else:
                                hillCounter = 0
                        else:
                            print("Acceleration data is not available.")

                    X = self.joystick.get_axis(0)
                    Y = self.joystick.get_axis(1)

                    # Speed presets
                    if self.joystick.get_button(buttons['1']) == 1:
                        self.speed = 50
                        self.color = Color(255, 200, 0)
                        self.display_number(api)
                    if self.joystick.get_button(buttons['2']) == 1:
                        self.speed = 70
                        self.color = Color(255, 100, 0)
                        self.display_number(api)
                    if self.joystick.get_button(buttons['3']) == 1:
                        self.speed = 100
                        self.color = Color(255, 50, 0)
                        self.display_number(api)
                    if self.joystick.get_button(buttons['4']) == 1:
                        self.speed = 200
                        self.color = Color(255, 0, 0)
                        self.display_number(api)

                    # Button edge detection
                    select_now = self.joystick.get_button(buttons['SELECT'])
                    start_now  = self.joystick.get_button(buttons['START'])

                    if select_now == 1 and self.prev_select == 0:
                        self.toggle_calibration_mode(api, X)
                    self.prev_select = select_now

                    # Manual drive
                    if Y < -0.7:
                        self.move(api, self.base_heading, self.speed)
                    elif Y > 0.7:
                        self.move(api, self.base_heading + 180, self.speed)
                    elif X > 0.7:
                        self.move(api, self.base_heading + 22, 0)
                    elif X < -0.7:
                        self.move(api, self.base_heading - 22, 0)
                    else:
                        api.set_speed(0)

                    self.base_heading = api.get_heading()

                    # START pressed once (rising edge)
                    if start_now == 1 and self.prev_start == 0 and not self.track_running:
                        self.track_running = True
                        try:
                            base = self.base_heading
                            race_speed = self.speed if self.gameOn else TRACK_SPEED
                            moves = make_moves(base, TRACK_PATH, race_speed)
                            print("Starting race!")
                            api.set_main_led(0, 255, 0)
                            self.run_track(api, moves)
                            api.set_main_led(0, 0, 255)
                            print("Finished race!")
                        finally:
                            time.sleep(0.1)
                            self.track_running = False
                    self.prev_start = start_now
        except Exception:
            # FAIL-SAFE
            print("ERROR")
            try:
                if 'api' in locals() and api is not None:
                    api.set_speed(0)            # stopt
                    api.set_main_led(255, 0, 0) # rood 
            except Exception:
                pass

        finally:
            pygame.quit()

def main(toy_name=None, joystickID=0, playerID=1):
    pygame.init()
    pygame.joystick.init()

    if pygame.joystick.get_count() == 0:
        print("No joysticks found.")
        return

    joystick = pygame.joystick.Joystick(joystickID)
    joystick.init()

    controller = SpheroController(joystick, Color(255, 0, 0), playerID)

    if toy_name is None:
        sys.exit("No toy name provided")

    controller.discover_toy(toy_name)

    if controller.toy:
        print(f"Connected controller: {joystick.get_name()}")
        print("Press SELECT to calibrate, START to race!")
        controller.control_toy()

if __name__ == "__main__":
    if len(sys.argv) < 4:
        print("Usage: python sphero_race.py <toy_name> <joystickIndex 0|1> <player 1-5>")
        sys.exit(1)

    toy_name = sys.argv[1]
    joystick = int(sys.argv[2])
    playerid = int(sys.argv[3])
    print(f"Try to connect to: {toy_name} with number {joystick} for player {playerid}")
    main(toy_name, joystick, playerid)