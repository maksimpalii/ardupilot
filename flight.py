from dronekit import connect, VehicleMode, LocationGlobalRelative
import time
import math
import csv

# --- 1. КЛАС ПІД-РЕГУЛЯТОРА ---
class PIDController:
    def __init__(self, kp, ki, kd, min_out, max_out):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.min_out = min_out
        self.max_out = max_out
        self.prev_error = 0
        self.integral = 0

    def calculate(self, error, dt):
        self.integral += error * dt
        self.integral = max(min(self.integral, self.max_out), self.min_out) 
        derivative = (error - self.prev_error) / dt if dt > 0 else 0
        self.prev_error = error
        
        output = (self.kp * error) + (self.ki * self.integral) + (self.kd * derivative)
        return max(min(output, self.max_out), self.min_out)

# --- 2. ДОПОМІЖНІ ФУНКЦІЇ (ВЕКТОРНА ГЕОМЕТРІЯ) ---
def get_bearing(loc1, loc2):
    off_x = loc2.lon - loc1.lon
    off_y = loc2.lat - loc1.lat
    bearing = 90.00 + math.atan2(-off_y, off_x) * 57.2957795
    if bearing < 0: bearing += 360.00
    return bearing

def get_body_frame_errors(current_loc, target_loc, heading):
    # Обчислення глобальних похибок (в метрах)
    d_lat = target_loc.lat - current_loc.lat
    error_north = d_lat * 1.113195e5 
    
    d_lon = target_loc.lon - current_loc.lon
    error_east = d_lon * 1.113195e5 * math.cos(math.radians(current_loc.lat))
    
    # Матриця повороту у систему координат дрона
    rad_head = math.radians(heading)
    error_forward = error_north * math.cos(rad_head) + error_east * math.sin(rad_head)
    error_right = -error_north * math.sin(rad_head) + error_east * math.cos(rad_head)
    
    return error_forward, error_right

# --- 3. ОСНОВНА ЛОГІКА ---
def main():
    print("Підключення до SITL...")
    vehicle = connect('127.0.0.1:14551', wait_ready=True)

    vehicle.parameters['SIM_WIND_SPD'] = 3
    vehicle.parameters['SIM_WIND_DIR'] = 30
    vehicle.parameters['SIM_WIND_TURB'] = 2
    vehicle.parameters['SIM_WIND_TURB_FREQ'] = 0.2

    target_loc = LocationGlobalRelative(50.443326, 30.448078, 100)
    target_alt = 100

    pid_alt = PIDController(kp=15.0, ki=2.0, kd=5.0, min_out=-500, max_out=500)
    pid_pitch = PIDController(kp=15.0, ki=1.0, kd=20.0, min_out=-450, max_out=450) 
    pid_roll  = PIDController(kp=15.0, ki=1.0, kd=20.0, min_out=-450, max_out=450)
    pid_yaw = PIDController(kp=4.0, ki=0.0, kd=1.0, min_out=-250, max_out=250)

    log_filename = "flight_log_full.csv"
    with open(log_filename, mode='w', newline='') as log_file:
        log_writer = csv.writer(log_file)
        log_writer.writerow(['Time', 'Distance_m', 'Alt_m', 'Err_Fwd_m', 'Err_Right_m', 'PWM_Roll', 'PWM_Pitch', 'PWM_Throttle', 'PWM_Yaw'])

        print("Перехід у режим STABILIZE...")
        vehicle.mode = VehicleMode("STABILIZE")
        while vehicle.mode.name != "STABILIZE": time.sleep(0.5)

        print("Армінг моторів...")
        vehicle.armed = True
        while not vehicle.armed: time.sleep(0.5)

        print("Зліт!")

        loop_rate = 0.1 
        is_landing = False
        start_time = time.time()

        try:
            while True:
                current_loc = vehicle.location.global_relative_frame
                current_alt = current_loc.alt
                current_heading = vehicle.heading

                err_fwd, err_right = get_body_frame_errors(current_loc, target_loc, current_heading)
                distance = math.sqrt(err_fwd**2 + err_right**2) 
                bearing_to_target = get_bearing(current_loc, target_loc)

                if distance < 3.0 and not is_landing: 
                    print("\n>>> Ціль у зоні захоплення! Вмикаю прецизійний спуск... <<<")
                    is_landing = True
                    pid_pitch.integral = 0
                    pid_roll.integral = 0
                
                if is_landing:
                    if distance < 1.5:
                        target_alt -= 0.3 
                    
                    if target_alt < 0: target_alt = 0
                    
                    if current_alt < 0.3:
                        print("\n>>> ІДЕАЛЬНА ПОСАДКА ЗАВЕРШЕНА! <<<")
                        vehicle.channels.overrides = {'3': 1000}
                        vehicle.armed = False
                        break

                alt_error = target_alt - current_alt
                
                yaw_error = bearing_to_target - current_heading
                if yaw_error > 180: yaw_error -= 360
                if yaw_error < -180: yaw_error += 360

                base_pwm = 1500
                hover_pwm = 1450 
                
                out_throttle = hover_pwm + pid_alt.calculate(alt_error, loop_rate)
                out_yaw = base_pwm + pid_yaw.calculate(yaw_error, loop_rate)
                out_pitch = base_pwm - pid_pitch.calculate(err_fwd, loop_rate) 
                out_roll = base_pwm + pid_roll.calculate(err_right, loop_rate) 

                def clamp(val): return max(1000, min(2000, int(val)))

                pwm_r = clamp(out_roll)
                pwm_p = clamp(out_pitch)
                pwm_t = clamp(out_throttle)
                pwm_y = clamp(out_yaw)

                vehicle.channels.overrides = {
                    '1': pwm_r, 
                    '2': pwm_p, 
                    '3': pwm_t,
                    '4': pwm_y
                }

                # Логування у файл
                elapsed_time = round(time.time() - start_time, 1)
                log_writer.writerow([elapsed_time, round(distance, 2), round(current_alt, 2), round(err_fwd, 2), round(err_right, 2), pwm_r, pwm_p, pwm_t, pwm_y])

                # ОНОВЛЕНО: Детальний і відформатований вивід у консоль кожні 0.5 секунд
                if int(time.time() * 10) % 5 == 0: 
                    # Використовуємо :5.1f для фіксованої ширини (5 символів всього, 1 після коми)
                    status_line = (
                        f"[{elapsed_time:5.1f}s] "
                        f"Дист: {distance:5.1f}м | Вис: {current_alt:5.1f}м | "
                        f"Пох(Вп/Пр): {err_fwd:5.1f} / {err_right:5.1f} | "
                        f"ШІМ[R:{pwm_r} P:{pwm_p} T:{pwm_t} Y:{pwm_y}]"
                    )
                    print(status_line)

                time.sleep(loop_rate)

        except KeyboardInterrupt:
            print("\nПерервано користувачем. Очищення каналів...")
            vehicle.channels.overrides = {}
            
        finally:
            vehicle.close()
            print(f"Лог збережено у файл {log_filename}")

if __name__ == '__main__':
    main()