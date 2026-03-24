import csv
import matplotlib.pyplot as plt

def main():
    log_filename = "flight_log_full.csv"
    
    times, distances, altitudes = [], [], []
    err_fwds, err_rights = [], []
    pwm_rolls, pwm_pitches, pwm_throttles, pwm_yaws = [], [], [], []

    print(f"Читання даних з {log_filename}...")
    try:
        with open(log_filename, 'r') as file:
            reader = csv.DictReader(file)
            for row in reader:
                times.append(float(row['Time']))
                distances.append(float(row['Distance_m']))
                altitudes.append(float(row['Alt_m']))
                err_fwds.append(float(row['Err_Fwd_m']))
                err_rights.append(float(row['Err_Right_m']))
                pwm_rolls.append(float(row['PWM_Roll']))
                pwm_pitches.append(float(row['PWM_Pitch']))
                pwm_throttles.append(float(row['PWM_Throttle']))
                pwm_yaws.append(float(row['PWM_Yaw']))
    except FileNotFoundError:
        print(f"Помилка: Файл {log_filename} не знайдено.")
        return

    print("Побудова графіків...")
    
    # Створюємо 4 зони для графіків
    fig, axs = plt.subplots(4, 1, figsize=(10, 14), sharex=True)
    fig.suptitle('Аналіз Векторного Керування (Режим Stabilize + Компенсація вітру)', fontsize=16)

    # 1. Дистанція та Висота (на одному графіку для порівняння)
    axs[0].plot(times, distances, 'b-', label='Дистанція до цілі (м)', linewidth=2)
    axs[0].plot(times, altitudes, 'g-', label='Висота (м)', linewidth=2)
    axs[0].set_ylabel('Метри')
    axs[0].grid(True)
    axs[0].legend(loc='upper right')
    axs[0].set_title('Профіль польоту (Дистанція та Висота)')

    # 2. Векторні похибки (Для ідеальної посадки вони мають зійтися в нуль)
    axs[1].plot(times, err_fwds, 'purple', label='Похибка Вперед/Назад (м)', linewidth=1.5)
    axs[1].plot(times, err_rights, 'brown', label='Похибка Вліво/Вправо (м)', linewidth=1.5)
    axs[1].set_ylabel('Похибка (м)')
    axs[1].grid(True)
    axs[1].legend(loc='upper right')
    axs[1].set_title('Локальні осі дрона (Боротьба зі знесенням)')
    axs[1].axhline(y=0, color='k', linestyle='--', alpha=0.5)

    # 3. ШІМ: Просторове керування (Крен та Тангаж)
    axs[2].plot(times, pwm_rolls, 'orange', label='Ch 1: Roll (Крен)', linewidth=1.5)
    axs[2].plot(times, pwm_pitches, 'cyan', label='Ch 2: Pitch (Тангаж)', linewidth=1.5)
    axs[2].set_ylabel('ШІМ (мкс)')
    axs[2].grid(True)
    axs[2].legend(loc='upper right')
    axs[2].set_title('Просторове керування (Нахили)')
    axs[2].axhline(y=1500, color='k', linestyle=':', alpha=0.5)

    # 4. ШІМ: Тяга та Напрямок (Газ та Нишпорення)
    axs[3].plot(times, pwm_throttles, 'red', label='Ch 3: Throttle (Газ)', linewidth=1.5)
    axs[3].plot(times, pwm_yaws, 'black', label='Ch 4: Yaw (Нишпорення)', linewidth=1.5, alpha=0.7)
    axs[3].set_ylabel('ШІМ (мкс)')
    axs[3].set_xlabel('Час (с)')
    axs[3].grid(True)
    axs[3].legend(loc='upper right')
    axs[3].set_title('Управління тягою та курсом')
    axs[3].axhline(y=1500, color='k', linestyle=':', alpha=0.5)

    plt.tight_layout()
    plt.subplots_adjust(top=0.95)
    
    output_image = "flight_analysis_vector.png"
    plt.savefig(output_image, dpi=300)
    print(f"Графік успішно збережено як {output_image}")
    
    plt.show()

if __name__ == '__main__':
    main()