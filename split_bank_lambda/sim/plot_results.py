import csv
import matplotlib.pyplot as plt
import os
import glob

def plot_scenario(csv_path):
    scenario_name = os.path.basename(csv_path).replace('sim_', '').replace('.csv', '')
    output_path = csv_path.replace('.csv', '.png')

    time = []
    l1_v = []
    l2_v = []
    off1 = []
    off2 = []
    drift_en = []
    lim_en = []

    with open(csv_path, 'r') as f:
        reader = csv.DictReader(f)
        for row in reader:
            time.append(float(row['Time']))
            l1_v.append(float(row['L1_V']))
            l2_v.append(float(row['L2_V']))
            off1.append(float(row['Off1']))
            off2.append(float(row['Off2']))
            drift_en.append(int(row['Drift_En']))
            lim_en.append(int(row['Lim_En']))

    fig, (ax1, ax2, ax3) = plt.subplots(3, 1, figsize=(10, 12), sharex=True)

    # Lambda Voltages
    ax1.plot(time, l1_v, label='L1 (Lean Target 0.45V)', color='blue')
    ax1.plot(time, l2_v, label='L2 (Rich Target 0.65V)', color='red')
    ax1.axhline(y=0.45, color='blue', linestyle='--', alpha=0.5)
    ax1.axhline(y=0.65, color='red', linestyle='--', alpha=0.5)
    ax1.set_ylabel('Voltage (V)')
    ax1.legend()
    ax1.set_title(f'Scenario: {scenario_name} - Lambda Sensor Voltages')

    # Offsets
    ax2.plot(time, off1, label='Offset 1', color='blue')
    ax2.plot(time, off2, label='Offset 2', color='red')
    ax2.set_ylabel('DAC Offset')
    ax2.legend()
    ax2.set_title('Injection Offsets')

    # Emulation Status
    ax3.step(time, drift_en, label='Drift Emulation', color='green', where='post')
    ax3.step(time, lim_en, label='Limit Emulation', color='orange', where='post')
    ax3.set_ylabel('Enabled (0/1)')
    ax3.set_xlabel('Time (s)')
    ax3.legend()
    ax3.set_title('Emulation Status')

    plt.tight_layout()
    plt.savefig(output_path)
    plt.close()
    print(f"Generated plot: {output_path}")

if __name__ == "__main__":
    csv_files = glob.glob('split_bank_lambda/sim/sim_*.csv')
    for csv_file in csv_files:
        plot_scenario(csv_file)
