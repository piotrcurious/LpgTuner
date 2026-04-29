import csv
import matplotlib.pyplot as plt

def plot_sim(csv_path, output_path):
    time = []
    l1_v = []
    l2_v = []
    off1 = []
    off2 = []
    drift_en = []
    lim_en = []
    rpm = []
    afr = []

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
            rpm.append(float(row['RPM']))
            afr.append(float(row['AFR1']))

    fig, (ax1, ax2, ax3, ax4) = plt.subplots(4, 1, figsize=(12, 14), sharex=True)

    # RPM and Base AFR
    ax1.plot(time, rpm, color='purple', label='RPM')
    ax1_twin = ax1.twinx()
    ax1_twin.plot(time, afr, color='green', label='Base AFR', linestyle='--')
    ax1.set_ylabel('RPM')
    ax1_twin.set_ylabel('AFR')
    ax1.legend(loc='upper left')
    ax1_twin.legend(loc='upper right')
    ax1.set_title('Engine Operating Conditions')

    # Lambda Voltages
    ax2.plot(time, l1_v, label='L1 (Lean Target 0.45V)', color='blue')
    ax2.plot(time, l2_v, label='L2 (Rich Target 0.65V)', color='red')
    ax2.axhline(y=0.45, color='blue', linestyle='--', alpha=0.5)
    ax2.axhline(y=0.65, color='red', linestyle='--', alpha=0.5)
    ax2.set_ylabel('Voltage (V)')
    ax2.legend()
    ax2.set_title('Lambda Sensor Voltages')

    # Offsets
    ax3.plot(time, off1, label='Offset 1', color='blue')
    ax3.plot(time, off2, label='Offset 2', color='red')
    ax3.set_ylabel('DAC Offset')
    ax3.legend()
    ax3.set_title('Injection Offsets')

    # Emulation Status
    ax4.step(time, drift_en, label='Drift Emulation', color='green', where='post')
    ax4.step(time, lim_en, label='Limit Emulation', color='orange', where='post')
    ax4.set_ylabel('Enabled (0/1)')
    ax4.set_xlabel('Time (s)')
    ax4.legend()
    ax4.set_title('Emulation Status')

    plt.tight_layout()
    plt.savefig(output_path)
    print(f"Plot saved to {output_path}")

if __name__ == "__main__":
    plot_sim('split_bank_lambda/sim/sim_output.csv', 'split_bank_lambda/sim/sim_plot.png')
