import time
from mcculw import ul
from mcculw.enums import ULRange

# Geräteeinstellungen
BOARD_NUM = 0          # Standardgerät (instaCal)
MEASURE_CHANNEL = 0    # Analogkanal (0-7 beim USB-1208FS-PL)
MEASURE_TIME = 10      # Sekunde(n)

# Formel zur Umrechnung (bipolar, ±5V, 12 Bit)
def adc_to_voltage(adc, n_bits=12, v_range=5):
    max_adc = (2 ** n_bits) - 1  # z.B. 4095
    return (adc - (max_adc / 2)) * (v_range / (max_adc / 2))

print(f"Messung für {MEASURE_TIME} Sekunden gestartet …\n")

start_time = time.time()
while time.time() - start_time < MEASURE_TIME:
    adc_value = ul.a_in(BOARD_NUM, MEASURE_CHANNEL, ULRange.BIP5VOLTS)
    voltage = adc_to_voltage(adc_value)        # Standard: ±5V, 12bit
    print(f"[Polling] Zeit: {time.strftime('%H:%M:%S')} | "
          f"ADC-Wert (Bits): {adc_value} | Spannung: {voltage:+.4f} V")
    time.sleep(1)

print(f"\nMessung beendet. Kanäle aus CH{MEASURE_CHANNEL}.")