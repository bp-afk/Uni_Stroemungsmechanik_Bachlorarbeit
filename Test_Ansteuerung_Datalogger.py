import time
from mcculw import ul
from mcculw.enums import ULRange, DigitalPortType, DigitalIODirection, ScanOptions
from mcculw.device_info import DaqDeviceInfo
import numpy as np
import ctypes

# Geräteeinstellungen
BOARD_NUM = 0        # Standardmäßig das erste Gerät in InstaCal/Universal Library

# --- 1. LED ansteuern ---
print("Schalte die LED EIN ...")
ul.flash_led(BOARD_NUM)
print("LED-Kommando gesendet.\n")

# --- 2. 10 Sekunden warten ---
time.sleep(10)
print("Starte Polling (Einzelabfrage) für 10 Sekunden ...")

start_time = time.time()
while time.time() - start_time < 10:
    wert = ul.a_in(BOARD_NUM, 0, ULRange.BIP5VOLTS)
    print(f"[Polling] Zeit: {time.strftime('%H:%M:%S')} | Kanal 0 Wert: {wert}")
    time.sleep(1)  # Einmal pro Sekunde abfragen

print("\nPolling beendet.\n")
time.sleep(2)

# --- 3. Streaming für 10 Sekunden ---
print("Starte Streaming (kontinuierlicher Scan) für 10 Sekunden ...")
sampling_rate = 1000  # 1000 Abtastungen/Sekunde
duration = 10         # 10 Sekunden
total_count = sampling_rate * duration
scan_channel = 0

# Leeres Array für die Messwerte anlegen
data = np.zeros(total_count, dtype=np.uint16)

# Scan starten (im Hintergrund)
data = (ctypes.c_ushort * total_count)()

ul.a_in_scan(
    BOARD_NUM,
    scan_channel,  # low_channel
    scan_channel,  # high_channel (nur 1 Kanal)
    total_count,
    sampling_rate,
    ULRange.BIP5VOLTS,
    data,
    ScanOptions.BACKGROUND
)


previous_count = 0
while ul.get_status(BOARD_NUM)[0] == 2:  # 2 = Run
    # Anzahl der bereits gemessenen Werte aus Buffer lesen
    status, curr_count, curr_index = ul.get_status(BOARD_NUM)
    if curr_count != previous_count:
        print(f"[Streaming] Buffer: {curr_count}/{total_count} Werte aufgenommen")
        previous_count = curr_count
    time.sleep(1)

# Messdaten anzeigen (nur die ersten 5 als Beispiel)
print(f"\nStreaming beendet. Erster Messwert: {data[0]}\nErste 5 Messwerte: {data[:5]}\n")

print("Fertig.")
