import sys
import socket
import collections
import csv
import time
from datetime import datetime
from PyQt5.QtWidgets import QApplication, QMainWindow, QVBoxLayout, QWidget, QTextEdit
from PyQt5.QtCore import QTimer
import pyqtgraph as pg

class ESP32Plotter(QMainWindow):
    def __init__(self, host="0.0.0.0", port=5005, max_points=1000):
        super().__init__()
        self.setWindowTitle("ESP32 PID Telemetry & Logger via Wi-Fi (UDP)")
        self.resize(1000, 800)

        # Configurazione UDP non bloccante
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.bind((host, port))
        self.sock.setblocking(False)

        self.start_time = None  # Orario di avvio prima telemetria

        # Buffer Dati Grafico
        self.max_points = max_points
        self.temp_time_data = collections.deque(maxlen=max_points)
        self.temp_val_data = collections.deque(maxlen=max_points)
        self.pwm_time_data = collections.deque(maxlen=max_points)
        self.pwm_val_data = collections.deque(maxlen=max_points)

        # Buffer Setpoint REALE trasmesso dall'ESP32
        self.sp_time_data = collections.deque(maxlen=max_points)
        self.sp_val_data = collections.deque(maxlen=max_points)

        # Storico COMPLETO per l'esportazione in CSV
        self.full_log_history = []
        self.sample_index = 0

        # Layout Interfaccia Grafica
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        layout = QVBoxLayout(central_widget)

        # Plot 1: Temperatura & Setpoint
        self.plot_temp = pg.PlotWidget(title="Temperatura (°C) vs Setpoint ESP32")
        self.plot_temp.showGrid(x=True, y=True)
        self.plot_temp.setLabel('left', 'Temperatura (°C)')
        self.plot_temp.setLabel('bottom', 'Tempo (secondi)')
        self.curve_temp = self.plot_temp.plot(pen=pg.mkPen('r', width=2), name="Temp Misurata")
        self.curve_setpoint = self.plot_temp.plot(pen=pg.mkPen('g', width=2, style=pg.QtCore.Qt.DashLine), name="Setpoint ESP32")
        layout.addWidget(self.plot_temp)

        # Plot 2: PWM Ventola
        self.plot_pwm = pg.PlotWidget(title="Output Ventola PWM (0-100%)")
        self.plot_pwm.showGrid(x=True, y=True)
        self.plot_pwm.setLabel('left', 'PWM Output (0-100%)')
        self.plot_pwm.setLabel('bottom', 'Tempo (secondi)')
        self.plot_pwm.setYRange(0, 105)
        self.curve_pwm = self.plot_pwm.plot(pen=pg.mkPen('b', width=2), name="PWM Ventola")
        layout.addWidget(self.plot_pwm)

        # Console Log Testuale
        self.log_console = QTextEdit()
        self.log_console.setReadOnly(True)
        self.log_console.setMaximumHeight(150)
        layout.addWidget(self.log_console)

        # Timer Refresh GUI a 20 Hz (50 ms)
        self.timer = QTimer()
        self.timer.setInterval(50)
        self.timer.timeout.connect(self.update_telemetry)
        self.timer.start()

    def update_telemetry(self):
        curr_sys_time = time.time()
        new_packet_received = False

        # 1. SVUOTAMENTO DEL BUFFER UDP
        while True:
            try:
                data, _ = self.sock.recvfrom(1024)
                line = data.decode('utf-8').strip()

                if line.startswith("$DATA"):
                    parts = line.split(',')
                    if len(parts) >= 5:
                        temp = float(parts[1])
                        humidity = float(parts[2])
                        esp_setpoint = float(parts[3])
                        pwm = float(parts[4])
                        # Legge lo stato se presente, altrimenti imposta 0
                        system_state = int(parts[5]) if len(parts) >= 6 else 0

                        if self.start_time is None:
                            self.start_time = curr_sys_time

                        elapsed_sec = curr_sys_time - self.start_time
                        timestamp_str = datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]

                        # Aggiornamento sincronizzato dei buffer
                        self.sample_index += 1
                        self.temp_time_data.append(elapsed_sec)
                        self.temp_val_data.append(temp)
                        
                        self.sp_time_data.append(elapsed_sec)
                        self.sp_val_data.append(esp_setpoint)

                        self.pwm_time_data.append(elapsed_sec)
                        self.pwm_val_data.append(pwm)

                        new_packet_received = True

                        # Registrazione CSV (incluso lo Stato)
                        self.full_log_history.append([
                            timestamp_str,
                            self.sample_index,
                            round(elapsed_sec, 2),
                            temp,
                            humidity,
                            esp_setpoint,
                            pwm,
                            system_state
                        ])

                        # Evidenziazione guasto in Console
                        state_str = "NORMAL" if system_state == 0 else ("FINISHED" if system_state == 1 else "FAULT")
                        log_msg = f"[{timestamp_str}] {line} | STATE: {state_str}"
                        self.log_console.append(log_msg)

            except (BlockingIOError, socket.error):
                break
            except Exception as e:
                print(f"Errore durante il parsing del pacchetto: {e}")
                break

        # 2. RENDER GRAFICO
        if new_packet_received:
            self.curve_setpoint.setData(list(self.sp_time_data), list(self.sp_val_data))
            self.curve_temp.setData(list(self.temp_time_data), list(self.temp_val_data))
            self.curve_pwm.setData(list(self.pwm_time_data), list(self.pwm_val_data))

    def closeEvent(self, event):
        """Salvataggio automatico CSV alla chiusura"""
        if self.full_log_history:
            filename = f"telemetria_pid_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"
            try:
                with open(filename, mode='w', newline='', encoding='utf-8') as file:
                    writer = csv.writer(file)
                    writer.writerow(["Timestamp", "SampleIndex", "ElapsedSeconds", "Temperatura", "Umidita", "Setpoint_ESP32", "PWM_Output", "Stato"])
                    writer.writerows(self.full_log_history)
                print(f"\n[LOGGER] Dati salvati con successo in: {filename}")
            except Exception as e:
                print(f"\n[LOGGER] Errore durante il salvataggio CSV: {e}")
        else:
            print("\n[LOGGER] Nessun dato da salvare.")
        
        self.sock.close()
        event.accept()

if __name__ == '__main__':
    app = QApplication(sys.argv)
    window = ESP32Plotter()
    window.show()
    sys.exit(app.exec_())