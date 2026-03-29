# ============================================================
#  A320 FCU – XPPython3 Plugin PI_A320_FCU.py
# ============================================================

import xp
import serial
import serial.tools.list_ports
import json
import threading
import time

SERIAL_PORT  = "/dev/ttyACM0"   # ← anpassen falls nötig (ttyACM1, ttyACM2 on linux)
SERIAL_BAUD  = 115200

class PythonInterface:

    def XPluginStart(self):
        self.name    = "A320 FCU"
        self.sig     = "a320fcu.arduino"
        self.desc    = "A320 FCU Arduino über Serial JSON"

        self.ser     = None
        self.running = False

        # ── DataRefs ──────────────────────────────────────────
        self.drSpeed   = xp.findDataRef("sim/cockpit2/autopilot/airspeed_dial_kts_mach")
        self.drMach    = xp.findDataRef("sim/cockpit2/autopilot/airspeed_is_mach")
        self.drHdg     = xp.findDataRef("sim/cockpit/autopilot/heading_mag")
        self.drAlt     = xp.findDataRef("sim/cockpit/autopilot/altitude")
        self.drVS      = xp.findDataRef("sim/cockpit/autopilot/vertical_velocity")
        self.drAP1     = xp.findDataRef("sim/cockpit2/autopilot/autopilot_on")
        self.drATHR    = xp.findDataRef("sim/cockpit2/autopilot/autothrottle_on")

        # ── Commands ──────────────────────────────────────────
        self.commands = {
            "AP1":     xp.findCommand("sim/autopilot/fdir_servos_toggle"),
            "ATHR":    xp.findCommand("sim/autopilot/autothrottle_toggle"),
            "APPR":    xp.findCommand("sim/autopilot/approach"),
            "LOC":     xp.findCommand("sim/autopilot/NAV"),
            "EXPED":   xp.findCommand("sim/autopilot/level_change"),
            "HDGVS":   xp.findCommand("sim/autopilot/heading"),
            "ALT_BTN": xp.findCommand("sim/autopilot/altitude_hold"),
            "SPD_BTN": xp.findCommand("sim/autopilot/fdir_servos_toggle"),
            "HDG_BTN": xp.findCommand("sim/autopilot/heading"),
            "VS_BTN":  xp.findCommand("sim/autopilot/altitude_hold"),
            "TRKFPA":  None,  # nur lokal am Arduino
        }

        # ── Flight Loop ───────────────────────────────────────
        self.flightLoopCB = self.flightLoop
        xp.registerFlightLoopCallback(self.flightLoopCB, 0.2, 0)

        xp.log("A320 FCU: Plugin gestartet")
        return self.name, self.sig, self.desc

    def XPluginStop(self):
        self.running = False
        xp.unregisterFlightLoopCallback(self.flightLoopCB, 0)
        if self.ser and self.ser.is_open:
            self.ser.close()
        xp.log("A320 FCU: Plugin gestoppt")

    def XPluginEnable(self):
        self.connectSerial()
        return 1

    def XPluginDisable(self):
        self.running = False
        if self.ser and self.ser.is_open:
            self.ser.close()

    def XPluginReceiveMessage(self, fromWho, message, param):
        pass

    # ── Serial verbinden ──────────────────────────────────────
    def connectSerial(self):
        try:
            self.ser = serial.Serial(SERIAL_PORT, SERIAL_BAUD, timeout=0.01)
            self.running = True
            t = threading.Thread(target=self.serialReader, daemon=True)
            t.start()
            xp.log(f"A320 FCU: Verbunden mit {SERIAL_PORT}")
        except Exception as e:
            xp.log(f"A320 FCU: Serial Fehler – {e}")
            xp.log("A320 FCU: Port in PI_A320_FCU.py anpassen (SERIAL_PORT)")

    # ── Serial lesen (eigener Thread) ─────────────────────────
    def serialReader(self):
        while self.running:
            try:
                if self.ser and self.ser.in_waiting:
                    line = self.ser.readline().decode("utf-8", errors="ignore").strip()
                    if line:
                        self.handleMessage(line)
            except Exception as e:
                xp.log(f"A320 FCU: Lesefehler – {e}")
                time.sleep(0.1)

    # ── Nachricht vom Arduino verarbeiten ─────────────────────
    def handleMessage(self, line):
        try:
            msg = json.loads(line)
        except json.JSONDecodeError:
            return

        t = msg.get("t")

        # Encoder gedreht
        if t == "e":
            name  = msg.get("n")
            delta = msg.get("d", 0)
            if name == "SPD":
                val = xp.getDataf(self.drSpeed)
                xp.setDataf(self.drSpeed, max(100, min(400, val + delta)))
            elif name == "HDG":
                val = xp.getDataf(self.drHdg)
                xp.setDataf(self.drHdg, (val + delta) % 360)
            elif name == "ALT":
                val = xp.getDataf(self.drAlt)
                xp.setDataf(self.drAlt, max(0, min(45000, val + delta * 100)))
            elif name == "VS":
                val = xp.getDataf(self.drVS)
                xp.setDataf(self.drVS, max(-6000, min(6000, val + delta * 100)))

        # Button gedrückt → Command senden
        elif t == "c":
            name = msg.get("n")
            cmd  = self.commands.get(name)
            if cmd:
                xp.commandOnce(cmd)

    # ── Flight Loop: DataRefs → Arduino senden ─────────────────
    def flightLoop(self, sinceLast, elapsedTime, counter, refcon):
        if not self.ser or not self.ser.is_open:
            return 0.2

        try:
            data = {
                "spd":  int(xp.getDataf(self.drSpeed)),
                "hdg":  int(xp.getDataf(self.drHdg)),
                "alt":  int(xp.getDataf(self.drAlt)),
                "vs":   int(xp.getDataf(self.drVS)),
                "mach": int(xp.getDatai(self.drMach)),
                "ap1":  int(xp.getDatai(self.drAP1)),
                "athr": int(xp.getDatai(self.drATHR)),
            }
            line = json.dumps(data) + "\n"
            self.ser.write(line.encode("utf-8"))
        except Exception as e:
            xp.log(f"A320 FCU: Sendefehler – {e}")

        return 0.2  
