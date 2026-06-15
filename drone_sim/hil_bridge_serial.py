# =============================================================================
# hil_bridge_serial.py  —  Pont HIL vers ESP32 #1 via Serial (USB)
#
# Format JSON envoyé (1 ligne terminée par \n) :
# {"alt":1.5,"vz":-0.02,"vx":0.0,"vy":0.0,
#  "roll":0.01,"pitch":0.02,"yaw":0.5,
#  "bat_pct":98.5,"bat_v":12.4,
#  "m0":0.55,"m1":0.55,"m2":0.55,"m3":0.55,
#  "mode":2,"throttle":0.55,"dist":0.0}
#
# Modes : 0=SOL, 1=DECOLLAGE, 2=VOL, 3=ATTERRO, 4=URGENCE
# =============================================================================

import json
import serial          # pip install pyserial
import time

SERIAL_PORT = "COM7"
SERIAL_BAUD = 115200

# Correspondance mode string → entier (identique à hil_bridge.py)
MODE_CODE = {
    "SOL":       0,
    "DECOLLAGE": 1,
    "VOL":       2,
    "ATTERRO":   3,
    "URGENCE":   4,
}


def _mode_int(mode_raw) -> int:
    """
    Convertit le mode en entier, qu'il soit déjà un int ou une string.
    Ex : "VOL" → 2 / 2 → 2 / "DECOLLAGE" → 1
    """
    if isinstance(mode_raw, str):
        return MODE_CODE.get(mode_raw, 0)
    return int(mode_raw)


class HilBridgeSerial:
    """
    Pont HIL simplifié qui envoie la télémétrie vers l'ESP32 #1
    via liaison série USB, à la cadence souhaitée.
    """

    def __init__(self, port: str = SERIAL_PORT, baud: int = SERIAL_BAUD):
        self.port   = port
        self.baud   = baud
        self._ser   = None
        self._actif = False

    # ------------------------------------------------------------------

    def demarrer(self):
        try:
            self._ser   = serial.Serial(self.port, self.baud, timeout=1)
            self._actif = True
            print(f"[HIL-Serial] Connecté sur {self.port} @ {self.baud}")
        except serial.SerialException as e:
            print(f"[HIL-Serial] ERREUR ouverture port : {e}")
            self._ser = None

    def arreter(self):
        self._actif = False
        if self._ser and self._ser.is_open:
            self._ser.close()
        print("[HIL-Serial] Déconnecté.")

    # ------------------------------------------------------------------

    def envoyer(self, drone_state) -> bool:
        """
        Appelé depuis _tick() à chaque frame (50 Hz).
        Accepte un objet DroneState OU un dict exporté.
        """
        if not self._ser or not self._ser.is_open:
            return False

        payload = self._construire_payload(drone_state)

        try:
            ligne = json.dumps(payload, separators=(',', ':')) + '\n'
            self._ser.write(ligne.encode('ascii'))
            return True
        except serial.SerialException as e:
            print(f"[HIL-Serial] Erreur envoi : {e}")
            return False

    # ------------------------------------------------------------------

    @staticmethod
    def _construire_payload(drone_state) -> dict:
        """
        Construit le dict JSON plat compatible avec parseJSON() sur l'ESP32.
        Le mode est TOUJOURS converti en entier (0-4).
        """
        if isinstance(drone_state, dict):
            # ── Dict exporté par exporter_telemetrie() ────────────────
            d    = drone_state
            pos  = d.get("pos",     [0.0, 0.0, 0.0])
            vel  = d.get("vel",     [0.0, 0.0, 0.0])
            att  = d.get("att_rad", [0.0, 0.0, 0.0])
            mot  = d.get("moteurs", [0.0, 0.0, 0.0, 0.0])
            cmd  = d.get("cmd",     {})

            return {
                "alt"     : round(float(pos[2]),               3),
                "vz"      : round(float(vel[2]),               3),
                "vx"      : round(float(vel[0]),               3),
                "vy"      : round(float(vel[1]),               3),
                "roll"    : round(float(att[0]),               4),
                "pitch"   : round(float(att[1]),               4),
                "yaw"     : round(float(att[2]),               4),
                "bat_pct" : round(float(d.get("bat_pct", 100.0)), 1),
                "bat_v"   : round(float(d.get("bat_v",   12.6)),  2),
                "m0"      : round(float(mot[0]),               3),
                "m1"      : round(float(mot[1]),               3),
                "m2"      : round(float(mot[2]),               3),
                "m3"      : round(float(mot[3]),               3),
                "mode"    : _mode_int(d.get("mode", "SOL")),   # ← int !
                "throttle": round(float(cmd.get("throttle", 0.0)), 3),
                "dist"    : round(float(d.get("cible_alt", 0.0)),  2),
            }

        else:
            # ── Objet DroneState directement ──────────────────────────
            s    = drone_state
            mot  = s.moteurs if hasattr(s, "moteurs") else [0, 0, 0, 0]

            return {
                "alt"     : round(float(s.position.z),        3),
                "vz"      : round(float(s.vitesse.z),         3),
                "vx"      : round(float(s.vitesse.x),         3),
                "vy"      : round(float(s.vitesse.y),         3),
                "roll"    : round(float(s.roll),              4),
                "pitch"   : round(float(s.pitch),             4),
                "yaw"     : round(float(s.yaw),               4),
                "bat_pct" : round(float(s.batterie_pct),      1),
                "bat_v"   : round(float(s.batterie_tension),  2),
                "m0"      : round(float(mot[0]),              3),
                "m1"      : round(float(mot[1]),              3),
                "m2"      : round(float(mot[2]),              3),
                "m3"      : round(float(mot[3]),              3),
                "mode"    : _mode_int(s.mode_vol),            # ← int !
                "throttle": round(float(s.cmd_throttle),      3),
                "dist"    : round(float(s.distance_origine),  2),
            }


# =============================================================================
# Test standalone
# =============================================================================
if __name__ == "__main__":
    import math, random

    bridge = HilBridgeSerial(port=SERIAL_PORT)
    bridge.demarrer()

    t = 0.0
    print("Envoi de données de test... Ctrl+C pour arrêter.")
    try:
        while True:
            class FakeVec: pass
            class FakeState: pass

            s             = FakeState()
            pos           = FakeVec(); pos.x = 0; pos.y = 0
            pos.z         = 2.0 + math.sin(t) * 0.3
            vel           = FakeVec(); vel.x = 0; vel.y = 0
            vel.z         = math.cos(t) * 0.1
            s.position    = pos
            s.vitesse     = vel
            s.roll        = math.sin(t * 0.7) * 0.05
            s.pitch       = math.cos(t * 0.5) * 0.03
            s.yaw         = t * 0.2
            s.batterie_pct     = max(10.0, 98.0 - t * 0.05)
            s.batterie_tension = 12.6 - t * 0.001
            s.moteurs          = [0.55 + random.gauss(0, 0.01) for _ in range(4)]
            s.mode_vol         = "VOL"   # ← string, converti en 2 automatiquement
            s.cmd_throttle     = 0.55
            s.distance_origine = t * 0.1

            ok = bridge.envoyer(s)
            mode_int = _mode_int(s.mode_vol)
            print(f"t={t:.1f}s  alt={pos.z:.2f}m  mode={s.mode_vol}({mode_int})"
                  f"  bat={s.batterie_pct:.1f}%  envoi={'OK' if ok else 'ERR'}")

            t += 0.1
            time.sleep(0.1)

    except KeyboardInterrupt:
        print("\nArrêt.")
        bridge.arreter()
        