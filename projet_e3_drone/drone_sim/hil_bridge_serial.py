# =============================================================================
# hil_bridge_serial.py  —  Pont HIL bidirectionnel vers ESP32 #1 via Serial (USB)
#
# Ce pont gère sur UN SEUL port série :
#   ÉCRITURE → télémétrie JSON envoyée vers ESP32 #1 (1 ligne par frame, 50 Hz)
#   LECTURE  → flux multiplexé reçu depuis ESP32 #1 :
#                - trames 'T' : JSON texte (commandes RC relayées depuis le drone)
#                - trames 'I' : JPEG brut (images caméra relayées depuis le drone)
#
# Protocole de lecture (défini côté firmware telemetry_sender_v3.cpp) :
#   En-tête 6 octets : [type(1)] [taille uint32 little-endian (4)] ['\n'(1)]
#   payload ensuite, de la taille annoncée.
#
# La lecture se fait dans un thread dédié pour ne jamais bloquer la boucle
# d'envoi télémétrie (qui tourne à 50 Hz dans le thread principal Qt).
# =============================================================================

import json
import struct
import threading
import time

import numpy as np
import cv2
import serial          # pip install pyserial

SERIAL_PORT = "COM7"
SERIAL_BAUD = 115200

HEADER_LEN = 6
TYPE_TELEM = ord('T')   # RC relayées depuis le drone
TYPE_IMAGE = ord('I')   # image caméra relayée depuis le drone

# Correspondance mode string → entier (identique au firmware ESP32)
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
    Pont HIL bidirectionnel : envoie la télémétrie vers l'ESP32 #1,
    et reçoit en parallèle (thread séparé) les images caméra + commandes RC
    relayées par le drone, sur le même port série.
    """

    def __init__(self, port: str = SERIAL_PORT, baud: int = SERIAL_BAUD):
        self.port = port
        self.baud = baud
        self._ser   = None
        self._actif = False

        # ── Thread de lecture ──────────────────────────────────────────
        self._thread_lecture = None
        self._lock = threading.Lock()

        self._derniere_image = None   # np.ndarray BGR ou None
        self._derniere_rc    = None   # dict ou None

        self._frames_recues = 0
        self._erreurs_jpeg  = 0
        self._rc_recues     = 0

    # ------------------------------------------------------------------
    # Démarrage / arrêt
    # ------------------------------------------------------------------

    def demarrer(self):
        try:
            self._ser = serial.Serial(self.port, self.baud, timeout=0.5)
            self._actif = True
            print(f"[HIL-Serial] Connecté sur {self.port} @ {self.baud}")
        except serial.SerialException as e:
            print(f"[HIL-Serial] ERREUR ouverture port : {e}")
            self._ser = None
            return

        self._thread_lecture = threading.Thread(
            target=self._boucle_lecture, daemon=True)
        self._thread_lecture.start()

    def arreter(self):
        self._actif = False
        if self._thread_lecture:
            self._thread_lecture.join(timeout=1.0)
        if self._ser and self._ser.is_open:
            self._ser.close()
        print("[HIL-Serial] Déconnecté.")

    # ------------------------------------------------------------------
    # ÉCRITURE — télémétrie vers ESP32 #1
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
            # Verrou partagé avec le thread de lecture pour éviter toute
            # collision si jamais on touchait au même buffer interne.
            # pyserial gère lui-même les accès concurrents read/write,
            # donc ce lock protège surtout nos structures Python internes.
            self._ser.write(ligne.encode('ascii'))
            return True
        except serial.SerialException as e:
            print(f"[HIL-Serial] Erreur envoi : {e}")
            return False

    @staticmethod
    def _construire_payload(drone_state) -> dict:
        """
        Construit le dict JSON plat compatible avec parseJSON() sur l'ESP32.
        Le mode est TOUJOURS converti en entier (0-4).
        """
        if isinstance(drone_state, dict):
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
                "mode"    : _mode_int(d.get("mode", "SOL")),
                "throttle": round(float(cmd.get("throttle", 0.0)), 3),
                "dist"    : round(float(d.get("cible_alt", 0.0)),  2),
            }

        else:
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
                "mode"    : _mode_int(s.mode_vol),
                "throttle": round(float(s.cmd_throttle),      3),
                "dist"    : round(float(s.distance_origine),  2),
            }

    # ------------------------------------------------------------------
    # LECTURE — flux multiplexé depuis ESP32 #1 (thread dédié)
    # ------------------------------------------------------------------

    def derniere_image(self):
        """Retourne la dernière image caméra reçue (np.ndarray BGR) ou None."""
        with self._lock:
            return None if self._derniere_image is None else self._derniere_image.copy()

    def derniere_rc(self):
        """Retourne le dernier dict RC relayé par le drone, ou None."""
        with self._lock:
            return self._derniere_rc

    @property
    def stats_lecture(self):
        return {
            "frames_recues": self._frames_recues,
            "erreurs_jpeg":  self._erreurs_jpeg,
            "rc_recues":     self._rc_recues,
        }

    def _lire_exact(self, n: int) -> bytes:
        """Lit exactement n octets depuis le port série, en accumulant."""
        data = b""
        while len(data) < n and self._actif:
            chunk = self._ser.read(n - len(data))
            if chunk:
                data += chunk
        return data

    def _boucle_lecture(self):
        while self._actif:
            try:
                header = self._lire_exact(HEADER_LEN)
                if len(header) < HEADER_LEN:
                    continue

                type_octet = header[0]
                taille     = struct.unpack("<I", header[1:5])[0]

                # Garde-fou anti-désynchronisation
                if taille == 0 or taille > 200_000:
                    continue

                payload = self._lire_exact(taille)
                if len(payload) < taille:
                    continue

                if type_octet == TYPE_TELEM:
                    self._traiter_telem(payload)
                elif type_octet == TYPE_IMAGE:
                    self._traiter_image(payload)

            except serial.SerialException as e:
                print(f"[HIL-Serial] Erreur lecture : {e}")
                break

    def _traiter_telem(self, payload: bytes):
        try:
            obj = json.loads(payload.decode("utf-8", errors="ignore"))
            with self._lock:
                self._derniere_rc = obj
            self._rc_recues += 1
        except json.JSONDecodeError:
            pass

    def _traiter_image(self, payload: bytes):
        arr = np.frombuffer(payload, dtype=np.uint8)
        img = cv2.imdecode(arr, cv2.IMREAD_COLOR)
        if img is not None:
            with self._lock:
                self._derniere_image = img
            self._frames_recues += 1
        else:
            self._erreurs_jpeg += 1


# =============================================================================
# Test standalone — envoie une télémétrie simulée et affiche le flux reçu
# =============================================================================
if __name__ == "__main__":
    import math, random

    bridge = HilBridgeSerial(port=SERIAL_PORT)
    bridge.demarrer()

    t = 0.0
    print("Envoi de données de test + affichage retour caméra... Ctrl+C pour arrêter.")
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
            s.mode_vol         = "VOL"
            s.cmd_throttle     = 0.55
            s.distance_origine = t * 0.1

            ok = bridge.envoyer(s)

            img = bridge.derniere_image()
            if img is not None:
                cv2.imshow("Retour caméra drone", img)

            rc = bridge.derniere_rc()
            stats = bridge.stats_lecture
            print(f"t={t:.1f}s envoi={'OK' if ok else 'ERR'} "
                  f"img_recues={stats['frames_recues']} rc={rc}", end="\r")

            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

            t += 0.1
            time.sleep(0.1)

    except KeyboardInterrupt:
        print("\nArrêt.")
    finally:
        bridge.arreter()
        cv2.destroyAllWindows()