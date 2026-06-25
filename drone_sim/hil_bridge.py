# =============================================================================
# hil_bridge.py
# Hardware-In-the-Loop Bridge — non-bloquant, 50 Hz
#
# Deux canaux indépendants :
#   ① UDP  → ESP32 #1 (JSON aplati, compatible parseJSON côté Arduino)
#   ② Serial → carte physique (STM32, ESP32, Pixhawk…)
#
# Protocole UDP  : JSON aplati, un paquet par tick, port configurable
# Protocole Serial : trame binaire compacte (46 octets) + checksum XOR
#
# Format JSON envoyé en UDP (compatible telemetry_sender.cpp) :
# {"alt":1.5,"vz":-0.02,"vx":0.0,"vy":0.0,"roll":0.01,"pitch":0.02,"yaw":0.5,
#  "bat_pct":98.5,"bat_v":12.4,"m0":0.55,"m1":0.55,"m2":0.55,"m3":0.55,
#  "mode":1,"throttle":0.55,"dist":0.0}
#
# Trame Serial (46 octets) :
#   [0]     0xAB            — header
#   [1]     0xCD            — header
#   [2-3]   uint16          — compteur de trame (little-endian)
#   [4-7]   float32         — temps_vol (s)
#   [8-11]  float32         — pos.x (m)
#   [12-15] float32         — pos.y (m)
#   [16-19] float32         — pos.z / altitude (m)
#   [20-23] float32         — vel.x (m/s)
#   [24-27] float32         — vel.y (m/s)
#   [28-31] float32         — vel.z (m/s)
#   [32-35] float32         — roll  (rad)
#   [36-39] float32         — pitch (rad)
#   [40-43] float32         — yaw   (rad)
#   [44]    uint8           — mode_vol (0=SOL 1=DECO 2=VOL 3=ATTERRO 4=URG)
#   [45]    uint8           — checksum XOR des octets [0..44]
#
# Architecture thread :
#   MainThread  → appelle HilBridge.envoyer() à chaque tick 50 Hz
#   UDPThread   → socket UDP non-bloquant dans le thread principal (sendto)
#   SerialThread→ queue + thread dédié (Serial.write bloque potentiellement)
#
# Usage dans main_2.py :
#   from hil_bridge import HilBridge
#   self.hil = HilBridge()          # dans __init__
#   self.hil.demarrer()
#   ...
#   self.hil.envoyer(s.exporter_telemetrie())   # dans _tick()
#   ...
#   self.hil.arreter()              # dans closeEvent
# =============================================================================

import json
import socket
import struct
import threading
import queue
import time
import logging
from dataclasses import dataclass
from typing import Optional

log = logging.getLogger("HIL")


# ---------------------------------------------------------------------------
# Configuration — modifiez ici sans toucher au reste
# ---------------------------------------------------------------------------

@dataclass
class HilConfig:
    # ── UDP ──────────────────────────────────────────────────────────────
    udp_actif      : bool  = True
    udp_host       : str   = "192.168.4.1"  # IP de l'ESP32 #1 (AP mode)
    udp_port       : int   = 1234           # port d'écoute côté ESP32
    udp_timeout_s  : float = 0.0            # 0 = non-bloquant (sendto best-effort)

    # ── Serial ───────────────────────────────────────────────────────────
    serial_actif   : bool  = False          # False tant que la carte n'est pas branchée
    serial_port    : str   = "/dev/ttyUSB0" # Linux : /dev/ttyUSB0 | Windows : COM3
    serial_baud    : int   = 115200
    serial_timeout : float = 0.01           # lecture non-bloquante

    # ── Général ──────────────────────────────────────────────────────────
    queue_max      : int   = 4              # profondeur queue Serial (drops si pleine)
    log_interval_s : float = 5.0            # fréquence des stats dans les logs


# ---------------------------------------------------------------------------
# Correspondance mode_vol string → uint8
# ---------------------------------------------------------------------------

MODE_CODE = {
    "SOL":       0,
    "DECOLLAGE": 1,
    "VOL":       2,
    "ATTERRO":   3,
    "URGENCE":   4,
}

HEADER_A  = 0xAB
HEADER_B  = 0xCD
TRAME_LEN = 46   # octets, checksum inclus


# ===========================================================================
# HilBridge
# ===========================================================================

class HilBridge:
    """
    Point d'entrée unique pour l'envoi de télémétrie vers l'ESP32 et la carte.

    Thread-safe : seule la méthode envoyer() est appelée depuis le thread Qt.
    Le Serial tourne dans son propre thread pour ne jamais bloquer le timer.
    L'UDP est non-bloquant dans le thread Qt (sendto immédiat).

    Format UDP :
        Le dict telemetrie (structure interne avec pos[], vel[], att_rad[])
        est aplati en JSON plat avant envoi, compatible avec parseJSON()
        dans telemetry_sender.cpp :
        {"alt":…,"vx":…,"vy":…,"vz":…,"roll":…,"pitch":…,"yaw":…,
         "bat_pct":…,"bat_v":…,"m0":…,"m1":…,"m2":…,"m3":…,
         "mode":<int>,"throttle":…,"dist":…}
    """

    def __init__(self, config: Optional[HilConfig] = None):
        self.cfg   = config or HilConfig()
        self.actif = False

        # ── Compteurs de diagnostic ──────────────────────────────────
        self._frame_count    = 0
        self._udp_sent       = 0
        self._udp_errors     = 0
        self._serial_sent    = 0
        self._serial_drops   = 0
        self._serial_errors  = 0
        self._t_dernier_log  = 0.0
        self._latence_udp_ms = 0.0   # mesure EWMA

        # ── UDP socket (non-bloquant) ────────────────────────────────
        self._sock_udp: Optional[socket.socket] = None

        # ── Serial ──────────────────────────────────────────────────
        self._serial_queue : queue.Queue         = queue.Queue(maxsize=self.cfg.queue_max)
        self._serial_thread: Optional[threading.Thread] = None
        self._serial_stop  = threading.Event()
        self._port_serial  = None   # pyserial.Serial, ouvert dans le thread

    # ------------------------------------------------------------------
    # Cycle de vie
    # ------------------------------------------------------------------

    def demarrer(self):
        """Ouvre les connexions. Appeler une fois dans MainWindow.__init__."""
        if self.actif:
            return

        if self.cfg.udp_actif:
            self._ouvrir_udp()

        if self.cfg.serial_actif:
            self._demarrer_serial()

        self.actif = True
        self._t_dernier_log = time.monotonic()
        log.info("HilBridge démarré — UDP=%s  Serial=%s",
                 self.cfg.udp_actif, self.cfg.serial_actif)

    def arreter(self):
        """Ferme proprement. Appeler dans MainWindow.closeEvent."""
        if not self.actif:
            return
        self.actif = False

        if self._sock_udp:
            try:
                self._sock_udp.close()
            except Exception:
                pass
            self._sock_udp = None

        if self._serial_thread and self._serial_thread.is_alive():
            self._serial_stop.set()
            self._serial_thread.join(timeout=2.0)

        log.info("HilBridge arrêté — %d trames envoyées", self._frame_count)

    # ------------------------------------------------------------------
    # API principale — appelée depuis _tick() à 50 Hz
    # ------------------------------------------------------------------

    def envoyer(self, telemetrie: dict):
        """
        Envoie la télémétrie sur les deux canaux actifs.
        Non-bloquant : retourne immédiatement même si un canal est indisponible.
        """
        if not self.actif:
            return

        self._frame_count += 1

        if self.cfg.udp_actif and self._sock_udp:
            self._envoyer_udp(telemetrie)

        if self.cfg.serial_actif:
            self._enqueuer_serial(telemetrie)

        self._log_stats_periodique()

    # ------------------------------------------------------------------
    # Propriétés de diagnostic (pour l'overlay HUD)
    # ------------------------------------------------------------------

    @property
    def stats(self) -> dict:
        """Snapshot des compteurs pour affichage dans le HUD ingénieur."""
        return {
            "frames"         : self._frame_count,
            "udp_sent"       : self._udp_sent,
            "udp_errors"     : self._udp_errors,
            "serial_sent"    : self._serial_sent,
            "serial_drops"   : self._serial_drops,
            "serial_errors"  : self._serial_errors,
            "latence_udp_ms" : round(self._latence_udp_ms, 2),
            "udp_ok"         : self._sock_udp is not None,
            "serial_ok"      : (self._port_serial is not None
                                and getattr(self._port_serial, "is_open", False)),
        }

    # ==================================================================
    # Privé — UDP
    # ==================================================================

    def _ouvrir_udp(self):
        try:
            s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            s.setblocking(False)
            self._sock_udp = s
            log.info("UDP socket ouvert → %s:%d", self.cfg.udp_host, self.cfg.udp_port)
        except OSError as e:
            log.error("UDP open failed: %s", e)
            self._sock_udp = None

    @staticmethod
    def _aplatir_telemetrie(t: dict) -> dict:
        """
        Convertit la structure interne (pos[], vel[], att_rad[], mode string)
        en dict plat compatible avec parseJSON() sur l'ESP32.

        Champs produits :
            alt, vx, vy, vz       — position z et vélocités
            roll, pitch, yaw      — attitudes en radians
            bat_pct, bat_v        — batterie
            m0, m1, m2, m3        — moteurs 0.0–1.0
            mode                  — entier (0=SOL … 4=URGENCE)
            throttle, dist        — commandes / capteur distance
        """
        pos = t.get("pos",     [0.0, 0.0, 0.0])
        vel = t.get("vel",     [0.0, 0.0, 0.0])
        att = t.get("att_rad", [0.0, 0.0, 0.0])
        mot = t.get("moteurs", [0.0, 0.0, 0.0, 0.0])

        # mode peut être une string ("SOL") ou déjà un int
        mode_raw  = t.get("mode", "SOL")
        mode_int  = MODE_CODE.get(mode_raw, mode_raw) if isinstance(mode_raw, str) \
                    else int(mode_raw)

        return {
            "alt"     : float(pos[2]),
            "vx"      : float(vel[0]),
            "vy"      : float(vel[1]),
            "vz"      : float(vel[2]),
            "roll"    : float(att[0]),
            "pitch"   : float(att[1]),
            "yaw"     : float(att[2]),
            "bat_pct" : float(t.get("bat_pct",  100.0)),
            "bat_v"   : float(t.get("bat_v",    12.6)),
            "m0"      : float(mot[0]),
            "m1"      : float(mot[1]),
            "m2"      : float(mot[2]),
            "m3"      : float(mot[3]),
            "mode"    : mode_int,           # ← entier, pas string
            "throttle": float(t.get("throttle", 0.0)),
            "dist"    : float(t.get("dist",     0.0)),
        }

    def _envoyer_udp(self, telemetrie: dict):
        try:
            t0 = time.monotonic()

            # Aplatir avant sérialisation JSON
            plat    = self._aplatir_telemetrie(telemetrie)
            payload = json.dumps(plat, separators=(',', ':')).encode()

            self._sock_udp.sendto(payload, (self.cfg.udp_host, self.cfg.udp_port))

            # EWMA latence (sérialisation + sendto)
            dt_ms = (time.monotonic() - t0) * 1000.0
            self._latence_udp_ms = 0.9 * self._latence_udp_ms + 0.1 * dt_ms
            self._udp_sent += 1

        except BlockingIOError:
            # Buffer réseau plein — drop silencieux, pas grave à 50 Hz
            self._udp_errors += 1
        except OSError as e:
            self._udp_errors += 1
            log.warning("UDP send error: %s", e)

    # ==================================================================
    # Privé — Serial
    # ==================================================================

    def _demarrer_serial(self):
        self._serial_stop.clear()
        self._serial_thread = threading.Thread(
            target=self._serial_worker,
            name="HIL-Serial",
            daemon=True
        )
        self._serial_thread.start()

    def _enqueuer_serial(self, telemetrie: dict):
        """
        Pousse dans la queue sans bloquer.
        Si la queue est pleine on drop la plus ancienne trame (pas la nouvelle).
        """
        trame = self._encoder_trame_binaire(telemetrie)
        try:
            self._serial_queue.put_nowait(trame)
        except queue.Full:
            # Vide la plus vieille, insère la nouvelle
            try:
                self._serial_queue.get_nowait()
            except queue.Empty:
                pass
            try:
                self._serial_queue.put_nowait(trame)
            except queue.Full:
                pass
            self._serial_drops += 1

    def _serial_worker(self):
        """Thread dédié Serial — tente d'ouvrir le port, puis vide la queue."""
        try:
            import serial as pyserial
        except ImportError:
            log.error("pyserial non installé — pip install pyserial")
            return

        while not self._serial_stop.is_set():
            # ── Ouverture / ré-ouverture du port ────────────────────
            if self._port_serial is None or not self._port_serial.is_open:
                try:
                    self._port_serial = pyserial.Serial(
                        port     = self.cfg.serial_port,
                        baudrate = self.cfg.serial_baud,
                        timeout  = self.cfg.serial_timeout
                    )
                    log.info("Serial ouvert : %s @ %d baud",
                             self.cfg.serial_port, self.cfg.serial_baud)
                except Exception as e:
                    log.warning("Serial open failed (%s) — retry in 3s", e)
                    self._serial_stop.wait(3.0)
                    continue

            # ── Envoi des trames en attente ──────────────────────────
            try:
                trame = self._serial_queue.get(timeout=0.05)
                self._port_serial.write(trame)
                self._serial_sent += 1
            except queue.Empty:
                pass
            except Exception as e:
                log.warning("Serial write error: %s — reconnexion", e)
                self._serial_errors += 1
                try:
                    self._port_serial.close()
                except Exception:
                    pass
                self._port_serial = None

        # Nettoyage à l'arrêt
        if self._port_serial and self._port_serial.is_open:
            try:
                self._port_serial.close()
            except Exception:
                pass

    # ------------------------------------------------------------------
    # Encodage trame binaire (46 octets) — canal Serial
    # ------------------------------------------------------------------

    @staticmethod
    def _encoder_trame_binaire(t: dict) -> bytes:
        """
        Encode la télémétrie en trame binaire compacte pour la carte.

        Format (46 octets) :
          2B header | 2B counter | 10×float32 (40B) | 1B mode | 1B XOR checksum
        """
        pos  = t.get("pos",     [0.0, 0.0, 0.0])
        vel  = t.get("vel",     [0.0, 0.0, 0.0])
        att  = t.get("att_rad", [0.0, 0.0, 0.0])
        tps  = float(t.get("t", 0.0))
        cnt  = int(t.get("_cnt", 0)) & 0xFFFF

        mode_raw = t.get("mode", "SOL")
        mode     = MODE_CODE.get(mode_raw, mode_raw) if isinstance(mode_raw, str) \
                   else int(mode_raw)

        # Pack : header(2) + counter(2) + 10 floats(40) + mode(1) = 45 octets
        body = struct.pack(
            "<BBH10fb",
            HEADER_A, HEADER_B,
            cnt,
            tps,
            float(pos[0]), float(pos[1]), float(pos[2]),
            float(vel[0]), float(vel[1]), float(vel[2]),
            float(att[0]), float(att[1]), float(att[2]),
            mode
        )

        # Checksum XOR sur les 45 premiers octets
        cs = 0
        for b in body:
            cs ^= b

        return body + bytes([cs & 0xFF])

    # ------------------------------------------------------------------
    # Stats périodiques
    # ------------------------------------------------------------------

    def _log_stats_periodique(self):
        now = time.monotonic()
        if now - self._t_dernier_log < self.cfg.log_interval_s:
            return
        self._t_dernier_log = now
        st = self.stats
        log.info(
            "HIL stats | frames=%d | UDP sent=%d err=%d lat=%.2fms "
            "| Serial sent=%d drops=%d err=%d",
            st["frames"], st["udp_sent"], st["udp_errors"],
            st["latence_udp_ms"], st["serial_sent"],
            st["serial_drops"], st["serial_errors"]
        )
