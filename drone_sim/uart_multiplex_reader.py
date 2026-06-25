# =============================================================================
# uart_multiplex_reader.py — Lecteur du flux UART multiplexé depuis ESP32 #1
#
# Protocole (voir telemetry_sender_v3.cpp côté ESP32) :
#   En-tête 6 octets : [type(1)] [taille uint32 little-endian (4)] ['\n'(1)]
#   type = 'T' → payload = JSON texte (RC relayées par ESP32 #1)
#   type = 'I' → payload = JPEG brut (image caméra)
#
# Utilisation typique dans main_combined.py :
#
#   from uart_multiplex_reader import UartMultiplexReader
#   reader = UartMultiplexReader(port="COM7")
#   reader.demarrer()
#   ...
#   frame = reader.derniere_image()   # np.ndarray (BGR, prêt pour OpenCV/YOLO) ou None
#   rc    = reader.derniere_rc()      # dict JSON ou None
# =============================================================================

import threading
import struct
import json
import numpy as np
import cv2
import serial   # faites  pip install pyserial   dans le terminal 


HEADER_LEN = 6
TYPE_TELEM = ord('T')
TYPE_IMAGE = ord('I')


class UartMultiplexReader:
    """
    Lit en continu (thread dédié) le port série partagé avec ESP32 #1,
    et démultiplexe télémétrie JSON / images JPEG selon le protocole défini
    côté firmware (telemetry_sender_v3.cpp).

    Cette classe est indépendante de HilBridgeSerial (qui ÉCRIT sur le même
    port). Si les deux utilisent le même port, ouvrir une seule fois la
    connexion série et la partager — voir note en bas de fichier.
    """

    def __init__(self, port: str = "COM7", baud: int = 115200, ser: serial.Serial = None):
        self.port = port
        self.baud = baud

        # Permet de réutiliser une connexion série déjà ouverte
        # (ex. partagée avec HilBridgeSerial) plutôt que d'en ouvrir une 2e.
        self._ser_externe = ser is not None
        self._ser = ser

        self._actif  = False
        self._thread = None
        self._lock   = threading.Lock()

        self._derniere_image = None   # np.ndarray BGR ou None
        self._derniere_rc    = None   # dict ou None

        self._frames_recues  = 0
        self._erreurs_jpeg   = 0

    # ------------------------------------------------------------------

    def demarrer(self):
        if not self._ser_externe:
            try:
                self._ser = serial.Serial(self.port, self.baud, timeout=0.5)
                print(f"[UartMux] Connecté sur {self.port} @ {self.baud}")
            except serial.SerialException as e:
                print(f"[UartMux] ERREUR ouverture port : {e}")
                return

        self._actif  = True
        self._thread = threading.Thread(target=self._boucle_lecture, daemon=True)
        self._thread.start()

    def arreter(self):
        self._actif = False
        if self._thread:
            self._thread.join(timeout=1.0)
        if self._ser and self._ser.is_open and not self._ser_externe:
            self._ser.close()
        print("[UartMux] Arrêté.")

    # ------------------------------------------------------------------
    # Accès thread-safe aux dernières données reçues
    # ------------------------------------------------------------------

    def derniere_image(self):
        with self._lock:
            return None if self._derniere_image is None else self._derniere_image.copy()

    def derniere_rc(self):
        with self._lock:
            return self._derniere_rc

    @property
    def stats(self):
        return {
            "frames_recues": self._frames_recues,
            "erreurs_jpeg":  self._erreurs_jpeg,
        }

    # ------------------------------------------------------------------
    # Boucle de lecture (thread)
    # ------------------------------------------------------------------

    def _lire_exact(self, n: int) -> bytes:
        """Lit exactement n octets, en accumulant si nécessaire."""
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

                # Garde-fou : taille déraisonnable → désync, on resynchronise
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
                print(f"[UartMux] Erreur lecture : {e}")
                break

    def _traiter_telem(self, payload: bytes):
        try:
            obj = json.loads(payload.decode("utf-8", errors="ignore"))
            with self._lock:
                self._derniere_rc = obj
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
# Test standalone — affiche le flux reçu avec OpenCV
# =============================================================================
if __name__ == "__main__":
    import time

    reader = UartMultiplexReader(port="COM7")
    reader.demarrer()

    print("Lecture en cours... 'q' pour quitter.")
    try:
        while True:
            img = reader.derniere_image()
            if img is not None:
                cv2.imshow("ESP32-CAM via UART", img)

            rc = reader.derniere_rc()
            if rc:
                print(f"RC reçu : {rc}", end="\r")

            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
            time.sleep(0.01)
    except KeyboardInterrupt:
        pass
    finally:
        reader.arreter()
        cv2.destroyAllWindows()
        print(f"\nStats : {reader.stats}")