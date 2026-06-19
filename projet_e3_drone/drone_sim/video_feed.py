# video_feed.py
# Capture webcam dans un thread dédié et expose la dernière frame
# prête à être dessinée dans le HUD Qt (QImage).

import threading
import cv2
import numpy as np
from PyQt6.QtGui import QImage


class VideoFeed:
    """
    Lance la capture webcam dans un thread daemon.
    Expose :
      - derniere_qimage()  → QImage | None   (pour QPainter dans le HUD)
      - derniere_frame()   → np.ndarray | None (pour YOLO)
      - actif              → bool
    """

    def __init__(self, source: int = 0):
        self._source   = source
        self._lock     = threading.Lock()
        self._frame    : np.ndarray | None = None
        self._qimage   : QImage | None     = None
        self._running  = False
        self._thread   : threading.Thread | None = None
        self.actif     = False            # True dès que la première image arrive

    # ------------------------------------------------------------------
    def demarrer(self):
        self._running = True
        self._thread  = threading.Thread(target=self._boucle, daemon=True)
        self._thread.start()

    def arreter(self):
        self._running = False

    # ------------------------------------------------------------------
    def _boucle(self):
        cap = cv2.VideoCapture(self._source)
        if not cap.isOpened():
            print(f"[VideoFeed] Impossible d'ouvrir la source {self._source}")
            return

        # Résolution réduite pour ne pas peser sur le HUD
        cap.set(cv2.CAP_PROP_FRAME_WIDTH,  640)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

        while self._running:
            ret, frame = cap.read()
            if not ret:
                continue

            # Convertir BGR → RGB pour Qt
            rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            h, w, ch = rgb.shape
            qi = QImage(rgb.data, w, h, ch * w, QImage.Format.Format_RGB888).copy()

            with self._lock:
                self._frame  = frame.copy()   # BGR pour YOLO
                self._qimage = qi
                self.actif   = True

        cap.release()

    # ------------------------------------------------------------------
    def derniere_qimage(self) -> QImage | None:
        with self._lock:
            return self._qimage

    def derniere_frame(self) -> np.ndarray | None:
        with self._lock:
            return self._frame.copy() if self._frame is not None else None