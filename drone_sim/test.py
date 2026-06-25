import cv2
import numpy as np
from ultralytics import YOLO

# Chargement du modèle une seule fois au niveau module
_model = YOLO("yolov8n.pt")

# URL du flux vidéo ESP32-CAM
# Tester ces URLs selon le firmware de ta caméra :
# MJPEG stream : http://192.168.4.3/stream  ou  http://192.168.4.3:81/stream
# JPEG snapshot : http://192.168.4.3/capture
STREAM_URL = "http://192.168.4.3:81/stream"   #  ajuster si besoin ça peut etre 2: ou autre a verifier lors de k implementation


class image_Comparateur:
    def __init__(self, img):
        self.img2  = img
        self.gray2 = cv2.cvtColor(self.img2, cv2.COLOR_BGR2GRAY)

    def detect_personne(self):
        """
        Détecte les personnes dans l'image.
        Retourne (True, confiance_max_%) si au moins une personne est trouvée,
        (False, 0.0) sinon.
        """
        confiance_max    = 0.0
        personne_trouvee = False

        results = _model(self.img2, verbose=False)  # verbose=False → moins de spam console

        for result in results:
            for box in result.boxes:
                if int(box.cls[0]) == 0:            # classe 0 = personne
                    conf = box.conf[0].item()
                    if conf > confiance_max:
                        confiance_max = conf
                    personne_trouvee = True

                    x1, y1, x2, y2 = map(int, box.xyxy[0])
                    cv2.rectangle(self.img2, (x1, y1), (x2, y2), (0, 255, 0), 2)
                    label = f"Personne {conf*100:.1f}%"
                    cv2.putText(self.img2, label, (x1, max(y1 - 8, 0)),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)

        return personne_trouvee, confiance_max * 100

    def affiche(self):
        cv2.imshow("Camera drone", self.img2)


# ---------------------------------------------------------------------------
#  Lecture robuste d'un flux MJPEG HTTP (ESP32-CAM / caméra IP)
# ---------------------------------------------------------------------------
def ouvrir_flux(url: str, tentatives: int = 5) -> cv2.VideoCapture:
    """Tente d'ouvrir le flux HTTP, réessaie en cas d'échec."""
    for i in range(tentatives):
        cap = cv2.VideoCapture(url)
        if cap.isOpened():
            print(f"Flux ouvert : {url}")
            return cap
        print(f"Tentative {i+1}/{tentatives} échouée — nouvelle tentative...")
        cv2.waitKey(1000)
    return None


def lire_frame_robuste(cap: cv2.VideoCapture, url: str) -> tuple[bool, np.ndarray | None, cv2.VideoCapture]:
    """
    Lit une frame. Si la lecture échoue, tente de rouvrir le flux.
    Retourne (succès, frame_ou_None, capture_mise_a_jour).
    """
    ret, frame = cap.read()
    if ret:
        return True, frame, cap

    print("Perte du flux — tentative de reconnexion...")
    cap.release()
    cap = ouvrir_flux(url)
    if cap is None:
        return False, None, cap

    ret, frame = cap.read()
    return ret, frame if ret else None, cap


# ---------------------------------------------------------------------------
if __name__ == "__main__":
    video = ouvrir_flux(STREAM_URL)

    if video is None:
        print(f"Erreur : impossible d'ouvrir le flux après plusieurs tentatives.\n"
              f"URLs à tester :\n"
              f"  http://192.168.4.3/stream\n"
              f"  http://192.168.4.3:81/stream\n"
              f"  http://192.168.4.3/capture  (snapshot JPEG)")
    else:
        print("Démarrage de la détection — appuie sur 'q' pour quitter.")
        erreurs_consecutives = 0
        MAX_ERREURS = 10

        while True:
            ok, img, video = lire_frame_robuste(video, STREAM_URL)

            if not ok or img is None:
                erreurs_consecutives += 1
                print(f"Échec lecture ({erreurs_consecutives}/{MAX_ERREURS})")
                if erreurs_consecutives >= MAX_ERREURS:
                    print("Trop d'erreurs consécutives — arrêt.")
                    break
                continue

            erreurs_consecutives = 0  # reset si la frame est bonne

            comparateur = image_Comparateur(img)
            trouve, conf = comparateur.detect_personne()

            if trouve:
                print(f"Personne détectée — confiance max : {conf:.1f}%")

            comparateur.affiche()

            if cv2.waitKey(1) & 0xFF == ord('q'):
                print("Arrêt demandé.")
                break

        if video:
            video.release()
        cv2.destroyAllWindows()