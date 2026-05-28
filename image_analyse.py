import cv2
from matplotlib import image
import numpy as np
import matplotlib.pyplot as plt

class image_Comparateur:
    def __init__(self, img1_path, img2_path):
        self.img1 = cv2.imread(img1_path)
        self.img2 = cv2.imread(img2_path)
        self.img2 = cv2.resize(self.img2, (self.img1.shape[1], self.img1.shape[0]))
        self.gray1 = cv2.cvtColor(self.img1, cv2.COLOR_BGR2GRAY)
        self.gray2 = cv2.cvtColor(self.img2, cv2.COLOR_BGR2GRAY)

    def comparer(self):

        # Calculer la différence absolue entre les deux images
        diff = cv2.absdiff(self.gray1, self.gray2)

        # Appliquer un seuillage pour mettre en évidence les différences
        _, thresh = cv2.threshold(diff, 40, 255, cv2.THRESH_BINARY)
        contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        img1_annotee = self.img1.copy()
        img2_annotee = self.img2.copy()
        for contour in contours:
            aire = cv2.contourArea(contour)
            if aire > 300:  # Ignorer les micro-différences (bruit)
               
                x, y, w, h = cv2.boundingRect(contour)
                cv2.rectangle(img1_annotee, (x, y), (x+w, y+h), (0, 255,0 ), 2)
                cv2.rectangle(img2_annotee, (x, y), (x+w, y+h), (0, 0, 255), 2)

        plt.figure(figsize=(12, 8))
        plt.subplot(1, 2, 1)
        plt.imshow(cv2.cvtColor(img1_annotee, cv2.COLOR_BGR2RGB))
        plt.title('Image 1 Annotée')
        plt.axis('off')
        

        plt.subplot(1, 2, 2)
        plt.imshow(cv2.cvtColor(img2_annotee, cv2.COLOR_BGR2RGB))
        plt.title('Image 2 Annotée')
        plt.axis('off')
        plt.show()


    def detect_faces(self):

    # Charger le classifieur de visage pré-entraîné
        ref = cv2.CascadeClassifier( cv2.data.haarcascades + "haarcascade_fullbody.xml")
        ref1 = cv2.CascadeClassifier( cv2.data.haarcascades + "haarcascade_profileface.xml")

    # Détecter les visages
        visage = ref.detectMultiScale(self.gray2, scaleFactor=1.1, minNeighbors=10)
        visage1 = ref1.detectMultiScale(self.gray2, scaleFactor=1.1, minNeighbors=10)

        if len(visage) == 0 and len(visage1) == 0:
            print("Aucun visage détecté.") 
            return None
        else :
            for (x, y, w, h) in visage:
                cv2.rectangle(self.img2, (x, y), (x + w, y + h), (0, 255, 0), 2)
            for (x, y, w, h) in visage1:
                cv2.rectangle(self.img2, (x, y), (x + w, y + h), (255, 0, 0), 2)

# Afficher l'image avec les visages détectés
        cv2.imshow("Visages détectés", self.img2)
        cv2.waitKey(0)
        cv2.destroyAllWindows()


# --- Exemple d'utilisation ---
if __name__ == "__main__":
    img1_path = "test.png"
    img2_path = "test1.png" 
    comparateur = image_Comparateur(img1_path, img2_path)
    comparateur.comparer()
    comparateur.detect_faces()
