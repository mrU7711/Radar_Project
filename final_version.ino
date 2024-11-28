#include <Servo.h>

// Définir les broches pour les servos et le moteur DC
Servo servoAzimuth;
Servo servoElevation;
const int servoPinAzimuth = 9;
const int servoPinElevation = 10;
const int motorPin = 8; // Broche de contrôle du moteur DC

// Définir les broches pour les microphones
const int microphonePinA = A0;
const int microphonePinB = A1;
const int microphonePinC = A2;
const int microphonePinD = A3;

// Déclarer des variables pour stocker les lectures des microphones
int readingA, readingB, readingC, readingD;
unsigned long voiceDetectedTime[4] = {0};

// Buffers pour stocker les lectures de chaque microphone
const int shortTermBufferSize = 5; // Réduire la taille pour des détections plus rapides
const int longTermBufferSize = 20; // Tampon à long terme réduit pour réagir plus rapidement aux changements
int shortTermBufferA[shortTermBufferSize], shortTermBufferB[shortTermBufferSize], shortTermBufferC[shortTermBufferSize], shortTermBufferD[shortTermBufferSize];
int longTermBufferA[longTermBufferSize], longTermBufferB[longTermBufferSize], longTermBufferC[longTermBufferSize], longTermBufferD[longTermBufferSize];
int shortTermBufferIndex = 0, longTermBufferIndex = 0;

// Variables d'état de détection
bool detectingA = true, detectingB = true, detectingC = true, detectingD = true;
float baselineA = 0, baselineB = 0, baselineC = 0, baselineD = 0;
unsigned long detectionStartTime = 0;
const unsigned long detectionDuration = 1000000; // 1 seconde en microsecondes pour une réaction rapide
const unsigned long initialCollectionDuration = 2000000; // 2 secondes en microsecondes pour collecte initiale
unsigned long startTime;

// Variables pour le contrôle du moteur
bool motorActivated = false;
unsigned long motorStartTime = 0;
const unsigned long motorDuration = 1000000; // 1 seconde en microsecondes

void setup() {
    // Initialiser les broches des microphones
    pinMode(microphonePinA, INPUT);
    pinMode(microphonePinB, INPUT);
    pinMode(microphonePinC, INPUT);
    pinMode(microphonePinD, INPUT);

    // Initialiser la broche du moteur
    pinMode(motorPin, OUTPUT);
    digitalWrite(motorPin, LOW); // S'assurer que le moteur est éteint au démarrage

    // Initialiser la communication série pour le débogage
    Serial.begin(9600);

    // Initialiser les servos
    servoAzimuth.attach(servoPinAzimuth);
    servoElevation.attach(servoPinElevation);

    // Initialiser le temps de début
    startTime = micros();
}

void loop() {
    // Si le moteur est activé, ne pas collecter de nouvelles données
    if (motorActivated) {
        // Vérifier si le temps de désactivation du moteur est atteint
        if (micros() - motorStartTime > motorDuration) {
            digitalWrite(motorPin, LOW);
            motorActivated = false;
        }
        return;
    }

    // Lire les signaux des microphones
    readingA = analogRead(microphonePinA);
    readingB = analogRead(microphonePinB);
    readingC = analogRead(microphonePinC);
    readingD = analogRead(microphonePinD);

    // Mettre à jour les tampons
    shortTermBufferA[shortTermBufferIndex] = readingA;
    shortTermBufferB[shortTermBufferIndex] = readingB;
    shortTermBufferC[shortTermBufferIndex] = readingC;
    shortTermBufferD[shortTermBufferIndex] = readingD;

    longTermBufferA[longTermBufferIndex] = readingA;
    longTermBufferB[longTermBufferIndex] = readingB;
    longTermBufferC[longTermBufferIndex] = readingC;
    longTermBufferD[longTermBufferIndex] = readingD;

    shortTermBufferIndex = (shortTermBufferIndex + 1) % shortTermBufferSize;
    longTermBufferIndex = (longTermBufferIndex + 1) % longTermBufferSize;

    // Calculer les moyennes
    float shortTermAvgA = calculateAverage(shortTermBufferA, shortTermBufferSize);
    float shortTermAvgB = calculateAverage(shortTermBufferB, shortTermBufferSize);
    float shortTermAvgC = calculateAverage(shortTermBufferC, shortTermBufferSize);
    float shortTermAvgD = calculateAverage(shortTermBufferD, shortTermBufferSize);

    float longTermAvgA = calculateAverage(longTermBufferA, longTermBufferSize);
    float longTermAvgB = calculateAverage(longTermBufferB, longTermBufferSize);
    float longTermAvgC = calculateAverage(longTermBufferC, longTermBufferSize);
    float longTermAvgD = calculateAverage(longTermBufferD, longTermBufferSize);

    unsigned long currentTime = micros();

    // Phase de collecte initiale de données pendant 2 secondes
    if (currentTime - startTime < initialCollectionDuration) {
        return;
    }

    // Détecter le son en fonction de la détection initiale et maintenir le seuil

    // Microphone A
    if (detectingA && readingA > shortTermAvgA + 7) { // Si detectingA est vrai, cela signifie que nous recherchons la première détection
        voiceDetectedTime[0] = currentTime;            // Enregistrer le temps de détection
        baselineA = longTermAvgA;                      // Mettre à jour la baseline
        detectingA = false;                            // Passer à false pour indiquer que la détection initiale est faite
    }

    // Microphone B
    if (detectingB && readingB > shortTermAvgB + 7) {
        voiceDetectedTime[1] = currentTime;
        baselineB = longTermAvgB;
        detectingB = false;
    }

    // Microphone C
    if (detectingC && readingC > shortTermAvgC + 7) {
        voiceDetectedTime[2] = currentTime;
        baselineC = longTermAvgC;
        detectingC = false;
    }

    // Microphone D
    if (detectingD && readingD > shortTermAvgD + 7) {
        voiceDetectedTime[3] = currentTime;
        baselineD = longTermAvgD;
        detectingD = false;
    }

    // Maintenir la détection pour une certaine durée

    // Microphone A
    if (!detectingA && readingA > baselineA) { // Si detectingA est false, cela signifie que nous maintenons la détection
        voiceDetectedTime[0] = currentTime;    // Mettre à jour le temps de détection
    }

    // Microphone B
    if (!detectingB && readingB > baselineB) {
        voiceDetectedTime[1] = currentTime;
    }

    // Microphone C
    if (!detectingC && readingC > baselineC) {
        voiceDetectedTime[2] = currentTime;
    }

    // Microphone D
    if (!detectingD && readingD > baselineD) {
        voiceDetectedTime[3] = currentTime;
    }

    // Réinitialiser la logique de détection après la durée
    if (currentTime - detectionStartTime > detectionDuration) {
        detectingA = detectingB = detectingC = detectingD = true; // Réinitialiser les variables d'état
        detectionStartTime = currentTime; // Réinitialiser le temps de début de détection
    }

    // Envoyer les temps de détection une fois que tous sont capturés
    if (voiceDetectedTime[0] != 0 && voiceDetectedTime[1] != 0 && voiceDetectedTime[2] != 0 && voiceDetectedTime[3] != 0) {
        // Vérifier que les temps de détection sont dans une plage raisonnable
        unsigned long minTime = voiceDetectedTime[0];
        unsigned long maxTime = voiceDetectedTime[0];
        for (int i = 1; i < 4; i++) {
            if (voiceDetectedTime[i] < minTime) minTime = voiceDetectedTime[i];
            if (voiceDetectedTime[i] > maxTime) maxTime = voiceDetectedTime[i];
        }
        if (maxTime - minTime < 100000) { // Si la différence est raisonnable (moins de 100 ms)
            Serial.print("ARRAY: [");
            Serial.print(voiceDetectedTime[0]);
            Serial.print(", ");
            Serial.print(voiceDetectedTime[1]);
            Serial.print(", ");
            Serial.print(voiceDetectedTime[2]);
            Serial.print(", ");
            Serial.print(voiceDetectedTime[3]);
            Serial.println("]");
            // Réinitialiser les temps après l'envoi
            for (int i = 0; i < 4; i++) {
                voiceDetectedTime[i] = 0;
            }
            // Réinitialiser les variables d'état pour éviter les détections multiples
            detectingA = detectingB = detectingC = detectingD = true;
        }
    }

    // Vérifier si des données sont disponibles sur le port série
    if (Serial.available() > 0) {
        // Lire la ligne de données
        String data = Serial.readStringUntil('\n');
        
        // Diviser la ligne en azimut et élévation
        int commaIndex = data.indexOf(',');
        if (commaIndex > 0) {
            String azimuthStr = data.substring(0, commaIndex);
            String elevationStr = data.substring(commaIndex + 1);
            
            // Convertir les chaînes en nombres
            float azimuth = azimuthStr.toFloat();
            float elevation = elevationStr.toFloat();
            
            // Contrainte des angles entre 0 et 180 degrés pour les servos
            azimuth = constrain(azimuth, 0, 180);
            elevation = constrain(elevation, 0, 180);
            
            // Déplacer les servos aux angles reçus
            servoAzimuth.write(azimuth);
            servoElevation.write(elevation);

            // Activer le moteur pour tirer
            digitalWrite(motorPin, HIGH);
            motorStartTime = micros();
            motorActivated = true;
        }
    }
}

// Fonction pour calculer la moyenne d'un tableau
float calculateAverage(int* buffer, int bufferSize) {
    long sum = 0;
    for (int i = 0; i < bufferSize; i++) {
        sum += buffer[i];
    }
    return sum / (float)bufferSize;
}
