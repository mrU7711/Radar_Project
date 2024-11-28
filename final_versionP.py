import numpy as np
from scipy.optimize import fsolve
import serial
import time
import matplotlib.pyplot as plt

# Configuration du port série pour communiquer avec l'Arduino
ser = serial.Serial('COM5', 9600, timeout=1)
time.sleep(2)  # Attendre l'initialisation de la communication série

# Coordonnées des microphones (en mètres) par rapport à l'origine
xn = np.array([0, 0.265, 0.265, 0])
yn = np.array([0, 0.175, 0, 0.175])
zn = np.array([0, 0, 0.3, 0.3])

# Position des servos (en mètres) par rapport à l'origine
servo_position_azimuth = np.array([0.135, 0.27, 0.03])  # Position du servo d'azimut
servo_position_elevation = np.array([0.085, 0.27, 0.075])  # Position du servo d'élévation

# Vitesse du son en m/s
c = 343

# Nombre d'acquisitions à effectuer
num_acquisitions = 5
positions = []

# Préparer la visualisation en temps réel
plt.ion()
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
ax.set_xlim([-5, 5])
ax.set_ylim([-5, 5])
ax.set_zlim([-5, 5])
ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')
ax.set_title('Position de la Source Sonore')

# Fonction pour convertir les temps de détection en secondes
def microseconds_to_seconds(microseconds):
    return microseconds / 1_000_000

# Fonction représentant le système d'équations pour la localisation TDOA
def equations(vars, t, c, xn, yn, zn):
    x, y, z, t_s = vars
    equations = []
    for i in range(1, len(xn)):
        equations.append(np.sqrt((x - xn[i])**2 + (y - yn[i])**2 + (z - zn[i])**2) - np.sqrt((x - xn[0])**2 + (y - yn[0])**2 + (z - zn[0])**2) - c * (t[i] - t[0]))
    return equations

# Fonction pour calculer les angles d'azimut et d'élévation à partir des coordonnées (x, y, z)
def calculate_angles(x, y, z, servo_position_azimuth, servo_position_elevation):
    # Calculer les coordonnées par rapport aux servos
    x_az = x - servo_position_azimuth[0]
    y_az = y - servo_position_azimuth[1]
    z_az = z - servo_position_azimuth[2]

    x_el = x - servo_position_elevation[0]
    y_el = y - servo_position_elevation[1]
    z_el = z - servo_position_elevation[2]

    # Calculer les angles d'azimut et d'élévation
    azimuth = np.degrees(np.arctan2(y_az, x_az))
    elevation = np.degrees(np.arctan2(z_el, np.sqrt(x_el**2 + y_el**2)))

    return azimuth, elevation

# Lire les temps de détection depuis l'Arduino
def read_detection_times():
    line = ser.readline().decode('utf-8').strip()
    if line.startswith("ARRAY"):
        times_str = line.split("[")[1].split("]")[0].split(", ")
        return [int(t) for t in times_str]
    return None

# Envoyer les angles à l'Arduino
def send_angles_to_arduino(azimuth, elevation):
    command = f"{azimuth:.2f},{elevation:.2f}\n"
    ser.write(command.encode())

# Mettre à jour la visualisation en temps réel
def update_plot(positions, barycenter=None):
    ax.clear()
    ax.set_xlim([-5, 5])
    ax.set_ylim([-5, 5])
    ax.set_zlim([-5, 5])
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    ax.set_title('Position de la Source Sonore')

    if positions:
        positions = np.array(positions)
        ax.scatter(positions[:, 0], positions[:, 1], positions[:, 2], color='b', label='Positions Acquises')

    if barycenter is not None:
        ax.scatter(barycenter[0], barycenter[1], barycenter[2], color='r', label='Barycentre', s=100)

    plt.legend()
    plt.draw()
    plt.pause(0.01)

accepted_measurements = 0
rejected_measurements = 0

while True:
    detection_times = read_detection_times()
    if detection_times:
        t = np.array([microseconds_to_seconds(t) for t in detection_times])
        initial_guess = [0, 0, 0, 0]
        solution = fsolve(equations, initial_guess, args=(t, c, xn, yn, zn))
        x, y, z, t_s = solution

        # Calculer la distance entre l'origine et la source du son
        distance = np.sqrt(x**2 + y**2 + z**2)

        # Vérifier si la distance est entre 0.5 et 3.5 mètres
        if 0.5 <= distance <= 3.5:
            positions.append([x, y, z])
            accepted_measurements += 1
            print(f"Position accepted: {x:.2f}, {y:.2f}, {z:.2f} (Distance: {distance:.2f} m)")
        else:
            rejected_measurements += 1
            print(f"Position rejected: {x:.2f}, {y:.2f}, {z:.2f} (Distance: {distance:.2f} m)")

        update_plot(positions)

        if len(positions) >= num_acquisitions:
            positions = positions[-num_acquisitions:]  # Garder les dernières acquisitions
            barycenter = np.mean(positions, axis=0)
            azimuth, elevation = calculate_angles(barycenter[0], barycenter[1], barycenter[2], servo_position_azimuth, servo_position_elevation)
            print(f"Barycenter: {barycenter}, Azimuth: {azimuth:.2f}, Elevation: {elevation:.2f}")
            send_angles_to_arduino(azimuth, elevation)
            positions = []

        print(f"Accepted measurements: {accepted_measurements}, Rejected measurements: {rejected_measurements}")
        print(f"Remaining measurements: {num_acquisitions - len(positions)}")
