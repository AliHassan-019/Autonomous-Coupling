import RTIMU
import math
import time
import pandas as pd
from datetime import datetime
import xlsxwriter

# Chemin de sauvegarde
excel_file_path = "orientation_relative_data.xlsx"

# Initialisation des deux IMU
settings1 = RTIMU.Settings("RTIMULib_imu1.ini")
imu1 = RTIMU.RTIMU(settings1)
settings2 = RTIMU.Settings("RTIMULib_imu2.ini")
imu2 = RTIMU.RTIMU(settings2)

if not imu1.IMUInit():
    raise RuntimeError("IMU 1 Init Failed")
if not imu2.IMUInit():
    raise RuntimeError("IMU 2 Init Failed")

for imu in (imu1, imu2):
    imu.setSlerpPower(1)
    imu.setGyroEnable(True)
    imu.setAccelEnable(True)
    imu.setCompassEnable(True)

data = []
start_time = time.time()
duration = 50  # secondes

print("Mesure en cours...\n")
while time.time() - start_time < duration:
    timestamp = datetime.now().strftime("%H:%M:%S.%f")[:-3]

    if imu1.IMURead() and imu2.IMURead():
        d1 = imu1.getIMUData()
        d2 = imu2.getIMUData()

        roll1, pitch1, yaw1 = [angle * 180 / math.pi for angle in d1["fusionPose"]]
        roll2, pitch2, yaw2 = [angle * 180 / math.pi for angle in d2["fusionPose"]]

        # Orientation relative (IMU1 - IMU2)
        rel_roll = roll1 - roll2
        rel_pitch = pitch1 - pitch2
        rel_yaw = yaw1 - yaw2

        data.append([
            timestamp,
            roll1, pitch1, yaw1,
            roll2, pitch2, yaw2,
            rel_roll, rel_pitch, rel_yaw
        ])
    time.sleep(0.01)

# Création du DataFrame
columns = [
    "Temps",
    "IMU1 Roll", "IMU1 Pitch", "IMU1 Yaw",
    "IMU2 Roll", "IMU2 Pitch", "IMU2 Yaw",
    "Relatif Roll", "Relatif Pitch", "Relatif Yaw"
]

df = pd.DataFrame(data, columns=columns)

# Moyennes
mean_roll = df["Relatif Roll"].mean()
mean_pitch = df["Relatif Pitch"].mean()
mean_yaw = df["Relatif Yaw"].mean()

print("\n--- MOYENNES DES ORIENTATIONS RELATIVES ---")
print(f"Moyenne Roll relatif : {mean_roll:.2f}°")
print(f"Moyenne Pitch relatif : {mean_pitch:.2f}°")
print(f"Moyenne Yaw relatif : {mean_yaw:.2f}°")

# Sauvegarde Excel
writer = pd.ExcelWriter(excel_file_path, engine='xlsxwriter')
df.to_excel(writer, index=False, sheet_name='Données')
workbook = writer.book
worksheet = writer.sheets['Données']

# Fonction pour ajouter un graphique
def add_chart(title, x_col, y1_col, y2_col=None, cell="J2", color1="#1f77b4", color2="#ff7f0e"):
    chart = workbook.add_chart({'type': 'line'})
    chart.add_series({
        'name': f'=Données!${y1_col}$1',
        'categories': f'=Données!${x_col}$2:${x_col}${len(df)+1}',
        'values': f'=Données!${y1_col}$2:${y1_col}${len(df)+1}',
        'line': {'width': 1.0, 'color': color1}
    })
    if y2_col:
        chart.add_series({
            'name': f'=Données!${y2_col}$1',
            'categories': f'=Données!${x_col}$2:${x_col}${len(df)+1}',
            'values': f'=Données!${y2_col}$2:${y2_col}${len(df)+1}',
            'line': {'width': 1.0, 'color': color2}
        })
    chart.set_title({'name': title})
    chart.set_x_axis({'name': 'Temps', 'label_position': 'low'})
    chart.set_y_axis({'name': 'Angle (°)'})
    chart.set_style(10)
    worksheet.insert_chart(cell, chart)

# Graphiques IMU1 vs IMU2
add_chart("Roll IMU1 vs IMU2", 'A', 'B', 'E', 'J2')
add_chart("Pitch IMU1 vs IMU2", 'A', 'C', 'F', 'J18')
add_chart("Yaw IMU1 vs IMU2", 'A', 'D', 'G', 'J34')

# Graphiques relatifs
add_chart("Roll relatif", 'A', 'H', None, 'J50')
add_chart("Pitch relatif", 'A', 'I', None, 'J66')
add_chart("Yaw relatif", 'A', 'J', None, 'J82')

writer.close()
print(f"\n✅ Données enregistrées dans : {excel_file_path}")
