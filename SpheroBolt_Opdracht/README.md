# IntelligentRobotics2025
#  Sphero BOLT – Automated Track Race (Raspberry Pi)

##  Installatie & Uitvoerinstructies

1. **Clone de repository en open de Sphero-map**
   ```bash
   git clone https://github.com/EnwinDang/IntelligentRobotics2025.git
   cd IntelligentRobotics2025/sphero
   ```

2. **Maak en activeer een virtuele omgeving**
   ```bash
   python -m venv venv
   ```
   - **Windows:**  
     ```bash
     venv\Scripts\activate
     ```
   - **Linux/macOS (of Raspberry Pi):**  
     ```bash
     source venv/bin/activate
     ```

3. **Installeer de vereiste pakketten**
   ```bash
   pip install -r requirements.txt
   ```

4. **Verbind de Sphero BOLT via Bluetooth**
   - Zet de BOLT aan en noteer de naam (bijv. `SB-27A5`).
   - Pair via het Bluetooth-menu of laat het script zelf scannen.

5. **Voer het script uit**
   ```bash
   python3 sphero_race.py <toy_name> <joystickIndex> <playerNumber>
   ```
   Voorbeeld:
   ```bash
   python3 sphero_race.py SB-27A5 0 1
   ```

---

## 🎮 Joystickbediening
| Actie | Functie |
|--------|----------|
| **SELECT** | Kalibratie aan/uit |
| **START** | Start automatische race |
| **Knoppen 1–4** | Snelheid (50 / 70 / 100 / 200) |
| **Joystick** | Handmatige besturing (vooruit, achteruit, links, rechts) |

---

## 🧭 Kalibratiestappen

1. Plaats de Sphero op de **startpositie** (finishlijn bovenaan in het midden).  
2. Richt de robot zodat hij **naar rechts (oost)** kijkt → dit is **0° heading**.  
3. Druk op **SELECT** om kalibratiemodus te starten (LED = roze).  
4. Gebruik de joystick links/rechts om richting te corrigeren.  
5. Druk nogmaals op **SELECT** om te bevestigen (LED = groen).  
6. Druk op **START** om de automatische race te starten.

---

## 🧠 Beschrijving van de aanpak

- De baan is opgebouwd uit **segmenten (waypoints)** die elk bestaan uit:
  - `panels`: aantal tegels van 50 cm  
  - `angle`: de richting (heading in graden)
  
  | Richting | Graden |
  |-----------|--------|
  | Rechts (Oost) | 0° |
  | Omhoog (Noord) | 90° |
  | Links (West) | 180° |
  | Omlaag (Zuid) | 270° |

- Het script vertaalt deze segmenten naar `(hoek, snelheid, duur)` via de functies  
  `make_moves()` en `time_for_distance()`.

- De route (`TRACK_PATH`) vormt het pad dat de Sphero autonoom aflegt op het parcours.  

- **Fail-safe:** bij een fout of verbroken verbinding stopt de robot automatisch  
  (`set_speed(0)`) en kleurt rood (veiligheidsstatus).

---

## ⚡ Snelheidsprofiel

- **TRACK_SPEED = 120** – standaard snelheid voor de automatische race  
- **Handmatige snelheden** via joystickknoppen (50 / 70 / 100 / 200)  
- De afstand/snelheid kan worden afgesteld via `SPEED_FACTOR` (0.85 standaard)

---

> 🎯 **Resultaat:** De Sphero rijdt autonoom het parcours af aan de hand van vooraf bepaalde segmenten, met correcte oriëntatie op basis van kalibratie en automatische fail-safe voor veiligheid.

De path is gebaseerd op onderstaande map (50×50 cm. Totale afmeting: 5 m × 3 m).

<img width="1607" height="947" alt="sphero_Parcour" src="https://github.com/user-attachments/assets/c6340f26-377d-455a-afe2-8b3c57670ddc" />
