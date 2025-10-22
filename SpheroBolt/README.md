# IntelligentRobotics2025
# Sphero BOLT – Automated Track Race (Raspberry Pi)

## Uitvoerinstructies (Raspberry Pi)

1. **Installeer vereisten**
   ```bash
   sudo apt update
   sudo apt install python3-pip bluetooth -y
   pip install spherov2 pygame
   ```

2. **Verbind de Sphero BOLT via Bluetooth**
   - Zet de BOLT aan en noteer de naam (bijv. `SB-27A5`).
   - Pair via het Bluetooth-menu of laat het script zelf scannen.

3. **Voer het script uit**
   ```bash
   python3 sphero_race.py <toy_name> <joystickIndex> <playerNumber>
   ```
   Voorbeeld:
   ```bash
   python3 sphero_race.py SB-27A5 0 1
   ```

4. **Joystickbediening**
   - 🎮 **SELECT** → kalibratie aan/uit  
   - 🏁 **START** → start automatische race  
   - 🔘 **1–4** → snelheidspreset (50 / 70 / 100 / 200)  
   - 🕹️ Stick → handmatige besturing  

---

## 🧭 Kalibratiestappen

1. Plaats de Sphero BOLT op de **startpositie (finishlijn, bovenaan in het midden)**.  
2. Richt de robot zodat hij **naar rechts (oost)** kijkt → dat is **0° heading**.  
3. Druk op **SELECT** om in kalibratiemodus te gaan (LED = roze).  
4. Gebruik de joystick links/rechts om de richting bij te stellen.  
5. Druk opnieuw op **SELECT** om te bevestigen (LED = groen → klaar).  
6. Druk op **START** om de automatische track te starten.

---

## 🧠 Beschrijving van de aanpak

- De race is opgebouwd uit **segmenten (waypoints)** die elk bestaan uit:
  - `panels`: aantal tegels van 50 cm per stuk (afstand)
  - `angle`: de heading in graden  
    - 0° → rechts (vooruit)  
    - 90° → omhoog  
    - 180° → links  
    - 270° → omlaag  

- Het script rekent elk segment om naar een **tijd (seconden)** via `SPEED_FACTOR`.  
  De functie `make_moves()` vertaalt het pad naar `(heading, speed, duration)`.  

- De route (`TRACK_PATH`) volgt een vaste baan op het parcours:  
  rechte stukken en bochten worden automatisch uitgevoerd in volgorde.  

- **Fail-safe:** bij een fout of verbroken verbinding stopt de BOLT onmiddellijk  
  (`set_speed(0)`) en kleurt rood (veiligheidsstatus).

---

## ⚡ Snelheidsprofiel

- **TRACK_SPEED = 120** – standaard snelheid voor automatische segmenten  
- **Handmatige snelheid:** instelbaar via joystickknoppen 1 t/m 4  
- Snelheid kan verder gekalibreerd worden door `SPEED_FACTOR` te wijzigen  
  (hoger = meer afstand per seconde)

---

> 🎯 **Resultaat:** De Sphero rijdt zelfstandig het volledige parcours af via vooraf bepaalde segmenten, met correcte oriëntatie op basis van kalibratie en automatische fail-safe bij fouten.

