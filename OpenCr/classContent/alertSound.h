#ifndef ALERT_SOUND_H
#define ALERT_SOUND_H

#include "pitches.h"

// 🚨 Play a low-battery alert sound
void playLowBatteryAlert() {
  for (int i = 0; i < 3; i++) {
    tone(BDPIN_BUZZER, NOTE_A6, 250);  // high beep
    delay(300);
    tone(BDPIN_BUZZER, NOTE_F6, 250);  // lower beep
    delay(300);
    noTone(BDPIN_BUZZER);
  }

  // Final long tone
  tone(BDPIN_BUZZER, NOTE_B5, 800);
  delay(900);
  noTone(BDPIN_BUZZER);
}

#endif
