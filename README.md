# MegaCNC
Arduino Mega based CNC controller
Windows independed CNC soft

DISCLAIMER
THE AUTHOR OF THIS SOFTWARE ACCEPT ABSOLUTELY NO LIABILITY FOR ANY HARM OR LOSS RESULTING FROM ITS USE.
IT IS EXTREMELY UNWISE TO RELY ON SOFTWARE ALONE FOR SAFETY.
Any machinery capable of harming persons must have provisions for completely removing power from all motors, etc, before persons enter any danger area.
All machinery must be designed to comply with local and national safety codes, and the authors of this software can not, and do not, take any responsibility for such compliance.

Основная идея проекта это переход от компьютера, управляющего станком с ЧПУ к управлению от контроллера Ардуино Мега.
Изначально даже подумывал об Ардуино Нано! Но вовремя остановился. :)
К тому же эта задача является неплохой тренировкой для управления портами и регистрами ардуины напрямую.

Исходные данные:
1. Ардуино Мега:
2. LCD shield (16x2) c 4-мя кнопками
3. Дайвер моторов TB-3
4. Станок с ЧПУ

Задача:
  Научить Мегу управлять станком не хуже чем LinuxCNC для своих задач, с учетом опыта работы с LinuxCNC и Mach3.
  
Профит: 
  1. Экономия энергии
  2. Повышение надёжности при сбоях электропитания
  3. Контроль над процессом
  4. Возможность расширить базу за счет модных фенек ардуины 
  4. SKILLUP


