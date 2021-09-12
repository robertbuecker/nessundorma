# nessundorma

Tools for robot operas. 
* Grab a google docs link and make a JSON, with fun data mangling as needed. (`tbl2json.py`)
* Putzini software and calibrations, incl. modified ArUco file and set of used markers as PNGs. (`hover.py`, `calib_usbgs`)
* Putzini client GUI (`plotLive.py`)
* Music control script (`control_music.py`)

# Markus' ARKA instructions

- Kompressor -> Wasser raus
- Kompressor anschalten
- KRC anschalten
- SmartPad 
  - auf Externen Modus schalten (Schlesselschalter)
  - Motoren einschalten bei den Vier symbolen oben auf das O druecken und dann I 
- Laptop anmachen


- Google Doc konvertieren
  - CMD
  - cd C:\Users\markus\Documents\GitHub\nessundorma
  - python tbl2json.py https://docs.google.com/spreadsheets/d/1OhzEmibkJMFbsruqyJ8bs3RpznTwPGjXmI10UIOyNr0 "C:\Users\markus\Desktop\Nessun Dorma Stories\forum-stadtpark.json"
  - python tbl2json.py https://docs.google.com/spreadsheets/d/1YD_SuiSU65OZHTQqJl-264Gbm5Re9Fdzw8rxpyc-V8U "C:\Users\markus\Desktop\Nessun Dorma Stories\forum-stadtpark.json"

  - Mal-Loop: python tbl2json.py https://docs.google.com/spreadsheets/d/1cTVTUAAv6rOKKnQBwJ-0xe22A099qP58Q5T8udukMVk "C:\Users\markus\Desktop\Nessun Dorma Stories\Final-Mal-Loop.json"
  - Show:     python tbl2json.py https://docs.google.com/spreadsheets/d/1YD_SuiSU65OZHTQqJl-264Gbm5Re9Fdzw8rxpyc-V8U "C:\Users\markus\Desktop\Nessun Dorma Stories\Final-Show.json"

### Hamburg 

https://docs.google.com/spreadsheets/d/1BTUcaonniEF3Fmb19ldBlSeylyghuEIvVK9wKNdwFXI/edit#gid=0

cd C:\Users\markus\Documents\GitHub\nessundorma
python tbl2json.py https://docs.google.com/spreadsheets/d/1BTUcaonniEF3Fmb19ldBlSeylyghuEIvVK9wKNdwFXI C:\Users\markus\Desktop\Nessun-Dorma-Stories\Hamburg-01.json



- Unity starten
  - Unity Hub starten
  - (wahrscheinlich 2 mal, weil beim ersten Mal ein Fehler)
  - einfach oben Mitte auf Play druecken um die App zu starten


- Beamer starten
  - der Pi sollte bereits an sein, ansonsten checken und ggf. Kabel reinstecken
  - der Pi bootet und started automatisch den Browser mit der gewuenschten Adresse
    - aber nicht im Fullscreen Modus, weil immer noch einige Dinge haendisch gemacht werden muessen
    - Sound: rechte Maustaste auf das Audio Symbol oben rechts, das muss auf "Analog" stehen
      - es muss (nach einem Neustart) mindestens einmal in dem Browserfenster geklickt werden (sonst funktionieren die Notifications nicht) (die Maus liegt auf dem Regietisch)
    - Fullscreen Modus aktivieren durch druecken von F11 auf der Tastatur (auch auf dem Regietisch)

80x55x30

