
## Putzini switch-on
- Make sure the internal loudspeaker is switched on (button on the far side) and connected (3.5" jack)
- Make sure the vacuum cleaner is connected (three-pole wire, green must be matched with brown)
- Switch on the logic boards (upper lever switch up!)
- Switch on the drive board (lower push button: "upwards pitched" sound!)

## Putzini switch-off
- Recommended: in Putzini terminal, stop the software by hitting `Ctrl-C`. Then, run `sudo shutdown now` and enter the password: `lovearka`.
- Switch off drive board (lower push button: "downwards pitched" sound)
- Switch off logic board (upper lever switch down)
- Charge speaker using USB-C cable. Either take it out, or fiddle the cable in while it is mounted. Take care not to damage the vacuum cleaner and/or the antenna when opening Putzini (or take them off)
- Attach main charger to external jack.
- If required, attach vacuum cleaner charger.
- Goodnight.

## Putzini and music computer
If not running, start small MacBook Pro. user `antonie`, pw `20L07`.
- Make sure it is connected to the `NESSUN DORMA` WiFi network.
- Check that Bluetooth is connected to `blueLino 4G+`.
- Check that the sound output (`alt-Click` on sound level control) is set to `blueLino 4G+`.

If computer was off entirely (not just sleeping), you will have to restart the command line terminals for Putzini and music programs.
For this, open `iTerm` (e.g. by just `cmd-Space` and searching for it), and start four tabs in the window using `cmd-T`.
In those, enter the following commands:
### Tab 1 (control)
```
cd nessundorma
./plotLive.py
```
A graphical window will appear which displays Putzini's position within the room on top of the no-go (black) and forbidden (grey) zones (that is, it should remain on the white area).

### Tab 2 (speech)
```
cd nessundorma
./say_via_mqtt.py
```
In the following, this tab will show status messages pertaining to the speech synthesizer.

### Tab 3 (music):
```
cd nessundorma
./music_via_mqtt.py
```
In the following, this tab will show status messages pertaining to the sound of the show.

### Tab 4 (Putzini)
This one is special as it connects to Putzini and runs code there.
Start by 
```
ssh putzini@putzini
```
After a short while, the command prompt should change to `putzini@jetputz:~$`.
You are now on Putzini's internal computer.
The next step depends on whether Putzini is already running (see above) or you start it for the first time.
- If it is running already, just call 
    ```
    tmux attach
    ``` 
    The lowest line should turn green and you will get a display of log messages from Putzini.
- If it is not running, run 
    ```tmux```
    (without the `attach`!). A green bar wil appear at the bottom. Now, run
    ```
    cd nessundorma
    ./hover.py
    ``` 
    ...it will now print a lot of status messages, run some tests, drive the neck down to zero position, do a quick nod with the head, reset the LEDs etc. Finally, you should see Putzini's position and direction properly in the status window. Putzini is now ready to go!

## Putzini operation

...using the graphical interface.

##  ARKA hardware switch on

- Drain water from compressor and switch it on
- Switch on KUKA mains switch (back of rack)
- SmartPad 
  - Turn key, switch to EXT mode, turn key back
  - Tap "0" button on top, switch on motors

## ARKA hardware switch off

- Unity control software: click "disconnect" -> ARKA's socket turns orange
- SmartPad: 
  - turn key, set to T1, turn key back
  - "User" button -> Administrator, pw kuka -> log on
  - Top left corner button -> Shutdown -> "Shut down control PC" -> Yes
- Switch off power supply mains
- Switch off compressor

## ARKA computer switch on

If not running, switch on the large MacBook Pro.

If not already running, start the Unity software:
- start unity hub, load project. It will likely crash with an error.
- repeat. Second time will work.
- Click "Play" button on top center.
- The ARKA user interface should appear now

If required (that is, if the show sequence on Google Docs has changed):
- Start a command prompt (CMD in search field)
- `cd C:\Users\markus\Documents\GitHub\nessundorma`
- run `tbl2json.py` with the Google Doc URL and the output file name as parameters. Here are common commands:
- Graz painting loop:
  ```
  python tbl2json.py https://docs.google.com/spreadsheets/d/1cTVTUAAv6rOKKnQBwJ-0xe22A099qP58Q5T8udukMVk "C:\Users\markus\Desktop\Nessun Dorma Stories\Final-Mal-Loop.json"
  ```
- Graz show:
  ```
  python tbl2json.py https://docs.google.com/spreadsheets/d/1YD_SuiSU65OZHTQqJl-264Gbm5Re9Fdzw8rxpyc-V8U "C:\Users\markus\Desktop\Nessun Dorma Stories\Final-Show.json"
  ```
- Hamburg xpon-art:
    ```
    python tbl2json.py https://docs.google.com/spreadsheets/d/1BTUcaonniEF3Fmb19ldBlSeylyghuEIvVK9wKNdwFXI C:\Users\markus\Desktop\Nessun-Dorma-Stories\Hamburg-01.json
    ```

## Projector
The Raspberry Pi should automatically switch on and be properly configured... mostly (see below).
After a while, the screen should become black, and you should see the test messages that you can send from the ARKA computer.

..._except_ it wll not play sound unless you do one mouse click.
To do so, start `VNC Viewer` on the small MacBook, connect to the Raspberry Pi, and click once.