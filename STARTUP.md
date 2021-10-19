
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

## Aruco mapping

Video settings: `v4l2-ctl -c focus_auto=0 -c saturation=0 -c contrast=10 -c brightness=50`.

Connect to Putzini via NoMachine. If you don't see it in the connection window, connect via SSH and run `sudo /usr/NX/nxserver --restart`. Password is `lovearka`.
On Putzini, do NOT start `hover.py`, but `hover_manual.py`. This will allow you to drive around during the mapping manually.

Once you have logged in via NoMachine, start a terminal. Navigate to folder: `cd ~/nessundorma`. Then start the camera using `aruco_test live:0 -c microsoft.yml` and check if the markers are found. You can furthermore fiddle with the settings.

If it looks well, start the mapping program by:

```
mapper_from_video live calib_usbgs/usbgs.yml <MARKERSIZE> -out <NAME OF THEATER> -d ARUCO_MIP_16h3
```

where `<MARKERSIZE>` is the size of the markers in m, typically e.g. 17.3, and `<NAME OF THEATER>`... guess yourself.

Now drive around, watching the video screen, until you think you have seen all markers. Then, press Esc. A lengthy computation will start in the command window. Have a coffee. After it's done, a window with a 3D rendering of the marker positions will appear. You can scroll and zoom around, to check if the result of the marker positions make sense. Hit Esc again.
You will now have a file `<NAME OF THEATER>.yml` which contains the mapping information.
Enter that filename as `marker_map` in `putzini.yaml`.

## Putzini navigation bugfixing

If you do not see three/four distance circles in the Putzini UI, something is wrong with the config of the navigation transmitters.

As a first attempt of fixing, quit `hover.py` if it is still running (`Ctrl-C`), and (in the `nessundorma` folder), run `./putzini_nav.py`.
This should fully re-configure the navigation transmitters or at least give some useful error messages.

If that does not work, unplug all USB transmitters **including** the one on Putzini itself, wait for 3 minutes, replug them, and then run `./putzini_nav.py`.

## Putzini shut-down

For a graceful shutdown of Putzini, please do the following:
- In the terminal running the Putzini software within `tmux` (green bar at the bottom), hit `Ctrl-C` to stop the Putzini code.
- Run `putzini_nav.py` in the same window. This will shutdown the navigation system cleanly.
- Run `sudo shutdown now`. It will ask for the password, which is `lovearka`.
- Wait a minute, then switch Putzini off using the rocker switch (to down position) and the pusb button for the motor ("downwards" sound)

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
- start unity hub, load project `ArkaTest19`. It will likely show an error about window layout. **DO NOT CHOOSE 'STANDARD LAYOUT'**. Instead, just quit.
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

## Speech synthesis
**...only works on a Mac**

...is required if anything changes in the "Regie" lines.
- Generate a json story file as described above... just call it `story.json`.
- run: `./record_speech.py`. This will generate `.wav` sound files in the subfolder `speech`. They are named `<xxx>.wav`, where `<xxx>` is the md5-hash of the text that is spoken. If that file already exists, it will not be re-generated.
- upload the `speech` folder to the projector computer using this command:
    ```
    rsync -avz speech pi@beamer-pi:/home/pi/nessundorma
    ```
## Projector
The Raspberry Pi should automatically switch on and be properly configured... mostly (see below).

After a while, you should see the Desktop interface on the projection. 
Connect to the Raspberry Pi using NoMachine.
It is called `beamer-pi`.
If you do not see it, seach also for its IP: `172.31.1.131`.
user: pi
password: lovearka
sound checkbox off (don't check!)
You will see the desktop (as shown on the projector) mirrored on your screen.
Now double-click the file `start_beamer.sh`on the Desktop and choose `Execute`.
After a short bit the screen should go black.
move the curor out of the screen pls
Then, **click on the black screen once**.
Now, all is ready.
You can check it in the Arka interface using the rightmost control tab ("clipboard" icon), from where you can send test messages from the different actors.

<!-- After a while, the screen should become black, and you should see the test messages that you can send from the ARKA computer.

..._except_ it wll not play sound unless you do one mouse click.
To do so, start `VNC Viewer` on the small MacBook, connect to the Raspberry Pi, and click once. -->