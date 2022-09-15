# Startup 

##  ARKA hardware switch on

- Drain water from compressor and switch it on
- Switch on KUKA mains switch (back of rack)
- SmartPad 
  - Turn key, switch to EXT mode, turn key back
  - Tap "0" button on top, switch on motors

## ARKA computer switch on

If not running, switch on the main control computer. User `?`, pw `lovearka`.
Start the ARKA software (Desktop shortcut).

**ALL OF THE REST OF ARKA IS OUTDATED, REWRITE IT FOR THE NEW PC**

<!-- OUTDATED
If not already running, start the Unity software:
- start unity hub from the taskbar shortcut, load project `ArkaTest19`. It will likely show an error about window layout. **DO NOT CHOOSE 'STANDARD LAYOUT'**. Instead, just quit.
- repeat. Second time will work.
- Click "Play" button on top center.
- The ARKA user interface should appear now -->

If required (that is, if the show sequence on Google Docs has changed):
- Start a command prompt terminal (CMD in search field or shortcut on taskbar)
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
- Hamburg Lichthof:
    ```
    0.5
    ```

  Load the show into the main Arka sequencer in the interface on the left side on the second tab from the right.

## Putzini hardware switch-on
- Unplug all chargers and if required re-install the loudspeaker. **MAKE SURE TO LEAVE 2CM BETWEEN LOUDSPEAKER AND FRONT OF PUTZINI**... or the threaded rod of the neck motor will get stuck. You do not want that.
- Make sure the internal loudspeaker is switched on (button on the far side) and connected (3.5" jack)
- Make sure the vacuum cleaner is connected (three-pole wire, green must be matched with brown)
- Switch on the logic boards (upper lever switch up!)
- Switch on the drive board (lower push button: "upwards pitched" sound!)

## Putzini and music computer

**ALL OF THIS IS OUTDATED, THIS IS NOW HAPPENING ON THE MAIN COMPUTER AND/OR THE RASPI 4**

If not running, start small MacBook Pro. user `antonie`, pw `20L07`.
- Make sure it is connected to the `NESSUN DORMA` WiFi network.
- If using Bluetooth to transmit the music, check that Bluetooth is connected to `blueLino 4G+`, and that the sound output (`alt-Click` on sound level control) is set to `blueLino 4G+`.
- Otherwuse, the sound output should be set to the normal MacBook headphone out or whichever interface you are using.

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

#### Log into Putzini's internal computer (Jetson)
Start by 
```
ssh putzini@putzini
```
After a short while, the command prompt should change to `putzini@jetputz:~$`.
You are now on Putzini's internal computer.

#### Start or attach to a `tmux` session
Putzini's software needs to run within a `tmux` session (recognizable by a green bar at the bottom of the terminal window). This prevents Putzini from shutting down if you log out from the control computer or lose the connection.
The next step depends on whether Putzini is already running (see above) or you start it for the first time.
- If it is running already, just call 
    ```
    tmux attach
    ``` 
    The lowest line should turn green and you will get a display of log messages from Putzini.
- If it is not running, run 
    ```tmux```
    (without the `attach`!). A green bar wil appear at the bottom. 
Now, run
```
cd nessundorma
./hover.py
``` 
...it will now print a lot of status messages, run some tests, drive the neck down to zero position, do a quick nod with the head, reset the LEDs etc. Finally, you should see Putzini's position and direction properly in the graphical status window that you started above. Putzini is now ready to go!

## Projector

**ALSO OUTDATED, PROJECTOR IS NOW CONNECTED TO RASPI 4**

The Raspberry Pi (the black one with "TOTO.io" on it) should automatically switch on and be properly configured... mostly (see below). Best is to always leave it running, then you have no reason to worry.
You can try if it all works by clicking the buttons corresponding to the different roles in Arka's control window on the left side, in the rightmost tab.
Make sure you see the dialogue messages on the projector, and that the sounds come from the Raspberry Pi's output (which, obviously, you will need to connect first).

If the Raspberry Pi was switched off or other things go wrong, you will likely not get the sounds.
In this case, connect to the Raspberry Pi using NoMachine... and call Robert.
<!-- It is called `pggo01`.
If you do not see it, seach also for its IP: `172.31.1.131`.
user: `pi`,
password: `lovearka`,
sound checkbox off (don't check!).
You will see the desktop (as shown on the projector) mirrored on your screen, but not see much as it's just black (Web Browser in Kiosk mode).
Now double-click the file `start_beamer.sh`on the Desktop and choose `Execute`.
After a short bit the screen should go black.
move the curor out of the screen pls
Then, **click on the black screen once**.
Now, all is ready.
You can check it in the Arka interface using the rightmost control tab ("clipboard" icon), from where you can send test messages from the different actors. -->

## Putzini operation

...using the graphical interface.

# Shutdown

## Putzini shut-down

For a graceful shutdown of Putzini, please do the following:
- In the terminal running the Putzini software within `tmux` (green bar at the bottom), hit `Ctrl-C` to stop the Putzini code.
- Run `./putzini_nav.py` in the same window. This will take about 10 seconds and shut down the navigation system cleanly.
- Run `sudo shutdown now`. It will ask for the password, which is `lovearka`.
- Wait a minute, then switch Putzini off using the rocker switch (to down position) and the push button for the motor ("downwards" sound)
- Switch off drive board (lower push button: "downwards pitched" sound)
- Charge speaker using USB-C cable. Either take it out, or fiddle the cable in while it is mounted. Unplug the audio cable while charging, or it will emit an ugly sound if you charge Putzini at the same time. Take care not to damage the vacuum cleaner and/or the antenna when opening Putzini (or take them off)
- Attach main charger to the external jack.
- Attach the vacuum cleaner charger. If you prefer, you can take the vacuum cleaner off (it's attached with velcro tape)
- Goodnight.
  
## ARKA hardware switch off

- **! important** remove brush from robot arm
- make sure compressor is on and provides air pressure
- boot PC and start `ND Sim`
  - connect robot
  - move robot to **transport position** (brush needs to detached!!!)
- shutdown PC
- SmartPad: 
  - turn key, set to T1, turn key back
  - Hit "User" button on the left column (lowest icon) -> Choose Administrator, pw `kuka` -> log on
  - Top left corner "robot" button -> Shutdown -> "Shut down control PC" -> Yes
  - Wait until the screen goes black after 1-2 minutes.
- Switch off power supply mains
- Switch off compressor
- detach all cables

# Stuff needed for installation
## Aruco mapping

Video settings for Global Shutter board camera in absolute exposure mode: `v4l2-ctl -c exposure_auto=1 -c contrast=95 -c exposure_absolute=5000 -c backlight_compensation=0`.

Video settings for Global Shutter board camera in adaptive mode: `v4l2-ctl -c exposure_auto=3 -c contrast=95 -c backlight_compensation=50 -c brightness=0`.

Video settings for Microsoft camera: `v4l2-ctl -c focus_auto=0 -c saturation=0 -c contrast=10 -c brightness=30`.

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


## Speech synthesis
**...only works on a Mac**

...is required if anything changes in the "Regie" lines.
- Generate a json story file as described above... just call it `story.json`.
- run: `./record_speech.py`. This will generate `.wav` sound files in the subfolder `speech`. They are named `<xxx>.wav`, where `<xxx>` is the md5-hash of the text that is spoken. If that file already exists, it will not be re-generated.
- upload the `speech` folder to the projector computer using this command:
    ```
    rsync -avz speech pi@beamer-pi:/home/pi/nessundorma
    ```

# Remote connections

## Connect to gateway via AnyDesk
## SSH via ngrok
- Start ngrok on control Raspbi running at `172.31.1.141` within a persistent `tmux` session:
    ```
    tmux
    ./ngrok tcp 22 --region=eu
    ```
- Note the forwarding address, e.g. `tcp://0.tcp.eu.ngrok.io:16712`. It will be different each time!
- On your local computer, you can now connect to the control Raspi using (in our example) `ssh pi@0.tcp.eu.ngrok.io -p 16712`
- To access the other computers (including Putzini), either connect via `ssh` within the first session, or (even better) use the control Raspi as a jump host: `ssh -J pi@0.tcp.eu.ngrok.io:16712 putzini@putzini`.
- To start a VSCode session on Putzini from home, add this to your `vscodeset` file (again, assuming our adress) and connect to `PutziniJump`:
  ```
  Host PutziniJump
    User putzini
    HostName 172.31.1.200
    ProxyCommand ssh pi@0.tcp.eu.ngrok.io -p 16712 netcat -q 3 %h %p
  ```
- You can combine all of this with port forwardings e.g. for NoMachine.

<!-- After a while, the screen should become black, and you should see the test messages that you can send from the ARKA computer.

..._except_ it wll not play sound unless you do one mouse click.
To do so, start `VNC Viewer` on the small MacBook, connect to the Raspberry Pi, and click once. -->
