import subprocess
import csv
import simpleaudio as sa
import asyncio
import os
import logging

logger = logging.getLogger(__name__)

class PutziniSound:
    def __init__(self, dev_name):
        self.wave = None
        self.play_obj = None
        self.dev = dev_name
        self.logger = logger
        try:
            # Activate the proper sound device
            self.logger.info('Activating sound device: %s', dev_name)
            assert subprocess.run(['pactl', 'set-default-sink', 
            dev_name]).returncode == 0
        except:
            self.logger.error('Could not initialize sound device %s', dev_name)

    async def play(self, fn, loop=False, vol=None):
        if self.play_obj is not None and self.play_obj.is_playing():
            self.play_obj.stop()
            self.play_obj = None

        self.logger.info(f'Loading wave file %s...', fn)
        self.wave = sa.WaveObject.from_wave_file(fn)

        fn_lbl = fn.rsplit('.', 1)[0] + '.txt'
        if os.path.isfile(fn_lbl):
            self.logger.info(f'Found corresponding label file %s', fn_lbl)
            with open(fn_lbl, newline='') as fh:
                reader = csv.DictReader(delimiter='\t')

        assert subprocess.run(['pactl', 'set-sink-volume', self.dev, f'{int(vol)}%']).returncode == 0            
        self.play_obj = self.wave.play()

        self.fn = ''

        while True:
            if (self.play_obj is not None) and self.play_obj.is_playing():
                await asyncio.sleep(0.5)
            elif (self.play_obj is not None) and loop: 
                self.logger.info(f'Restarting wave file %s.', fn)
                self.play_obj = self.wave.play()
            else:
                self.logger.info(f'Stopped wave file %s.', fn)
                break

    def stop(self):
        if self.play_obj is not None:
            self.play_obj.stop()
            self.play_obj = None

        self.wave = None
