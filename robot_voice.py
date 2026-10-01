"""Offline Vietnamese greetings through the robot's default audio output."""
import shutil
import subprocess
import threading
import os
import tempfile


class TargetAnnouncement(object):
    """Announce the selected YOLO class once per encounter."""
    def __init__(self, voice):
        self.voice = voice
        self.target = None
        self.last_seen = None
        self.streak = 0
        self.announced = False
        self.last_attempt = -100.0
        self.message = ''

    def update(self, target, found, now, enabled=True):
        if target != self.target:
            self.target = target
            self.last_seen = None
            self.streak = 0
            self.announced = False
            self.message = ''
            self.last_attempt = -100.0
        if not enabled:
            self.streak = 0
            return False
        if self.last_seen is not None and now - self.last_seen > 10.0:
            self.announced = False
            self.message = ''
        if not found:
            self.streak = 0
            return False
        self.streak = self.streak + 1 if self.last_seen is not None and now - self.last_seen <= 2.0 else 1
        self.last_seen = now
        if self.voice.error:
            self.announced = False
        if self.streak < 2 or self.announced or now - self.last_attempt < 5.0:
            return False
        self.last_attempt = now
        message = ('Đã tìm thấy người. Xin chào bạn.' if target == 'person'
                   else 'Đã tìm thấy {}.'.format(target))
        if self.voice.greet(message):
            self.announced = True
            self.message = message
            return True
        return False


class RobotVoice(object):
    def __init__(self, logger):
        self.engine = shutil.which('espeak-ng') or shutil.which('espeak')
        self.error = '' if self.engine else 'Missing espeak: sudo apt-get install espeak'
        self._logger = logger
        self._lock = threading.Lock()
        self.output_device = ''
        if self.error:
            self._logger(self.error)

    def greet(self, text='Xin chào bạn.'):
        if not self.engine or not self._lock.acquire(False):
            return False
        worker = threading.Thread(target=self._play, args=(text,))
        worker.daemon = True
        try:
            worker.start()
        except Exception:
            self._lock.release()
            raise
        return True

    def _play(self, text='Xin chào bạn.'):
        try:
            # SSH sessions inherit the Jetson platform output as default.
            # Select the USB speaker explicitly without changing system audio.
            player = shutil.which('paplay')
            pactl = shutil.which('pactl')
            if not player or not pactl:
                raise RuntimeError('Missing PulseAudio tools: sudo apt-get install pulseaudio-utils')
            sinks = self._run([pactl, 'list', 'short', 'sinks']).decode('utf-8', 'replace')
            names = [line.split()[1] for line in sinks.splitlines() if len(line.split()) >= 2]
            selected = os.environ.get('JETAUTO_AUDIO_SINK', '')
            if not selected:
                candidates = [name for name in names if 'usb-GeneralPlus_USB_Audio_Device' in name]
                if not candidates:
                    candidates = [name for name in names if 'alsa_output.usb-' in name]
                if len(candidates) != 1:
                    raise RuntimeError('Cannot select USB speaker; set JETAUTO_AUDIO_SINK')
                selected = candidates[0]
            if selected not in names:
                raise RuntimeError('Audio sink unavailable: ' + selected)
            self.output_device = selected
            with tempfile.TemporaryDirectory(prefix='jetauto_voice_') as folder:
                wav = os.path.join(folder, 'greeting.wav')
                self._run([self.engine, '-v', 'vi', '-s', '145', '-w', wav,
                           text])
                self._run([player, '--device=' + selected, wav])
            self.error = ''
        except Exception as error:
            self.error = str(error)
            self._logger('Robot speaker failed: %s' % self.error)
        finally:
            self._lock.release()

    @staticmethod
    def _run(command):
        result = subprocess.run(command, stdout=subprocess.PIPE,
                                stderr=subprocess.PIPE, timeout=15)
        if result.returncode:
            raise RuntimeError(result.stderr.decode('utf-8', 'replace')[-500:])
        return result.stdout
