# Robots

You can optionally use different robot backends with the `--robot` flag.

## Nex Humanoid `--robot nex`

Nex is a HiWonder AiNex Humanoid running ROS+Ubuntu on a Raspberry Pi 4B. You will need to install some dependencies in the base system python3 env:

```
pip install opencv-python==4.8.1.78
pip install scipy==1.11.4
pip install filelock==3.13.1
```

Install any model api dependencies [see model docs](models.md).

Audio dependency soundevice requires pyaudio which requires portaudio

```
sudo apt-get install libportaudio2
sudo apt-get install python3-pyaudio
pip install sounddevice==0.4.6
pip install pydub==0.25.1
```

To autostart script on robot boot use crontab (sleep for some time to allow ROS to spin up):

```
chmod +x /home/ubuntu/o/o.sh
crontab -e
@reboot bash /home/ubuntu/o/scripts/nex.quiet.sh
*/5 * * * * /home/ubuntu/o/scripts/nex.quiet.sh
```

Only a USB microphone works, set the volume with:

```
aplay -l
amixer -c 2 set PCM 100%
```

Have to fix the ALSA config to make the USB mic the default:

```
sudo vim /etc/asound.conf
sudo alsa force-reload
```
```
pcm.!default {
    type hw
    card 2
}
ctl.!default {
    type hw
    card 2
}
```

Test the audio with python3

```
from pydub import AudioSegment
from pydub.playback import play
seg = AudioSegment.from_file("/tmp/tmp0e449aae8f.mp3", "mp3")
play(seg)
```

## Igi Observer `--robot igi`

Igi is a stationary observer robot running Debian on a Raspberry Pi 4B. It uses three Dynamixel servos for pan-tilt-yaw and `cv2` to capture stereo images.