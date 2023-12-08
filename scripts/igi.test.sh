set -e
echo "  🖥️   testing igi"
export DISPLAY=:0
xdotool key shift
echo "  🖥️    testing with model_api gpt and robot igi"
sh scripts/nuke.sh
export O_DEATH=20
export O_MAX_STEPS=3
export O_NEX_MOVE_ENABLED=0
python3 o.py --node body --model_api gpt --robot igi &
python3 o.py --node look --model_api gpt --robot igi &
python3 o.py --node quiet --model_api gpt --robot igi

echo "  🖥️    testing with model_api rep and robot igi"
sh scripts/nuke.sh
export O_DEATH=20
export O_MAX_STEPS=3
export O_NEX_MOVE_ENABLED=0
python3 o.py --node body --model_api rep --robot igi &
python3 o.py --node look --model_api rep --robot igi &
python3 o.py --node quiet --model_api rep --robot igi

status=$?
if [ $status -ne 0 ]; then
    echo "  🖥️   ❌ testing failed with exit status $status"
    exit $status
else
    echo "  🖥️   ✅ testing completed"
fi