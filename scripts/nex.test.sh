set -e

echo "🖥️ testing with gpt model_api and nex robot"
export O_DEATH=20
export O_MAX_STEPS=3
export O_NEX_MOVE_ENABLED=0
sh nuke.sh
python3 ../o.py --node body --model_api gpt --robot nex &
python3 ../o.py --node look --model_api gpt &
python3 ../o.py --node plan --model_api gpt &
python3 ../o.py --node talk --model_api gpt

echo "🖥️ testing with rep model_api and nex robot"
export O_DEATH=20
export O_MAX_STEPS=3
export O_NEX_MOVE_ENABLED=0
sh nuke.sh
python3 ../o.py --node body --model_api rep --robot nex &
python3 ../o.py --node look --model_api rep &
python3 ../o.py --node plan --model_api rep &
python3 ../o.py --node talk --model_api rep

status=$?
if [ $status -ne 0 ]; then
    echo "🖥️❌ testing failed with exit status $status"
    exit $status
else
    echo "🖥️✅ testing completed"
fi