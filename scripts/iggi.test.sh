set -e

echo "🖥️ testing with model_api gpt and robot iggi"
sh scripts/nuke.sh
export O_DEATH=20
export O_MAX_STEPS=3
export O_NEX_MOVE_ENABLED=0
python3 o.py --node body --model_api gpt --robot iggi &
python3 o.py --node look --model_api gpt --robot iggi &
python3 o.py --node plan --model_api gpt --robot iggi &
python3 o.py --node talk --model_api gpt --robot iggi

echo "🖥️ testing with model_api rep and robot iggi"
sh scripts/nuke.sh
export O_DEATH=20
export O_MAX_STEPS=3
export O_NEX_MOVE_ENABLED=0
python3 o.py --node body --model_api rep --robot iggi &
python3 o.py --node look --model_api rep --robot iggi &
python3 o.py --node plan --model_api rep --robot iggi &
python3 o.py --node talk --model_api rep --robot iggi

status=$?
if [ $status -ne 0 ]; then
    echo "🖥️❌ testing failed with exit status $status"
    exit $status
else
    echo "🖥️✅ testing completed"
fi