set -e

echo "🖥️ testing with test model_api and test robot"
export O_DEATH=3
export O_MAX_STEPS=3
sh s.nuke.sh
python3 o.node.py --node test --model_api test --robot test

echo "🖥️ testing with gpt model_api and test robot"
export O_DEATH=8
export O_MAX_STEPS=3
sh s.nuke.sh
python3 o.node.py --node test --model_api gpt --robot test

echo "🖥️ testing with rep model_api and test robot"
export O_DEATH=8
export O_MAX_STEPS=3
sh s.nuke.sh
python3 o.node.py --node test --model_api rep --robot test

status=$?
if [ $status -ne 0 ]; then
    echo "🖥️❌ testing failed with exit status $status"
    exit $status
else
    echo "🖥️✅ testing completed"
fi