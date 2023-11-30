set -e

echo "🖥️ testing with gpt model_api and nex robot"
export O_DEATH=20
export O_MAX_STEPS=3
sh s.nuke.sh
python3 o.body.py --model_api gpt --robot nex &
python3 o.look.py --model_api gpt &
python3 o.plan.py --model_api gpt &
python3 o.talk.py --model_api gpt

echo "🖥️ testing with rep model_api and nex robot"
export O_DEATH=20
export O_MAX_STEPS=3
sh s.nuke.sh
python3 o.body.py --model_api rep --robot nex &
python3 o.look.py --model_api rep &
python3 o.plan.py --model_api rep &
python3 o.talk.py --model_api rep

status=$?
if [ $status -ne 0 ]; then
    echo "🖥️❌ testing failed with exit status $status"
    exit $status
else
    echo "🖥️✅ testing completed"
fi