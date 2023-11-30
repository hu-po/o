set -e

echo "🖥️ testing with test model_api and test robot"
export O_DEATH=6
export O_MAX_STEPS=6
sh s.nuke.sh
python3 o.body.py --model_api test --robot test &
python3 o.look.py --model_api test &
python3 o.plan.py --model_api test &
python3 o.talk.py --model_api test

echo "🖥️ testing with gpt model_api and test robot"
export O_DEATH=20
export O_MAX_STEPS=6
sh s.nuke.sh
python3 o.body.py --model_api gpt --robot test &
python3 o.look.py --model_api gpt &
python3 o.plan.py --model_api gpt &
python3 o.talk.py --model_api gpt

echo "🖥️ testing with rep model_api and test robot"
export O_DEATH=20
export O_MAX_STEPS=6
sh s.nuke.sh
python3 o.body.py --model_api rep --robot test &
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