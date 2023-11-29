sh s.nuke.sh
python3 o.talk.py --model_api test &
python3 o.plan.py --model_api test &
python3 o.move.py --model_api test &
python3 o.body.py --model_api test --robot nex
wait