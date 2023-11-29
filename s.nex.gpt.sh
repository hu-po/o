sh s.nuke.sh
python3 o.talk.py --model_api gpt &
python3 o.plan.py --model_api gpt &
python3 o.move.py --model_api gpt &
python3 o.body.py --model_api gpt --robot nex
wait