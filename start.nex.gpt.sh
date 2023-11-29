rm /tmp/o.memory.txt
rm /tmp/o.memory.lock
python3 run.talk.py --model_api gpt &
python3 run.plan.py --model_api gpt &
python3 run.move.py --model_api gpt &
python3 run.body.py --model_api gpt --robot nex
wait