sh s.nuke.sh
export O_DEATH=30
export O_NEX_MOVE_ENABLED=0
python3 o.body.py --model_api gpt --robot nex &
python3 o.look.py --model_api gpt &
python3 o.quiet.py --model_api gpt