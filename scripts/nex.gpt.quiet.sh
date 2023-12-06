sh nuke.sh
export O_DEATH=60
export O_NEX_MOVE_ENABLED=0
python3 ../o.py --node body --model_api gpt --robot nex &
python3 ../o.py --node look --model_api gpt &
python3 ../o.py --node quiet --model_api gpt