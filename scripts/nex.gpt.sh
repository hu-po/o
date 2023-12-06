export O_DEATH=20
sh nuke.sh
python3 ../o.py --node body --model_api gpt --robot nex &
python3 ../o.py --node look --model_api gpt &
python3 ../o.py --node plan --model_api gpt &
python3 ../o.py --node talk --model_api gpt