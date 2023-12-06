export O_DEATH=20
sh s.nuke.sh
python3 o.node.py --node body --model_api gpt --robot nex &
python3 o.node.py --node look --model_api gpt &
python3 o.node.py --node plan --model_api gpt &
python3 o.node.py --node talk --model_api gpt