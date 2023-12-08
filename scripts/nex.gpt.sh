echo "  🖥️    running robot nex with model_api gpt"
bash scripts/nuke.sh
bash params/nex.sh
bash params/gpt.sh
export O_DEATH=20
python3 o.py --node body --model_api gpt --robot nex &
python3 o.py --node look --model_api gpt --robot nex &
python3 o.py --node plan --model_api gpt --robot nex &
python3 o.py --node talk --model_api gpt --robot nex 