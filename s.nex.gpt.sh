export DEATH=120
sh s.nuke.sh
cleanup() {
    exit
}
trap cleanup SIGINT
python3 o.body.py --model_api gpt --robot nex &&
python3 o.look.py --model_api gpt &&
python3 o.plan.py --model_api gpt &&
python3 o.talk.py --model_api gpt && wait