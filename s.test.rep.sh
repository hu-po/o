export DEATH=60
sh s.nuke.sh
python3 o.body.py --model_api rep --robot test &
python3 o.look.py --model_api rep &
python3 o.plan.py --model_api rep &
python3 o.talk.py --model_api rep