models=(test gpt rep gem)
for model in "${models[@]}"; do
    echo "🖥️   testing with $model model"
    source scripts/nuke.sh
    if [ "$model" = "test" ]; then
            source params/defaults.sh
        elif [ "$model" = "gpt" ]; then
            source params/defaults.sh
            source params/gpt.sh
        elif [ "$model" = "rep" ]; then
            source params/defaults.sh
            source params/rep.sh
    else
        echo "🖥️ ❌ no tests for model $model"
        exit 1
    fi
    if ! python3 o.py --node test --model $model --robot test; then
        echo "🖥️ ❌ testing failed with $model"
        exit 1
    fi
done
echo "🖥️ ✅ testing completed"