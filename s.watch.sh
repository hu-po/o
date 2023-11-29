get_formatted_output() {
    echo "@@@@@@@@@@@"
    echo "🖥️ o python ps:"
    echo "@@@@@@@@@@@"
    ps aux | grep "python3" | grep -v grep
    echo "@@@@@@@@@@@"
}
while true; do
    clear
    get_formatted_output
    sleep 0.1
done