get_formatted_output() {
    echo "  🖥️    Running python3 instances:"
    # Using ps with custom format to show memory usage (%mem) and elapsed time (etime)
    ps aux --sort=-%mem | awk '/python3 o\..*/ && !/grep/ {print $0, "Memory:", $4"%", "Duration:", $10}'
    echo "  🖥️    memory:"
    cat /tmp/o.memory.txt
    echo "  🖥️    files:"
    ls -lht /tmp/o.*
    echo "  🖥️    params:"
    printenv | grep '^O_' | while read -r line; do
        echo "$line"
    done
}

while true; do
    clear
    get_formatted_output
    sleep 4.2
done
