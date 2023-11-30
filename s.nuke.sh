echo "🖥️ killing python3 instances"
pkill -f 'python3 o\..*'
echo "🖥️ removing temporary files"
rm /tmp/o.memory.txt
rm /tmp/o.memory.lock
rm /tmp/o.image.*
rm /tmp/o.audio.*
echo "🖥️ removing environment variables"
printenv | grep '^O_' | while read -r line; do
    var_name=$(echo "$line" | cut -d'=' -f1)
    unset "$var_name"
    echo "unset $var_name"
done