import replicate


output = replicate.run(
    "openai/whisper:4d50797290df275329f202e48c76360b3f22b08d28c196cbc54600319435f8d2",
    input={"audio": open("path/to/file", "rb")},
)
print(output)

output = replicate.run(
    "meta/llama-2-13b-chat:f4e2de70d66816a838a89eeeb621910adffb0dd0baba3976c96980970978018d",
    input={"prompt": ...},
)
# The meta/llama-2-13b-chat model can stream output as it's running.
# The predict method returns an iterator, and you can iterate over that output.
for item in output:
    # https://replicate.com/meta/llama-2-13b-chat/versions/f4e2de70d66816a838a89eeeb621910adffb0dd0baba3976c96980970978018d/api#output-schema
    print(item, end="")


output = replicate.run(
    "suno-ai/bark:b76242b40d67c76ab6742e987628a2a9ac019e11d56ab96c4e91ce03b79b2787",
    input={
        "prompt": "Hello, my name is Suno. And, uh \u2014 and I like pizza. [laughs] But I also have other interests such as playing tic tac toe."
    },
)
print(output)

output = replicate.run(
    "yorickvp/llava-13b:2facb4a474a0462c15041b78b1ad70952ea46b5ec6ad29583c0b29dbd4249591",
    input={"image": open("path/to/file", "rb")},
)
# The yorickvp/llava-13b model can stream output as it's running.
# The predict method returns an iterator, and you can iterate over that output.
for item in output:
    # https://replicate.com/yorickvp/llava-13b/versions/2facb4a474a0462c15041b78b1ad70952ea46b5ec6ad29583c0b29dbd4249591/api#output-schema
    print(item, end="")
