def import_node(name: str) -> dict:
    if name == "body":
        from nodes.body import EMOJI, loop
    elif name == "look":
        from nodes.look import EMOJI, loop
    elif name == "plan":
        from nodes.plan import EMOJI, loop
    elif name == "quiet":
        from nodes.quiet import EMOJI, loop
    elif name == "listen":
        from nodes.listen import EMOJI, loop
    elif name == "talk":
        from nodes.talk import EMOJI, loop
    else:
        from nodes.test import EMOJI, loop
    print(f"🖥️ {EMOJI} starting node {name}")

    return {"emoji": EMOJI, "loop": loop}