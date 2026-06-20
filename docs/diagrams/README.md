# Diagrams

Architecture diagrams for the teleop layer, authored in [d2](https://d2lang.com).
Each `.d2` file is the source of truth; the matching `.svg` is a generated
artifact referenced by the docs. Edit the `.d2`, then re-render.

## Render

```bash
./render.sh          # renders every *.d2 to *.svg with the brand font
```

Needs `d2` on `PATH`. The font ships in [`fonts/`](fonts/), so rendering is
self-contained. The script passes the font explicitly:

```bash
d2 --layout elk --font-regular fonts/GeistMono-VF.ttf \
   --font-bold fonts/GeistMono-VF.ttf --font-italic fonts/GeistMono-VF.ttf in.d2 out.svg
```

## Palette + Grammar

Brand palette mirrors firstmotive.ai, defined once in [`styles.d2`](styles.d2),
imported with `...@styles`. Node graphs use `node` (plum box) + `topic` (cream
pill); stub nodes carry a dashed border. Full token table and block grammar:
[fm-robot/docs/diagrams](https://github.com/first-motive/fm-robot/blob/main/docs/diagrams/README.md).

## Diagrams

```
contract   every input source → fixed contract channels → control-stack sinks
```

The contract is the whole point of this layer: sources never hard-code topics,
sinks subscribe fixed channels, so an input device swaps without touching anything
downstream. See [ARCHITECTURE.md](../ARCHITECTURE.md).
