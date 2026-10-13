"""Check that every ```matlab listing in graphslam_code_walkthrough.md is a verbatim excerpt of the source.

    python Graph_SLAM/Documents/images/check_code_walkthrough_snippets.py

A listing passes if its lines (ignoring trailing spaces) appear as consecutive lines in
SIMULATION_GraphSLAM.m. A line that is exactly "% ..." splits a listing into pieces that are each checked
separately and in order. Exits with an error if any listing has drifted from the code.
"""
import os
import re
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
DOC = os.path.join(HERE, "..", "graphslam_code_walkthrough.md")
SRC = os.path.join(HERE, "..", "..", "SIMULATION_GraphSLAM.m")


def read_lines(path):
    with open(path, encoding="utf-8") as f:
        return [line.rstrip() for line in f.read().splitlines()]


def listings(doc_lines):
    inside, current = False, []
    for line in doc_lines:
        if re.match(r"^```matlab\s*$", line):
            inside, current = True, []
        elif inside and line.startswith("```"):
            inside = False
            yield current
        elif inside:
            current.append(line)


def find_block(source, block, start):
    n = len(block)
    for i in range(start, len(source) - n + 1):
        if source[i:i + n] == block:
            return i
    return -1


def main():
    source = read_lines(SRC)
    failures, count = [], 0
    for listing in listings(read_lines(DOC)):
        count += 1
        pieces, piece = [], []
        for line in listing:
            if line.strip() == "% ...":
                pieces.append(piece)
                piece = []
            else:
                piece.append(line)
        pieces.append(piece)
        position = 0
        for piece in pieces:
            if not piece:
                continue
            at = find_block(source, piece, position)
            if at < 0:
                failures.append("\n".join(piece))
                break
            position = at + len(piece)
    if failures:
        print(f"{len(failures)} of {count} listings do not match {os.path.basename(SRC)}:\n")
        for text in failures:
            print(text, "\n" + "-" * 60)
        sys.exit(1)
    print(f"All {count} MATLAB listings match {os.path.basename(SRC)} verbatim.")


if __name__ == "__main__":
    main()
